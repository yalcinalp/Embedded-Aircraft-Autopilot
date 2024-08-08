import logging
from queue import PriorityQueue
import threading
import time
from enum import Enum, IntEnum
from itertools import count

from cmds import AltitudeCommand, AltitudePeriod, Command, DistanceCommand, EndCommand, LedCommand, LedValue, ManualCommand, PressCommand, SpeedCommand
from commandqueue import CommandQueue
from ui.enums import AltitudeZoneState

logger = logging.getLogger("agents")


class Agent:
    autopilot: "AutoPilot"

    def __init__(self, agents_config, autopilot):
        self.agents_config = agents_config
        # Unfortunate tight coupling...
        self.autopilot = autopilot

    def relative_time(self):
        return time.time() - self.agents_config.get("go-time", 0)

    def to_real_time(self, relative_time: float):
        return relative_time + self.agents_config.get("go-time", 0)

    def to_relative_time(self, real_time: float):
        return real_time - self.agents_config.get("go-time", 0)

    def process_cmd(self, timestamp: float, cmd: Command):
        logger.warning(f"Not implemented")

    def process_empty(self):
        logger.warning(f"Not implemented")

    def send_command(self, cmd: Command):
        self.autopilot.write(cmd)

    def update_screen(self, update: object):
        self.autopilot.update_screen(update)

    def finish(self):
        logger.debug(f"{type(self).__name__} has finished.")


class ThreadedAgent(Agent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.alive = True
        self.thread = threading.Thread(target=self.worker)
        self.thread.daemon = True

    def worker(self):
        logger.warning(f"Not implemented")

    def start(self):
        self.thread.start()

    def stop(self):
        self.alive = False

    def finish(self):
        self.stop()
        return super().finish()


class CommandDispatcherAgent(ThreadedAgent):
    """
    Blocks on the given command queue and dispatches commands to the all sub-agents.
    """

    def __init__(self, cmd_queue: CommandQueue, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sub_agents = []
        self.cmd_queue = cmd_queue

    def add_agent(self, agent):
        self.sub_agents.append(agent)

    def worker(self):
        while self.alive:
            pair = self.cmd_queue.get()
            if pair != None and type(pair[1]) == Command:
                # The mock command is put by stop() call
                logger.debug(f"CommandDispatcherAgent thread is exiting.")
                return
            for a in self.sub_agents:
                a: Agent
                if pair == None:
                    a.process_empty()
                else:
                    a.process_cmd(*pair)

    def stop(self):
        super().stop()
        # Put a mock command to wake up the thread so that it can exit
        self.cmd_queue.put(Command())

    def finish(self):
        if not self.alive:
            logger.debug(
                f"Cannot finish CommandDispatcherAgent as it is not alive.")
            return
        # Calls stop() and adds log
        super().finish()


class AlarmAgent(ThreadedAgent):
    """
    Makes best use of a single thread for creating alarms.
    Not tested, might be buggy.

    TODO Make this a proper singleton
    """
    ALARM_ERROR_THRESHOLD = 0.1   # second(s) error margin
    _INSTANCE = None
    unique = count()

    @staticmethod
    def instance():
        if AlarmAgent._INSTANCE == None:
            # AlarmAgent is context independent so no agents_config
            AlarmAgent._INSTANCE = AlarmAgent(None, None)
        return AlarmAgent._INSTANCE

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.alarms = PriorityQueue()
        # RLock is needed as the users may add alarms during an alarm
        self.sleep_lock = threading.RLock()
        self.sleep_cv = threading.Condition(self.sleep_lock)
        self.wake_time = time.time() * 2  # FIXME
        self.start()

    def add_alarm(self, on_alarm, timestamp: float, args=[]):
        if timestamp < time.time():
            logger.critical(f"Add alarm received an alarm for past!")
        try:
            self.alarms.put((timestamp, next(AlarmAgent.unique), on_alarm, args))
        except TypeError as ex:
            logger.critical(
                f"Failure to put an alarm into the queue!\n" +
                f"Details: {repr(ex)}")
            raise ex
        # Update the thread
        self.wake_time = self.alarms.queue[0][0]
        logger.debug(f"AlarmAgent new wake time is {self.wake_time}")
        with self.sleep_cv:
            # Wake it up so that it waits until the new wake time instead
            self.sleep_cv.notify()

    def process_queue(self):
        with self.sleep_lock:
            while not self.alarms.empty():
                if self.alarms.queue[0][0] <= time.time():
                    # Get removes it from the priority queue
                    ts, _, on_alarm, args = self.alarms.get()
                    logger.debug(f"AlarmAgent runs an alarm at {ts}")
                    on_alarm(*args)
                    if abs(ts - time.time()) > AlarmAgent.ALARM_ERROR_THRESHOLD:
                        logger.critical(f"Alarm was delivered at an erroneous time! " +
                                        f"Expected {ts}, delivered on {time.time()}")
                else:
                    self.wake_time = self.alarms.queue[0][0]
                    logger.debug(
                        f"AlarmAgent new wake time is {self.wake_time}")
                    return
            self.wake_time = time.time() * 2  # FIXME

    def worker(self):
        while self.alive:
            with self.sleep_cv:
                timeout = self.wake_time - time.time()
                if timeout < 0:
                    logger.critical(
                        f"Timeout is less than 0! Wake time: {self.wake_time}. Processing the queue immediately.")
                    self.process_queue()
                self.sleep_cv.wait(timeout)
            if self.alive:
                # If finished, do not process the queue. Anything added between finish() and now is garbage.
                self.process_queue()

    def finish(self):
        # NOTE Finishing this does not make much sense.
        # Make it not alive
        self.stop()
        # Empty the queue
        while not self.alarms.empty():
            ts, _, on_alarm = self.alarms.get()
            logger.warning(
                f"An alarm of {on_alarm} scheduled for {ts} is being discarded because alarm agent has finished.")
        # Wake it up so that it exits
        with self.sleep_cv:
            self.sleep_cv.notify()

    def is_empty(self):
        return self.alarms.empty()


class PeriodStatus(IntEnum):
    SUCCESS = 2
    OVERRIDEN = 1
    IGNORED = 0
    MISSED = -1
    FAILURE = -2


class PeriodicAgent(Agent):
    periodicity_agent: "PeriodicityAgent"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.period_status = PeriodStatus.MISSED
        # Unavoidable circular dependency
        self.periodicity_agent = None

    def attempt_cmd(self, timestamp: float, period_number: int, cmd: Command) -> PeriodStatus:
        """
        Attempts to solve cmd and returns one of the period status values.

        cmd: A command that is received in the current period.
        It is guaranteed that cmd timestamp is is within period offset.
        """
        logger.warning(f"Not implemented")
        return PeriodStatus.IGNORED

    def notify_overriden(self, period_no: int, timestamp: float, overrider_type_name: str):
        if self.period_status != PeriodStatus.MISSED:
            logger.critical(f"Unexpected state!")
        logger.debug(
            f"{type(self).__name__} is overriden by {overrider_type_name} at period {period_no} and time {timestamp}")
        self.period_status = PeriodStatus.OVERRIDEN

    def on_period_finished(self, timestamp: float, period_number: int):
        if self.period_status == PeriodStatus.SUCCESS:
            logger.info(
                f"{type(self).__name__} has succeeded the period number {period_number} at {timestamp}")
        elif self.period_status == PeriodStatus.OVERRIDEN:
            logger.debug(
                f"{type(self).__name__} is overridden at the period number {period_number} at {timestamp}")
        elif self.period_status == PeriodStatus.IGNORED:
            logger.info(
                f"{type(self).__name__} ignored the period number {period_number} at {timestamp}")
        elif self.period_status == PeriodStatus.MISSED:
            logger.error(
                f"{type(self).__name__} has missed the period number {period_number} at {timestamp}")
        elif self.period_status == PeriodStatus.FAILURE:
            logger.error(
                f"{type(self).__name__} has failed the period number {period_number} at {timestamp}")
        # Reset for the next period
        self.period_status = PeriodStatus.MISSED

    def _set_periodicity_agent(self, periodicity_agent):
        if self.periodicity_agent != None:
            self.cancel()
        self.periodicity_agent = periodicity_agent

    def cancel(self):
        """
        Removes itself from the attached periodicity agent.

        FIXME Periodicity agent is used by two threads, this function may break sth
        """
        if self.periodicity_agent == None:
            logger.critical(
                f"PeriodicAgent has called cancel but no periodicity agent is found!")
            return
        self.periodicity_agent.remove_periodic_agent(self)
        self.periodicity_agent = None

    def finish(self):
        self.cancel()
        return super().finish()


class PeriodicityAgent(Agent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stack = []
        self.period_offset = self.agents_config["period-offset"]
        self.period = self.agents_config["period"]
        self.alive = True

        self.next_period_end = self.agents_config["period-offset"]    # seconds
        self.curr_period_no = 0
        # Send the initial period number to the screen, the rest
        # will be sent after period ends
        self.update_screen({"curr-period-no": self.curr_period_no})
        # FIXME Check if period 0 is handled properly
        # Self register for the end of next period (period 0)
        AlarmAgent.instance().add_alarm(self.on_period,
                                        self.to_real_time(self.next_period_end))

    def add_periodic_agent(self, agent: PeriodicAgent):
        agent._set_periodicity_agent(self)
        self.stack.append(agent)

    def remove_periodic_agent(self, agent: PeriodicAgent):
        try:
            self.stack.remove(agent)
        except ValueError:
            logger.warning(
                f"Attempted to remove a periodic agent but it is not in the stack.")

    def process_cmd(self, timestamp: float, cmd: Command):
        # Check if a periodic command is expected around now
        period_time = self.curr_period_no * self.period
        if not period_time - self.period_offset < timestamp < period_time + self.period_offset:
            logger.debug(
                f"PeriodicityAgent ignores the command of type {type(cmd).__name__} as it is outside of the period")
            return
        # An agent consumes the command and the rest are notified of being overriden or they miss the period.
        handled = False
        overrider_type_name = None
        for agent in reversed(self.stack):
            agent: PeriodicAgent
            if not handled:
                # Attempt to handle
                result = agent.attempt_cmd(timestamp, self.curr_period_no, cmd)
                if result == PeriodStatus.SUCCESS:
                    logger.debug(f"PeriodicAgent accepted {cmd}")
                    handled = True
                    overrider_type_name = type(agent).__name__
                elif result == PeriodStatus.FAILURE:
                    logger.debug(f"PeriodicAgent rejected {cmd}")
                    overrider_type_name = type(agent).__name__
                    handled = True
                elif result == PeriodStatus.IGNORED:
                    # Need to try other items
                    pass
                else:
                    logger.critical(f"Unknown attempt result: {result}")
            else:
                # Handled, notify the rest of the stack this period is owned by others
                agent.notify_overriden(
                    timestamp, self.curr_period_no, overrider_type_name)

    def on_period(self):
        timestamp: float = self.relative_time()
        self.next_period_end = self.next_period_end + self.period
        self.curr_period_no += 1
        if self.alive:
            AlarmAgent.instance().add_alarm(self.on_period,
                                            self.to_real_time(self.next_period_end))
        # Last period has ended, notify the agents
        for a in self.stack:
            a: PeriodicAgent
            a.on_period_finished(timestamp, self.curr_period_no - 1)
        # Update the screen's current period number
        self.update_screen({"curr-period-no": self.curr_period_no})

    def finish(self):
        # Empty the stack just in case
        self.stack = []
        self.alive = False
        return super().finish()


class DistanceAgent(PeriodicAgent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.total_distance = self.agents_config["total-distance"]
        self.remaining_distance = self.total_distance
        # FIXME Why is the first period missed?
        self.period_status = PeriodStatus.IGNORED

    def send_speed_cmd(self):
        speed_cmd = SpeedCommand(10)    # TODO Add other distance calculation strategies
        self.remaining_distance -= speed_cmd.speed
        self.send_command(speed_cmd)

    def attempt_cmd(self, timestamp: float, period_number: int, cmd: Command) -> PeriodStatus:
        if type(cmd) == DistanceCommand:
            # Check if this period already had a successful periodic command
            if self.period_status != PeriodStatus.MISSED:
                logger.error(
                    f"Distance command for this period was already received.")
                self.period_status = PeriodStatus.FAILURE
                return PeriodStatus.FAILURE
            if self.remaining_distance != cmd.distance:
                logger.error(
                    f"DistanceAgent expected distance to be {self.remaining_distance} but found {cmd.distance}")
                self.period_status = PeriodStatus.FAILURE
            else:
                self.period_status = PeriodStatus.SUCCESS
            if self.remaining_distance == 0:
                # The last distance we sent is 0. We end it all here. Good bye.
                self.send_command(EndCommand())
                self.cancel()
                self.autopilot.finish()
                return self.period_status
            # Send speed command and update remaining distance
            self.send_speed_cmd()
            return self.period_status
        return PeriodStatus.IGNORED

    def notify_overriden(self, period_no: int, timestamp: float, overrider_type_name: str):
        super().notify_overriden(period_no, timestamp, overrider_type_name)
        self.send_speed_cmd()


class AltitudeAgent(PeriodicAgent):
    def __init__(self, turbulence_number: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.turbulence_number = turbulence_number
        self.details = self.agents_config["turbulence"][self.turbulence_number]
        self.turbulence_enter = self.details["turbulence-enter"]
        self.turbulence_exit = self.turbulence_enter + \
            (self.details["cmd-count"] + 1) * \
            self.details["altitude-period"] / \
            1000
        self.period2altitude = {}
        self.setup_period2altitude()
        # Schedule enter and exit to the turbulence
        AlarmAgent.instance().add_alarm(
            self.on_enter, self.to_real_time(self.turbulence_enter))
        AlarmAgent.instance().add_alarm(
            self.on_exit, self.to_real_time(self.turbulence_exit))

    def on_enter(self):
        self.send_command(AltitudeCommand(self.details["altitude-period"]))

    def on_exit(self):
        self.send_command(AltitudeCommand(AltitudePeriod.ALT_000))
        self.cancel()

    def setup_period2altitude(self):
        # turbulence_enter is in seconds, altitude-period is in milliseconds
        curr_period_time = self.turbulence_enter + \
            self.details["altitude-period"] / 1000
        # period is in seconds
        curr_period_number = round(
            curr_period_time / self.agents_config["period"])
        for i in range(self.details["cmd-count"]):
            self.period2altitude[curr_period_number] = self.details["altitude"]
            curr_period_number += round(self.details["altitude-period"] /
                                        1000 / self.agents_config["period"])

    def attempt_cmd(self, timestamp: float, period_number: int, cmd: Command) -> PeriodStatus:
        if not self.turbulence_enter < timestamp < self.turbulence_exit:
            # Ignore and do not print anything if not in this time region
            return PeriodStatus.IGNORED
        if type(cmd) == AltitudeCommand:
            # If no periodic message is expected now, it is a failure
            if period_number not in self.period2altitude:
                logger.error(
                    f"Altitude command received but it is not expected for this period: {period_number}.")
                self.period_status = PeriodStatus.FAILURE
                return self.period_status
            # Check if this period already had a successful periodic command
            if self.period_status != PeriodStatus.MISSED:
                logger.error(
                    f"Altitude command for this period was already received.")
                self.period_status = PeriodStatus.FAILURE
                return self.period_status
            # Check if the altitude is what we want
            if self.period2altitude[period_number] != cmd.altitude:
                logger.error(
                    f"Expected altitude for period {period_number} was {self.period2altitude[period_number]} but found {cmd.altitude}")
                self.period_status = PeriodStatus.FAILURE
            else:
                self.period_status = PeriodStatus.SUCCESS
            return self.period_status
        return PeriodStatus.IGNORED

    def on_period_finished(self, timestamp: float, period_number: int):
        if not self.turbulence_enter < timestamp < self.turbulence_exit:
            # Do not print anything if not in the time region
            return
        if period_number not in self.period2altitude and self.period_status == PeriodStatus.MISSED:
            # If no altitude message is received and we were not supposed to, we have ignored this period
            self.period_status = PeriodStatus.IGNORED
            # Perhaps this will cause unnecessary confusion, so disable it
            # # Update turbulence zone in the screen
            # self.update_screen({"turbulence-zone":
            #                     {"number": self.turbulence_number,
            #                      "state": AltitudeZoneState.NEUTRAL_STATE}})
        elif self.period_status == PeriodStatus.FAILURE or self.period_status == PeriodStatus.MISSED:
            self.update_screen({"turbulence-zone":
                                {"number": self.turbulence_number,
                                 "state": AltitudeZoneState.BAD_STATE}})
        elif self.period_status == PeriodStatus.SUCCESS:
            self.update_screen({"turbulence-zone":
                                {"number": self.turbulence_number,
                                 "state": AltitudeZoneState.GOOD_STATE}})
        return super().on_period_finished(timestamp, period_number)


class AltitudeControlEventType(str, Enum):
    FREQ = "freq"
    FREE = "free"
    ALTITUDE = "altitude"


class AltitudeControllerAgent(PeriodicAgent):
    # If some altitude is expected but its value does not matter use this
    ANY_ALTITUDE = -1

    def __init__(self, controller_idx: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller_idx = controller_idx
        config = self.agents_config["altitude-controls"][controller_idx]
        self.events = config["events"]
        self.enter = config["enter"]
        self.exit = config["exit"]
        self.events_finished = False
        self.period = self.agents_config["period"]
        AlarmAgent.instance().add_alarm(self.on_enter, self.to_real_time(self.enter))
        AlarmAgent.instance().add_alarm(self.on_exit, self.to_real_time(self.exit))
        # Keep track of which events occurred and their progress
        self.last_freq: AltitudePeriod = AltitudePeriod.ALT_000
        self.last_freq_period: int = -1
        self.curr_zone_no: int = -1
        # Events with count field take multiple periods. Where in that are we?
        self.curr_progress: int = 0
        self.curr_event_idx: int = 0
        # Set on on_period_finished
        self.next_expected_altitude: int = None

    def on_enter(self):
        logger.info(f"AltitudeControllerAgent no {self.controller_idx} enters, stopping incoming altitude commands")
        self.update_screen({"altitude-controls": True})

    def on_exit(self):
        logger.info(f"AltitudeControllerAgent no {self.controller_idx} exits, stopping incoming altitude commands")
        # Send this in case it is forgotten
        self.send_command(AltitudeCommand(AltitudePeriod.ALT_000))
        self.update_screen({"altitude-controls": False})

    def attempt_cmd(self, timestamp: float, period_number: int, cmd: Command) -> PeriodStatus:
        if not self.enter < timestamp < self.exit:
            # Ignore all the messages and do not log anything
            self.period_status = PeriodStatus.MISSED
            return PeriodStatus.IGNORED
        if type(cmd) == AltitudeCommand:
            cmd: AltitudeCommand
            # Update screen
            self.update_screen({"altitude": cmd.altitude})
            # Check whether we expect a command and the altitude value
            if self.next_expected_altitude == None:
                self.period_status = PeriodStatus.IGNORED
            elif self.next_expected_altitude == AltitudeControllerAgent.ANY_ALTITUDE:
                self.period_status = PeriodStatus.SUCCESS
            elif self.next_expected_altitude != cmd.altitude:
                logger.error(f"AltitudeControllerAgent no {self.controller_idx} has failed period {period_number} at {timestamp}")
                self.period_status = PeriodStatus.FAILURE
            else:
                logger.info(f"AltitudeControllerAgent no {self.controller_idx} has succeeded period {period_number} at {timestamp}")
                self.period_status = PeriodStatus.SUCCESS
            return self.period_status
        return PeriodStatus.IGNORED
    
    def on_period_finished(self, timestamp: float, period_number: int):
        if not self.enter < timestamp < self.exit:
            # Ignore and do not print
            return
        if self.next_expected_altitude == None and self.period_status == PeriodStatus.MISSED:
            self.period_status = PeriodStatus.IGNORED
        # Update screen
        if self.period_status == PeriodStatus.FAILURE or self.period_status == PeriodStatus.MISSED:
            self.update_screen({"altitude-zone": {"controller-no": self.controller_idx, "zone-no": self.curr_zone_no, "state": AltitudeZoneState.BAD_STATE}})
        elif self.period_status == PeriodStatus.SUCCESS and self.next_expected_altitude != AltitudeControllerAgent.ANY_ALTITUDE:
            # If expected altitude is any then this is not an altitude zone but a free zone
            self.update_screen({"altitude-zone": {"controller-no": self.controller_idx, "zone-no": self.curr_zone_no, "state": AltitudeZoneState.GOOD_STATE}})
        # Process current event and set next_expected_altitude if not finished the events
        if self.events_finished:
            self.calculate_next_expected_altitude(period_number + 1, AltitudeControllerAgent.ANY_ALTITUDE)
            # No event processing, just evaluate period status
            return super().on_period_finished(timestamp, period_number)
        try:
            curr_event = self.events[self.curr_event_idx]
        except IndexError as ex:
            logger.critical(f"AltitudeControllerAgent has an index error: {repr(ex)}")
            raise ex
        if curr_event["type"] == AltitudeControlEventType.FREQ:
            # Send frequency message
            value = curr_event["value"]
            logger.debug(f"AltitudeControllerAgent no {self.controller_idx} sends freq event for period {value} at {timestamp}. " + 
                         f"Event no {curr_event}")
            self.send_command(AltitudeCommand(curr_event["value"]))
            self.last_freq = curr_event["value"]
            self.last_freq_period = period_number
            self.period_status = PeriodStatus.IGNORED
            self.curr_event_idx += 1
            # Update the next expected altitude
            self.calculate_next_expected_altitude(period_number + 1, -2)    # -2 for debugging just in case
        elif curr_event["type"] == AltitudeControlEventType.FREE:
            # Progress it
            self.curr_progress += 1
            if self.curr_progress == curr_event["count"]:
                # Finished progressing this event
                self.curr_event_idx += 1
                self.curr_progress = 0
            # # Calculate for the next period
            self.calculate_next_expected_altitude(period_number + 1, AltitudeControllerAgent.ANY_ALTITUDE)
        elif curr_event["type"] == AltitudeControlEventType.ALTITUDE:
            if self.curr_progress == 0:
                # We entered a new altitude zone
                self.curr_zone_no += 1
            if self.last_freq_period == -1:
                logger.critical(f"Period number of last frequency is -1! Make sure your test case is valid.")
            if self.last_freq == AltitudePeriod.ALT_000:
                logger.critical(f"Last frequency is 0! Make sure your test case is valid.")
            # Calculate for the next period
            self.calculate_next_expected_altitude(period_number + 1, curr_event["value"])
            # Progress it
            self.curr_progress += 1
            if self.curr_progress == curr_event["count"]:
                self.curr_event_idx += 1
                self.curr_progress = 0
        if self.curr_event_idx == len(self.events):
            logger.debug(f"AltitudeControllerAgent no {self.controller_idx} finished progressing all of its events.")
            self.events_finished = True
        return super().on_period_finished(timestamp, period_number)

    def calculate_next_expected_altitude(self, next_period_number, expected_altitude: int):
        if self.last_freq == AltitudePeriod.ALT_000:
            # No reading is expected
            self.next_expected_altitude = None
        # Distance of two expected altitude commands in period counts 
        period_count = round(self.last_freq / 1000 / self.period * 10)/10
        if (next_period_number - self.last_freq_period) % period_count == 0:
            # We need to get an altitude command on the next period
            self.next_expected_altitude = expected_altitude
        else:
            # We are NOT expecting an altitude command on the next period
            self.next_expected_altitude = None

class LedAgent(PeriodicAgent):
    _DEFAULT_REMOVE_TIMEOUT = 3    # secs
    LED_2_BUTTON = {1: 4, 2: 5, 3: 6, 4: 7}

    def __init__(self, start_time: float, led: LedValue, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if led not in LedAgent.LED_2_BUTTON:
            logger.critical(f"Led number {led} is invalid!")
        self.led = led
        self.satisfied = False
        self.add_time = start_time
        AlarmAgent.instance().add_alarm(self.on_add_alarm, self.to_real_time(self.add_time))
        # Also set an alarm for removal
        try:
            self.remove_time = self.add_time + \
                self.agents_config["led-timeout"]
        except KeyError:
            logger.critical(f"Led command needs 'led-timeout' in the config!")
            self.remove_time = self.add_time + LedAgent._DEFAULT_REMOVE_TIMEOUT
        AlarmAgent.instance().add_alarm(self.on_remove_alarm,
                                        self.to_real_time(self.remove_time))

    def on_add_alarm(self):
        logger.info(
            f"Led task is added at {self.add_time} for led {self.led}")
        self.send_command(LedCommand(self.led))

    def on_remove_alarm(self):
        # Tell plane to turn off the leds just in case we did not already
        self.send_command(LedCommand(LedValue.LED_0))
        if self.satisfied:
            logger.info(
                f"Led task of led {self.led} at {self.add_time} was successful.")
        else:
            logger.error(
                f"Led task of led {self.led} at {self.add_time} has failed.")
        # Remove it from the periodicity agent
        self.cancel()

    def attempt_cmd(self, timestamp: float, period_number: int, cmd: Command) -> PeriodStatus:
        # NOTE I chose to allow multiple presses for one agent
        if not self.add_time < timestamp < self.remove_time:
            # We are not yet handling it
            return PeriodStatus.IGNORED
        if type(cmd) == PressCommand:
            cmd: PressCommand
            if cmd.button == LedAgent.LED_2_BUTTON[self.led]:
                self.period_status = PeriodStatus.SUCCESS
                if self.satisfied:
                    logger.info(
                        f"Led task of led {self.led} at {self.add_time} is satisfied at {timestamp}.")
                else:
                    logger.info(
                        f"Led task of led {self.led} at {self.add_time} was already satisfied.")
                self.satisfied = True
                # Turn the led off immediately
                self.send_command(LedCommand(LedValue.LED_0))
            else:
                # Pressed wrong button
                self.period_status = PeriodStatus.FAILURE
                logger.error(
                    f"Led task of led {self.led} at {self.add_time} has received incorrect button {cmd.button} at {timestamp}.")
            return self.period_status
        return PeriodStatus.IGNORED

    def on_period_finished(self, timestamp: float, period_number: int):
        if not self.add_time < timestamp < self.remove_time:
            # Ignore this period, do not print anything
            return
        if self.period_status == PeriodStatus.MISSED:
            # It is ok that some periods have no button action, as long as one is succeessful
            self.period_status = PeriodStatus.IGNORED
        return super().on_period_finished(timestamp, period_number)


class ManualAgent(Agent):
    def __init__(self, periodicity_agent: PeriodicityAgent, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Leds will be added to the periodicity_agent
        self.periodicity_agent = periodicity_agent
        self.leds = []
        # Alarm for switching to manual and back
        alarm_agent = AlarmAgent.instance()
        self.manual_enter = self.agents_config["manual"]["manual-enter"]
        alarm_agent.add_alarm(self.on_manual_enter,
                              self.to_real_time(self.manual_enter))
        self.manual_exit = self.agents_config["manual"]["manual-exit"]
        alarm_agent.add_alarm(self.on_manual_exit,
                              self.to_real_time(self.manual_exit))

    def on_manual_enter(self):
        logger.info("Entering manual mode")
        self.send_command(ManualCommand(1))
        self.update_screen({"manual": True})
        # Create and register LedAgents
        for led_info in self.agents_config["manual"]["leds"]:
            led = LedAgent(led_info["start-time"],
                           led_info["button"],
                           self.agents_config, self.autopilot)
            self.leds.append(led)
            self.periodicity_agent.add_periodic_agent(led)

    def on_manual_exit(self):
        logger.info("Exiting manual mode")
        self.send_command(ManualCommand(0))
        self.update_screen({"manual": False})

    def finish(self):
        # Empty the list just in case
        self.leds = []
        return super().finish()
