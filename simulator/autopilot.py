#!/usr/bin/env python
import time
import threading
import serial
import json
import logging
from cmds import *
from commandqueue import CommandQueue
from screen import Screen
from enum import Enum
from pygame import locals as pygame_locals
from pygame.event import Event
import re
from agents import *


class PlaneState(Enum):
    IDLE = "IDLE"
    TAKE_OFF = "TAKE_OFF"
    CRUISE = "CRUISE"
    TURBULENCE = "TURBULENCE"
    LANDING = "LANDING"
    END = "END"


class SimulatorMode(Enum):
    """
    Simulator operation modes
    """
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    END = "END"


with open("autopilot-settings.json", "r") as f:
    SETTINGS = json.loads(f.read())

# Load test case, which is agents_config
with open("test-case-0.json", 'r') as f:
    lines = f.readlines()
    lines = [line for line in lines if not re.search(r'//', line)]
    TESTCASE = json.loads("\n".join(lines))

print(TESTCASE)

FPS = SETTINGS["FPS"]
PORT = SETTINGS["PORT"]
BAUDRATE = SETTINGS["BAUDRATE"]
LOG_LEVEL = SETTINGS["LOG_LEVEL"]
WAITING = 0
GETTING = 1
timeout = 100
CMD_PERIOD = 1  # seconds, not used with agents!
CMD_GET_TIMEOUT = CMD_PERIOD * 1.1  # seconds
logging.basicConfig(level=getattr(logging, LOG_LEVEL))


class AutoPilot:
    def __init__(self, port, baudrate, parity, rtscts, xonxoff):
        logging.info("AutoPilot initialization")
        self.serial = serial.Serial(port, baudrate, parity=parity,
                                    rtscts=rtscts, xonxoff=xonxoff,
                                    timeout=10  # 10 secs timeout
                                    )
        self.mode = SimulatorMode.IDLE
        self.mode_lock = threading.Lock()
        self.mode_cv = threading.Condition(self.mode_lock)
        self.start_time = 0

        # Reader
        self.alive = True
        self.reader_thread = threading.Thread(target=self.reader_worker)
        self.reader_thread.daemon = True
        self.reader_thread.start()
        self.cmd_buffer = CMDBuffer()
        self.cmd_queue = CommandQueue(CMD_GET_TIMEOUT)

        # Writer
        self.writer_lock = threading.Lock()

        # UI
        self.screen = Screen()
        self.screen.start()
        self.screen.add_keyboard_handler(self.screen_keyboard_handler)

        # Important agents
        self.cmd_dispatcher = None
        self.periodicity_agent = None

    def reader_worker(self):
        logging.info("AutoPilot reader thread has begun")
        while self.alive:
            byte = self.serial.read()
            if len(byte) == 0:
                print(
                    f"Reader has timed out. Timeout was {self.serial.timeout}")
                continue
            self.cmd_buffer.append(byte)
            cmd = self.cmd_buffer.parse_command()
            if cmd:
                # TODO Handle all cmds
                # TODO Update cmd freqs and keep cmd receive times
                # TODO Update screen
                logging.debug(
                    f"Queueing command {cmd} received at {time.time() - self.start_time}")
                cmd_type = type(cmd)
                if cmd_type == DistanceCommand:
                    logging.info(f"Distance report: {cmd.distance}")
                    self.screen.set_distance(cmd.distance)
                elif cmd_type == AltitudeCommand:
                    logging.info(f"Altitude report: {cmd.altitude}")
                    self.screen.set_altitude(cmd.altitude)
                else:
                    # TODO
                    # logging.warning(
                    #     f"Command handler is not implemented: {cmd}")
                    pass
                self.cmd_queue.put(cmd)

    def stop_reader(self):
        """
        Stops reading thread at most after self.serial.timeout seconds
        """
        logging.debug(
            f"AutoPilot reader thread will stop in {self.serial.timeout} seconds")
        self.alive = False

    def join(self):
        """
        Joins reader thread
        """
        logging.debug(f"AutoPilot reader thread will be joined")
        self.reader_thread.join(0.1)

    def write(self, message: bytes | Command):
        logging.debug(f"Writing '{str(message)}'")
        with self.writer_lock:
            if issubclass(type(message), Command):
                self.serial.write(message.make_bytes())
            elif type(message) == bytes:
                self.serial.write(message)
            else:
                logging.error(
                    f"Write has received message of unknown type {type(message)}")

    def update_screen(self, update: object):
        self.screen.update(update)

    def screen_keyboard_handler(self, event: Event):
        """
        Called from UI Thread, do not block!
        """
        if event.type == pygame_locals.KEYDOWN and event.key == pygame_locals.K_s:
            with self.mode_cv:
                if self.mode == SimulatorMode.IDLE:
                    # Start the simulator
                    self.mode = SimulatorMode.ACTIVE
                    self.mode_cv.notify()

    def wait_until_start(self):
        # Wait until user presses "s" in the screen
        with self.mode_cv:
            self.mode_cv.wait_for(lambda: self.mode == SimulatorMode.ACTIVE)
        logging.info(f"Simulator is now in ACTIVE mode.")

    def demo(self):
        self.wait_until_start()
        # Run a demo flight
        total_distance = 10000
        period = CMD_PERIOD    # secs
        logging.info(f"Demo sends GoCommand")
        self.start_time = time.time()
        self.cmd_queue.set_start_time(self.start_time)
        self.write(GoCommand(total_distance))
        self.write(AltitudeCommand(AltitudePeriod.ALT_400))
        # time.sleep(period)
        self.cmd_queue.get()
        for i in range(int(90/period - 1)):
            # FIXME Send this only after receiving a distance command
            logging.info(f"Demo sends SpeedCommand")
            self.write(SpeedCommand(10))
            # time.sleep(period)
            self.cmd_queue.get()
        with self.mode_cv:
            self.mode = SimulatorMode.END
            logging.info(f"Simulator is now in END mode.")
            logging.info(f"Demo sends end command")
            self.write(EndCommand())
        logging.info(f"Demo has ended.")

    def setup_periodicity_agent(self, testcase):
        periodicity_agent = PeriodicityAgent(testcase, self)
        # Add distance agent to the bottom of the stack
        distance_agent = DistanceAgent(testcase, self)
        periodicity_agent.add_periodic_agent(distance_agent)
        
        # Deprecated agent
        # # Add altitude agents
        # altitude_agents = [AltitudeAgent(
        #     idx, testcase, self) for idx, tur in enumerate(testcase["turbulence"])]
        # for aa in altitude_agents:
        #     periodicity_agent.add_periodic_agent(aa)
        
        for controller_idx, controls in enumerate(testcase["altitude-controls"]):
            periodicity_agent.add_periodic_agent(AltitudeControllerAgent(controller_idx, testcase, self))
        return periodicity_agent

    def agents_demo(self):
        self.wait_until_start()
        logging.info(f"Agents Demo sends GoCommand")
        # FIXME Too many time vars, reduce them
        self.start_time = time.time()
        self.cmd_queue.set_start_time(self.start_time)
        TESTCASE["go-time"] = self.start_time
        self.update_screen({"TESTCASE": TESTCASE})
        self.cmd_queue.set_start_time(TESTCASE["go-time"])
        self.write(GoCommand(TESTCASE["total-distance"]))
        # Create and setup agents
        cmd_dispatcher = CommandDispatcherAgent(self.cmd_queue, TESTCASE, self)
        periodicity_agent: PeriodicityAgent = self.setup_periodicity_agent(
            TESTCASE)
        manual_agent = ManualAgent(periodicity_agent, TESTCASE, self)
        # Register command consuming agents to the command dispatcher
        cmd_dispatcher.add_agent(periodicity_agent)
        cmd_dispatcher.start()
        # Assign to self
        self.cmd_dispatcher = cmd_dispatcher
        self.periodicity_agent = periodicity_agent
        time.sleep(100)

    def finish(self):
        if self.periodicity_agent:
            self.periodicity_agent.finish()
            self.periodicity_agent = None
        if self.cmd_dispatcher:
            self.cmd_dispatcher.finish()
            self.periodicity_agent = None
        self.stop_reader()
        # Finishing a singleton does not make sense unless the program is exiting
        # AlarmAgent.instance().finish()
        if not AlarmAgent.instance().is_empty():
            logger.warning(
                f"AutoPilot has finished but AlarmAgent is not empty. Some alarms may be delivered later.")


def main():
    ap = AutoPilot(PORT, BAUDRATE, 'N',
                   rtscts=False, xonxoff=False)
    # dw = DistanceWriter(ap, 1)
    ap.agents_demo()
    while True:
        pass


if __name__ == "__main__":
    # PORT = input("Dev name: ")
    main()
