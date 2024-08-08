import logging
from enum import Enum, IntEnum
from utils import int2hexstring, hexstring2int


CMD_START_BYTE = b'$'
CMD_END_BYTE = b'#'
CMD_START_INT = int.from_bytes(CMD_START_BYTE, byteorder="little")
CMD_END_INT = int.from_bytes(CMD_END_BYTE, byteorder="little")


class AltitudePeriod(IntEnum):
    """
    Possible values for altitude reading frequencies
    """
    ALT_000 = 0
    """
    Do not read altitude anymore
    """
    ALT_200 = 200
    """
    Read altitude every 200 ms
    """
    ALT_400 = 400
    """
    Read altitude every 400 ms
    """
    ALT_600 = 600
    """
    Read altitude every 600 ms
    """


class LedValue(IntEnum):
    """
    Possible led values
    """
    LED_0 = 0
    LED_MIN = LED_0
    LED_1 = 1
    LED_2 = 2
    LED_3 = 3
    LED_4 = 4
    LED_MAX = LED_4

class CommandID:
    # Plane CMD IDs
    SPEED_MSG_ID = b"SPD"
    DISTANCE_MSG_ID = b"DST"
    ALTITUDE_MSG_ID = b"ALT"
    PRESS_MSG_ID = b"PRS"
    # AutoPilot CMD IDs
    LED_MSG_ID = b"LED"
    FUEL_MSG_ID = b"FUE"
    TURBULENCE_MSG_ID = b"TUR"  # MAX ALTITUDE and MAX SPEED
    ALTITUDE_FREQ_MSG_ID = b"FRE"
    GO_MSG_ID = b"GOO"  # total distance
    END_MSG_ID = b"END"
    MANUAL_MSG_ID = b"MAN"


class Command:
    MSG_ID = None

    def make_bytes(self):
        raise Exception("Not implemented")

    @classmethod
    def parse_bytes(cls, buffer: bytes):
        if buffer[0] != CMD_START_INT or buffer[-1] != CMD_END_INT:
            logging.error("Unable to parse buffer!")
            return None
        if cls.MSG_ID != None and buffer[1:4] != cls.MSG_ID:
            logging.error("Wrong message id!")
            return None
        return cls._parse_bytes(buffer)

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        """
        To be overriden by subclass.
        This default one figures out the subclass by looking at msg id bytes.
        """
        cmd_id = buffer[1:4]
        if cmd_id == CommandID.SPEED_MSG_ID:
            return SpeedCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.DISTANCE_MSG_ID:
            return DistanceCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.ALTITUDE_MSG_ID:
            return AltitudeCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.GO_MSG_ID:
            return GoCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.LED_MSG_ID:
            return LedCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.MANUAL_MSG_ID:
            return ManualCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.PRESS_MSG_ID:
            return PressCommand._parse_bytes(buffer)
        elif cmd_id == CommandID.END_MSG_ID:
            return EndCommand._parse_bytes(buffer)
        else:
            # TODO Implement the rest of them
            logging.error(
                f"Command type for MSG ID {cmd_id} is not found! (Not implemented yet?)")
            return None

    def __repr__(self):
        cls_name = type(self).__name__
        repr = f"<{cls_name}"
        for k in self.__dict__:
            repr += f" {k}:\"{str(self.__dict__[k])}\""
        repr += ">"
        return repr


# ---------------- Plane CMDs
class SpeedCommand(Command):
    MSG_ID = CommandID.SPEED_MSG_ID

    speed: int

    def __init__(self, speed: int):
        self.speed = speed

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        speed = hexstring2int(buffer[4:8])
        return SpeedCommand(speed)

    def make_bytes(self):
        return CMD_START_BYTE + SpeedCommand.MSG_ID + int2hexstring(self.speed) + CMD_END_BYTE


class PressCommand(Command):
    MSG_ID = CommandID.PRESS_MSG_ID

    button: int

    def __init__(self, button: int):
        self.button = button
        if type(self.button) != int or self.button > 7 or self.button < 4:
            logging.error(f"Invalid PressCommand button value: {self.button}")

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        # FIXME Ask Merve
        button = hexstring2int(buffer[4:6])
        return PressCommand(button)

    def make_bytes(self):
        return CMD_START_BYTE + PressCommand.MSG_ID + int2hexstring(self.button, 2) + CMD_END_BYTE


class DistanceCommand(Command):
    MSG_ID = CommandID.DISTANCE_MSG_ID

    distance: int

    def __init__(self, distance: int):
        self.distance = distance

    def make_bytes(self):
        return CMD_START_BYTE + CommandID.DISTANCE_MSG_ID + int2hexstring(self.distance) + CMD_END_BYTE

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        distance = hexstring2int(buffer[4:8])
        return DistanceCommand(distance)


# ---------------- Simulator CMDs
class LedCommand(Command):
    MSG_ID = CommandID.LED_MSG_ID

    led: int

    def __init__(self, led: int | LedValue):
        self.led = int(led)

    def make_bytes(self):
        return CMD_START_BYTE + CommandID.LED_MSG_ID + int2hexstring(self.led, 2) + CMD_END_BYTE

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        button = hexstring2int(buffer[4:6])
        return LedCommand(button)


class ManualCommand(Command):
    MSG_ID = CommandID.MANUAL_MSG_ID

    value: int

    def __init__(self, value: int):
        self.value = value

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        # FIXME Ask Merve
        value = hexstring2int(buffer[4:6])
        return ManualCommand(value)

    def make_bytes(self):
        return CMD_START_BYTE + ManualCommand.MSG_ID + int2hexstring(self.value, 2) + CMD_END_BYTE


class GoCommand(Command):
    MSG_ID = CommandID.GO_MSG_ID

    total_distance: int

    def __init__(self, total_distance: int):
        self.total_distance = total_distance

    def make_bytes(self):
        return CMD_START_BYTE + GoCommand.MSG_ID + int2hexstring(self.total_distance) + CMD_END_BYTE

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        total_distance = hexstring2int(buffer[4:8])
        return GoCommand(total_distance)


class EndCommand(Command):
    MSG_ID = CommandID.END_MSG_ID

    def make_bytes(self):
        return CMD_START_BYTE + EndCommand.MSG_ID + CMD_END_BYTE
    
    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        return EndCommand()


# ---------------- Both simulator and plane CMDS


class AltitudeCommand(Command):
    MSG_ID = CommandID.ALTITUDE_MSG_ID

    altitude: int

    def __init__(self, altitude: int | AltitudePeriod):
        self.altitude = int(altitude)

    @classmethod
    def _parse_bytes(cls, buffer: bytes):
        altitude = hexstring2int(buffer[4:8])
        return AltitudeCommand(altitude)

    def make_bytes(self):
        return CMD_START_BYTE + AltitudeCommand.MSG_ID + int2hexstring(self.altitude) + CMD_END_BYTE


# ---------------- Buffering CMD bytes


class CMDBuffer:
    """
    Buffer for building a command. It can only store one command at a time.
    The command string must be parsed directly after it is completed.
    """

    def __init__(self):
        self.reset()

    def append(self, byte):
        if len(byte) != 1:
            logging.error(
                f"CMDBuffer append got byte array of size {len(byte)}")
            return False
        # Check if buffer is in valid state
        if self._is_command_string_built:
            logging.error(
                f"Byte received but the previously built command is not used! Undefined behaviour may occur")
        # Check if command string has started
        if len(self._buffer) == 0:
            if byte == CMD_START_BYTE:
                # Command string building is started now
                self._buffer += byte
                return True
            else:
                # Ignore, we expected a CMD_START_BYTE here
                return False
        if byte == CMD_START_BYTE:
            logging.warning(
                f"CMD_START_BYTE received before receiving CMD_END_BYTE")
            # Dispose old bytes
            self._buffer = CMD_START_BYTE
            return True
        elif byte == CMD_END_BYTE:
            # Command string building has finished
            self._buffer += byte
            self._is_command_string_built = True
            return True
        else:
            # Just a normal byte, extend
            self._buffer += byte
            return True

    def is_command_string_built(self):
        return self._is_command_string_built

    def parse_command(self):
        if not self._is_command_string_built:
            return None
        # Parse and return a command
        cmd = Command.parse_bytes(self._buffer)
        logging.debug(f"CMDBuffer parsed {cmd}")
        self.reset()
        return cmd

    def reset(self):
        self._is_command_string_built = False
        self._buffer = b""
