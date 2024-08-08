import logging
from queue import Empty, Queue
from cmds import Command
import time


class CommandQueue(Queue):
    get_timeout: int    # timeout in seconds
    start_time: float   # relative start offset in seconds
    queue: Queue
    
    def __init__(self, get_timeout: int):
        self.get_timeout = get_timeout
        self.queue = Queue()
        self.start_time = 0
        
    def set_start_time(self, start_time: float):
        self.start_time = start_time
    
    def get_current_relative_timestamp(self):
        return time.time() - self.start_time
    
    def get(self) -> tuple[float, Command]:
        """
        Returns a timestamp and command immediately or blocks until the queue is not empty.

        Returns:
            tuple[float, Command]: First item is the timestamp of the command relative to the GO command sent by the server, and the second is the command itself.
        """
        try:
            retval = self.queue.get(True, self.get_timeout)
            return retval
        except Empty as ex:
            logging.error(f"Command queue could not get an item in given timeout of {self.get_timeout} seconds.")
            return None

    def put(self, cmd: Command, timestamp: float = None):
        """
        Puts a command in the queue with a timestamp relative to the GO command sent by the server.
        If timestamp is None, milliseconds since self.start_time is used.

        Args:
            cmd (Command): _description_
            timestamp (float): _description_
        """
        if timestamp == None:
            timestamp = self.get_current_relative_timestamp()
        self.queue.put((timestamp, cmd))
