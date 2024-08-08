from enum import Enum
import sys
import threading
import pygame
from pygame.locals import *
from ui.autopilotvisualizer import *
from ui.drawable import *

DISPLAY_WIDTH = 640*2
DISPLAY_HEIGHT = 480*2
FPS = 24

class StatusValue(str, Enum):
    NORMAL = "NORMAL"
    ALTITUDE = "ALTITUDE"
    MANUAL = "MANUAL"


class Screen:
    # Altitudes from the top in order
    ALTITUDES = [12000, 11000, 10000, 9000]

    def __init__(self):
        self.ui_thread = threading.Thread(
            target=self.update_loop)
        # UI values
        self._speed = -1
        self._altitude = -1
        self._distance = -1
        # Handlers
        self._keyboard_handlers = []
        # UI
        self.visualizer: AutopilotVisualizer = None

    def add_keyboard_handler(self, handler):
        self._keyboard_handlers.append(handler)

    def initialize_ui(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(
            (DISPLAY_WIDTH, DISPLAY_HEIGHT), pygame.SRCALPHA)
        pygame.display.set_caption("CENG 336 THE3 2024")

        self.status_text_font = pygame.font.SysFont("Arial", 32, True)
        self.status_text_label = Text((0, 0), "Status: ", (0, 0, 0), self.status_text_font)
        self.status_text_label.set_anchor(0, 0)

        self.set_status_text(StatusValue.NORMAL)
        # talkbubble_sprite = pygame.image.load(f"the4_material/talkbubble.png")
        # talkbubble_sprite = pygame.transform.scale(
        #     talkbubble_sprite, (120, 80))

    def start(self):
        self.ui_thread.start()

    def _set_status_text(self, text: str, color: tuple[int, int, int]):
        self.status_text = Text((0, 0), text, color, self.status_text_font)

    def set_status_text(self, status: StatusValue):
        if status == StatusValue.NORMAL:
            self._set_status_text(status, (0, 0, 0))
        elif status == StatusValue.ALTITUDE:
            self._set_status_text(status, (255, 0, 0))
        elif status == StatusValue.MANUAL:
            self._set_status_text(status, (0, 255, 0))

    def update_loop(self):
        # Need to call all PyGame API from the same thread or we may get errors
        self.initialize_ui()

        self.visualizer = AutopilotVisualizer(
            (0, 70), Screen.ALTITUDES, DISPLAY_WIDTH)

        while True:
            self.screen.fill((0x88, 0xc2, 0xf6),
                             pygame.Rect(0, 0, DISPLAY_WIDTH, 300))
            self.screen.fill((0xd4, 0xef, 0xff), pygame.Rect(
                0, 300, DISPLAY_WIDTH, DISPLAY_HEIGHT-300))
            self.visualizer.draw(self.screen, Transform(0, 0))

            # distance_text = font_footer.render(
            #     f"Last Reported Distance: {self._distance}", True, (0, 0, 0))
            # self.screen.blit(distance_text, (320, 240))
            # altitude_text = font_footer.render(
            #     f"Last Reported Altitude: {self._altitude}", True, (0, 0, 0))
            # self.screen.blit(altitude_text, (320, 252))

            # Draw status texts
            total_status_text_width = self.status_text_label.get_width() + self.status_text.get_width()
            text_position_x = DISPLAY_WIDTH / 2 - total_status_text_width / 2
            self.status_text_label.draw(self.screen, Transform(text_position_x, AutopilotVisualizer.DEFAULT_HEIGHT + 100))
            self.status_text.draw(self.screen, Transform(text_position_x + self.status_text_label.get_width(), AutopilotVisualizer.DEFAULT_HEIGHT + 100))

            pygame.display.update()

            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        pygame.quit()
                        sys.exit()
                for h in self._keyboard_handlers:
                    h(event)
            self.clock.tick(FPS)

    def set_speed(self, speed: int):
        self._speed = speed

    def set_altitude(self, altitude: int):
        self._altitude = altitude

    def set_distance(self, distance: int):
        self._distance = distance

    def update(self, update: object):
        # TODO Handle other updates
        if update.get("curr-period-no"):
            self.visualizer.update(update["curr-period-no"])
        elif update.get("TESTCASE"):
            # Deprecated
            # # Let visualizer create turbulence zones
            # self.visualizer.configure_turbulences(
            #     update["TESTCASE"]["period"],
            #     update["TESTCASE"]["turbulence"])
            self.visualizer.configure_altitude_zones(   
                    update["TESTCASE"]["period"],
                    update["TESTCASE"]["altitude-controls"])
        elif update.get("altitude-zone"):
            self.visualizer.update_altitude_zone(
                update["altitude-zone"]["controller-no"], update["altitude-zone"]["zone-no"], update["altitude-zone"]["state"])
        elif update.get("altitude"):
            self.visualizer.set_plane_altitude(update["altitude"])
        elif update.get("manual") != None:
            if update["manual"]:
                self.set_status_text(StatusValue.MANUAL)
            else:
                self.set_status_text(StatusValue.NORMAL)
        elif update.get("altitude-controls") != None:
            if update["altitude-controls"]:
                self.set_status_text(StatusValue.ALTITUDE)
            else:
                self.set_status_text(StatusValue.NORMAL)


if __name__ == "__main__":
    Screen().update_loop()
