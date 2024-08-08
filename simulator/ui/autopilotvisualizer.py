from typing import List
from pygame import Surface
from agents import AltitudeControlEventType
from ui.drawable import Transform
from .drawable import *
from .enums import *

class AltitudeZone(Drawable):
    ALPHA = 140
    GOOD_COLOR = (0, 255, 0, ALPHA)
    NEUTRAL_COLOR = (255, 255, 0, ALPHA)
    BAD_COLOR = (255, 0, 0, ALPHA)
    BORDER_RADIUS = 25

    def __init__(self, turbulence_no: int, zone_start: int, zone_end: int, screen_rect: pygame.Rect):
        """
        zone_start: Index of the period that the zone starts at
        zone_end: Index of the period that the zone ends at
        """
        super().__init__()
        # deprecated field
        self.turbulence_no = turbulence_no
        self.zone_start = zone_start
        self.zone_end = zone_end
        self.screen_positions = (-1, -1)
        # Rectangle of the zone
        self.rect: pygame.Rect = None
        # The parts of the area the zone will fill in
        self.screen_rect = screen_rect
        self.color = AltitudeZone.NEUTRAL_COLOR

    def calculate_screen_position_in_period(self, curr_period_no: int, screen_length: int):
        """
        curr_period_no: The period that corresponds to where the plane currently is
        screen_length: Number of periods that fit in the screen
        """
        # Find the start position relative to curr period no
        if curr_period_no <= self.zone_start < curr_period_no + screen_length:
            screen_zone_start = self.zone_start - curr_period_no
        else:
            screen_zone_start = -1
        # Find the end position relative to curr period no
        if curr_period_no < self.zone_end < curr_period_no + screen_length:
            screen_zone_end = self.zone_end - curr_period_no
        else:
            screen_zone_end = -1
        if self.zone_start <= curr_period_no and curr_period_no + screen_length <= self.zone_end:
            # Screen is full with the altitude zone
            screen_zone_start, screen_zone_end = 0, screen_length - 1
        return (screen_zone_start, screen_zone_end)

    def draw(self, surface: Surface, transform: Transform):
        if self.rect:
            canvas = pygame.Surface(self.rect.size, pygame.SRCALPHA)
            pygame.draw.rect(canvas, self.color,
                             canvas.get_rect(), border_radius=AltitudeZone.BORDER_RADIUS)
            surface.blit(canvas, transform.transform_rect(self.rect))

    def set_state(self, state: AltitudeZoneState):
        if state == AltitudeZoneState.GOOD_STATE:
            self.color = AltitudeZone.GOOD_COLOR
        elif state == AltitudeZoneState.NEUTRAL_STATE:
            self.color = AltitudeZone.NEUTRAL_COLOR
        elif state == AltitudeZoneState.BAD_STATE:
            self.color = AltitudeZone.BAD_COLOR
        else:
            logger.critical(f"Unexpected altitude zone state: {state}")

    def update(self, curr_period_no: int, screen_length: int) -> bool:
        self.screen_positions = self.calculate_screen_position_in_period(
            curr_period_no, screen_length)
        if self.screen_positions[0] != -1 and self.screen_positions[1] != -1:
            # Both ends are visible
            left = self.screen_positions[0] * \
                self.screen_rect.width/screen_length
            right = self.screen_positions[1] * \
                self.screen_rect.width/screen_length
        elif self.screen_positions[0] != -1:
            # Only left end is visible
            left = self.screen_positions[0] * \
                self.screen_rect.width/screen_length
            right = screen_length * self.screen_rect.width/screen_length
        elif self.screen_positions[1] != -1:
            # Only right end is visible
            left = 0
            right = self.screen_positions[1] * \
                self.screen_rect.width/screen_length
        else:
            # Neither end is visible, do not draw anything
            self.rect = None
            return False
        self.rect = pygame.Rect(
            self.screen_rect.left + left, self.screen_rect.top, right-left, self.screen_rect.height)
        return True


class SlidingBackground(Container):
    DEFAULT_HEIGHT = 400
    DEFAULT_WIDTH = 500
    DEFAULT_BGR_COLOR = (255, 0, 0)
    DEFAULT_IMG_PATH = "./assets/tilesetOpenGameBackground.png"

    def __init__(self, position: tuple[float, float], image_path=DEFAULT_IMG_PATH, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, color=DEFAULT_BGR_COLOR):
        super().__init__(position)
        self.width = width
        self.height = height
        self.color = color
        self.offset = 0
        self.speed = 3
        # Add rectangle
        self.add_content(
            Rectangle(0, 0, self.width, self.height, self.color))
        # Add image
        self.image = pygame.image.load(image_path)
        self.canvas = None
        # Scale the image to with height
        scale_factor = self.height / self.image.get_height()
        self.image = pygame.transform.scale(
            self.image, (self.image.get_width() * scale_factor, self.height))

    def draw(self, surface: pygame.Surface, transform: Transform):
        super().draw(surface, transform)
        # Draw on canvas, crop it, put it on the surface
        image_width = self.image.get_width()
        image_height = self.image.get_height()
        repeat_count = ((self.width - 1)//image_width+2)
        if self.canvas == None:
            self.canvas = pygame.Surface(
                (repeat_count * image_width, image_height))
        # Initially the start point of the left most image instance
        x_offset = -self.offset
        for i in range(repeat_count):
            self.canvas.blit(self.image, (x_offset, 0))
            x_offset += image_width
        # Put it on the surface now
        transform = transform.combine(self.transform)
        surface.blit(self.canvas, (transform.x, transform.y),
                     pygame.Rect(0, 0, self.width, self.height))

    def update_offset(self):
        self.offset = (self.offset + self.speed) % self.image.get_width()


class AutopilotVisualizer(Container):
    DEFAULT_HEIGHT = 600
    DEFAULT_WIDTH = 1200
    DEFAULT_LINE_COLOR = (0, 0, 0)
    PLANE_PATH = "./assets/thy_small.png"
    PADDING = 30
    # Number of periods visible on the screen
    SCREEN_LENGTH = 100

    def __init__(self, position: tuple[float, float], altitudes: List[int], width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
        """
        Puts SlidingBackground, dashed altitude lines and texts, calculates 
        altitude constraint zone screen regions (where they are drawn).

        It is a subclass of Container and the order of add_content() calls
        determines z-order of the content.

        Its contents are in the following order:
        - self.container contains the SlidingBackground and altitude lines / texts.
        - self.altitude_zone_container contains altitude zones
        - self.plane, which is always the top most drawable.
        """
        super().__init__(position)
        self.width = width
        self.height = height
        self.altitudes = altitudes
        self.altitude_texts: List[Text] = []
        self.altitude_regions: List[pygame.Rect] = []
        self.altitude_zone_container = Container((0, 0))
        line_count = len(altitudes)
        self.line_offset_y = self.height/(line_count+1)
        self.font = pygame.font.SysFont("Arial", 12)
        # Provides fast access to zones: zones[controller_no][zone_no]
        self.zones: List[List[AltitudeZone]] = []
        # Put the sliding background and stuff above it in a container
        self.container = Container((0, 0))
        self.add_content(self.container)
        # Add background
        self.sliding_bgr = SlidingBackground(
            (0, 0), SlidingBackground.DEFAULT_IMG_PATH, self.width, self.height)
        self.container.add_content(self.sliding_bgr)
        self._add_altitude_lines_and_text()
        self._make_altitude_zone_regions()
        # Add altitude zones container
        self.add_content(self.altitude_zone_container)
        # Add plane
        self.plane = Image((0, 0), AutopilotVisualizer.PLANE_PATH)
        self.plane.scale(100, 50)
        self.container.add_content(self.plane)
        self.set_plane_altitude(self.altitudes[-1], False)

    def _add_altitude_lines_and_text(self):
        line_count = len(self.altitudes)
        for i in range(1, line_count+1):
            self.altitude_texts.append(
                Text((self.width - AutopilotVisualizer.PADDING, i*self.line_offset_y), f"{str(self.altitudes[i-1])}", (0, 0, 0), self.font))
            self.altitude_texts[-1].set_anchor(1, 0.5)
            self.add_content(self.altitude_texts[-1])
            dashed_line = DashedLine((AutopilotVisualizer.PADDING, i*self.line_offset_y),
                                     (self.sliding_bgr.width - 70, i*self.line_offset_y), AutopilotVisualizer.DEFAULT_LINE_COLOR)
            self.container.add_content(dashed_line)

    def _make_altitude_zone_regions(self):
        # Calculate altitude zone maximal screen regions
        for i in range(1, len(self.altitudes)+1):
            region = pygame.Rect(AutopilotVisualizer.PADDING,
                                 i*self.line_offset_y - self.line_offset_y/3/2,
                                 self.sliding_bgr.width - AutopilotVisualizer.PADDING,
                                 self.line_offset_y/3)
            self.altitude_regions.append(region)

    def draw(self, surface: Surface, transform: Transform):
        self.sliding_bgr.update_offset()
        return super().draw(surface, transform)

    def update(self, curr_period_no: int):
        for z in self.altitude_zone_container.contents:
            z: AltitudeZone
            z.update(curr_period_no, AutopilotVisualizer.SCREEN_LENGTH)

    # def configure_turbulences(self, period: float, turbulences):
    #     # Deprecated
    #     for tur_no, tur in enumerate(turbulences):
    #         idx = self.altitudes.index(tur["altitude"])
    #         start_period = tur["turbulence-enter"]/period
    #         # altitude-period is in milliseconds, all else in seconds
    #         end_period = start_period + \
    #             tur["cmd-count"]*tur["altitude-period"]/1000/period
    #         zone = AltitudeZone(tur_no, start_period, end_period,
    #                             self.altitude_regions[idx])
    #         self.altitude_zone_container.add_content(zone)
    
    def configure_altitude_zones(self, period: float, altitude_controls):
        for controller_no, control in enumerate(altitude_controls):
            curr_period_no = control["enter"]/period
            self.zones.append([])
            zone_no = 0
            for event in control["events"]:
                if AltitudeControlEventType.FREQ == event["type"]:
                    curr_period_no += 1
                    continue
                elif event["type"] == AltitudeControlEventType.FREE:
                    curr_period_no += event["count"]
                    continue
                elif event["type"] == AltitudeControlEventType.ALTITUDE:
                    idx = self.altitudes.index(event["value"])
                    start_period = curr_period_no
                    # end period is inclusive
                    end_period = start_period + event["count"] - 1
                    zone = AltitudeZone(zone_no, start_period, end_period,
                                        self.altitude_regions[idx])
                    zone_no += 1
                    self.altitude_zone_container.add_content(zone)
                    self.zones[controller_no].append(zone)
                    curr_period_no += event["count"]
                else:
                    logger.critical("Unknown event type")

    def update_altitude_zone(self, controller_no: int, zone_no: int, state: AltitudeZoneState):
        zone = self.zones[controller_no][zone_no].set_state(state)

    def set_plane_altitude(self, altitude: int, lerp: bool = True):
        # TODO Implement lerp
        idx = self.altitudes.index(altitude)
        self.plane.position = (AutopilotVisualizer.PADDING, (1 + idx) * self.line_offset_y - 35)
        