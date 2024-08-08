import pygame
import logging

logger = logging.getLogger("drawable")

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)


class Transform:
    x: float
    y: float

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def move(self, dx, dy):
        self.x += dx
        self.y += dy

    def combine(self, transform: "Transform"):
        return Transform(self.x + transform.x, self.y + transform.y)

    def transform_point(self, point: tuple[float, float]):
        return point[0] + self.x, point[1] + self.y

    def transform_rect(self, rect: pygame.Rect):
        return rect.move(self.x, self.y)


class Drawable:
    def draw(self, surface: pygame.Surface, transform: Transform):
        logger.warning(f"Not implemented")


class Image(Drawable):
    def __init__(self, position: tuple[float, float], path: str):
        super().__init__()
        self.path = path
        self.image = pygame.image.load(path)
        self.position = position

    def scale(self, width, height):
        image = pygame.image.load(self.path)
        self.image = pygame.transform.scale(
            image, (width, height))

    def draw(self, surface: pygame.Surface, transform: Transform):
        surface.blit(self.image, transform.transform_point(self.position))


class Text(Drawable):
    def __init__(self, position: tuple[float, float], text: str, color, font: pygame.font.Font):
        super().__init__()
        self.position = position
        self.text = text
        self.font = font
        self.color = color
        self.anchor = (0, 0)
        self.rendered_text = self.font.render(
            text, True, self.color)

    def set_anchor(self, x, y):
        self.anchor = (x, y)

    def draw(self, surface: pygame.Surface, transform: Transform):
        # By default pygame draws as if anchor is at the top left
        width, height = self.rendered_text.get_width(), self.rendered_text.get_height()
        position = transform.transform_point(self.position)
        position = (position[0] - width * self.anchor[0],
                    position[1] - height * self.anchor[1])
        surface.blit(self.rendered_text, position)
    
    def get_width(self):
        return self.rendered_text.get_width()
    
    def get_height(self):
        return self.rendered_text.get_height()


class Container(Drawable):
    def __init__(self, position: tuple[float, float]):
        super().__init__()
        self.contents = []
        self.transform = Transform(*position)

    def add_content(self, content):
        self.contents.append(content)

    def move(self, dx, dy):
        self.position[0] += dx
        self.position[1] += dy

    def draw(self, surface: pygame.Surface, transform: Transform):
        for content in self.contents:
            content.draw(surface, transform.combine(self.transform))


class Line(Drawable):
    def __init__(self, start_pos: tuple[float, float], end_pos: tuple[float, float], color, width=1):
        super().__init__()
        self.color = color
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.width = width

    def move(self, dx, dy):
        self.start_pos = (self.start_pos[0] + dx, self.start_pos[1] + dy)
        self.end_pos = (self.end_pos[0] + dx, self.end_pos[1] + dy)

    def draw(self, surface: pygame.Surface, transform: Transform):
        pygame.draw.line(surface, BLACK,
                         transform.transform_point(
                             self.start_pos), transform.transform_point(self.end_pos),
                         self.width)


class DashedLine(Line):
    def __init__(self, start_pos: tuple[float, float], end_pos: tuple[float, float], color, width=1, dash_length=4):
        super().__init__(start_pos, end_pos, color, width)
        self.dash_length = dash_length

    def draw(self, surface: pygame.Surface, transform: Transform):
        # Mostly ChatGPT
        x1, y1 = transform.transform_point(self.start_pos)
        x2, y2 = transform.transform_point(self.end_pos)
        dx = x2 - x1
        dy = y2 - y1
        distance = max(abs(dx), abs(dy))
        dx = dx / distance
        dy = dy / distance
        x, y = x1, y1
        for i in range(int(distance / self.dash_length)):
            if i % 2 == 0:
                pygame.draw.line(surface, self.color, (round(x), round(y)), (round(
                    x + dx * self.dash_length), round(y + dy * self.dash_length)), self.width)
            x += dx * self.dash_length
            y += dy * self.dash_length


class Rectangle(Drawable):
    def __init__(self, left: float, top: float, width: float, height: float, color: tuple[int, int, int]):
        self.rect = pygame.Rect(left, top, width, height)
        self.color = color

    def draw(self, surface: pygame.Surface, transform: Transform):
        pygame.draw.rect(surface, self.color,
                         transform.transform_rect(self.rect))
