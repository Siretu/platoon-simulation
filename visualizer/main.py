from kivy.core.window import Window
from kivy.graphics.context_instructions import Color
from kivy.uix.widget import Widget

from route import Route
from buttons import PlayButton
ZOOM_FACTOR = 1.1


class VisualizerMain(Widget):
    PATH_WIDTH = 1
    zoom = 0.001
    speed = 5
    paused = False

    def __init__(self, data):
        super(VisualizerMain, self).__init__()
        self.lines = []
        self.scroll_offset_x = 800
        self.scroll_offset_y = 600
        if data:
            self.scroll_offset_x -= data[0]["node_coords_lat"][0]
            self.scroll_offset_y -= data[0]["node_coords_lon"][0]
        self.data = data

        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)
        self.pausbutton = self.children[0].initialize()

        self.initialize_routes(data)


    def initialize_routes(self, data):
        for route in data:
            line = Route(route)
            self.add_widget(line)
            self.lines.append(line)

    def _keyboard_closed(self):
        self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        self._keyboard = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        if keycode[1] == 'w' or keycode[1] == 'up':
            self.scroll_offset_y -= 10/self.zoom
        if keycode[1] == 's' or keycode[1] == 'down':
            self.scroll_offset_y += 10/self.zoom
        if keycode[1] == 'a' or keycode[1] == 'left':
            self.scroll_offset_x += 10/self.zoom
        if keycode[1] == 'd' or keycode[1] == 'right':
            self.scroll_offset_x -= 10/self.zoom
        if keycode[1] == 'z':
            f = ZOOM_FACTOR
            self.scroll_offset_x = (self.center_x / f + self.scroll_offset_x * self.zoom - self.center_x) / self.zoom
            self.scroll_offset_y = (self.center_y / f + self.scroll_offset_y * self.zoom - self.center_y) / self.zoom
            self.zoom *= ZOOM_FACTOR
        if keycode[1] == 'x':
            f = 1./ZOOM_FACTOR
            self.scroll_offset_x = (self.center_x/f + self.scroll_offset_x * self.zoom - self.center_x) / self.zoom
            self.scroll_offset_y = (self.center_y/f + self.scroll_offset_y * self.zoom - self.center_y) / self.zoom
            self.zoom /= ZOOM_FACTOR
        if keycode[1] == 'c':
            self.focus((self.data[0]["node_coords_lat"][0], self.data[0]["node_coords_lon"][0]))
        if keycode[1] == 'v':
            maxX = max([max(x["node_coords_lat"]) for x in self.data])
            minX = min([min(x["node_coords_lat"]) for x in self.data])
            a = minX + (maxX - minX) / 2.0
            maxY = max([max(x["node_coords_lon"]) for x in self.data])
            minY = min([min(x["node_coords_lon"]) for x in self.data])
            b = minY + (maxY - minY) / 2.0
            self.focus((a, b))
        if keycode[1] == 'spacebar':
            if self.paused:
                self.pausbutton.play()
            else:
                self.pausbutton.pause()
            self.paused = not self.paused
        if keycode[1] == '.':
            self.speed += 1
        if keycode[1] == ',':
            self.speed -= 1
        return True

    def focus(self, pos):
        self.scroll_offset_x = self.center_x / self.zoom - pos[0]
        self.scroll_offset_y = self.center_y / self.zoom - pos[1]

    def update(self, dt):
        with self.canvas:
            Color(0, 0, 1.)
            for line in self.lines:
                if self.paused:
                    line.update(0)
                else:
                    line.update(dt * self.speed)