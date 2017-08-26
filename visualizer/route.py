from kivy.graphics.context_instructions import Color
from kivy.graphics.vertex_instructions import Line, Rectangle, Ellipse
from kivy.uix.widget import Widget
import numpy as np

from profiler import profile

FRAMERATE = 60.0


class Route(Widget):
    PATH_WIDTH = 1
    MARKER_SIZE = (5, 5)
    time = 0

    def __init__(self, assignment, route_data):
        super(Route, self).__init__()
        self.line = None
        self.lat = np.array([route_data[x]['lat'] for x in assignment.path])
        self.lon = np.array([route_data[x]['lon'] for x in assignment.path])
        self.links = assignment.path_weights
        self.speed = assignment.speed_history[0].speed
        self.speed_history = assignment.speed_history
        self.start_pos = (self.lat[0], self.lon[0])
        self.end_pos = (self.lat[-1], self.lon[-1])
        self.truck = None
        self.start_time = assignment.start_time
        self.following = False
        self.identifier = assignment.id
        self.assignment = assignment

    def draw(self):

        with self.canvas:
            Color(0, 0, 1.)
            self.line = Line(points=self.transform_data(self.lat, self.lon), width=self.PATH_WIDTH)
            Color(0, 1., 0)
            self.start = Rectangle(pos=self.transform_points(self.start_pos), size=self.MARKER_SIZE)
            Color(1., 0, 0)
            self.goal = Rectangle(pos=self.transform_points(self.end_pos), size=self.MARKER_SIZE)

    @profile
    def update(self, dt):
        self.time += dt
        if not self.line:
            self.draw()
        if self.time >= self.start_time and not self.truck:
            with self.canvas:
                Color(1., 0, 1.)
                self.truck = Ellipse(pos=[x - 5 for x in self.start.pos], size=(10, 10))
        self.line.points = self.transform_data(self.lat, self.lon)
        self.start.pos = self.transform_points(self.start_pos)
        self.goal.pos = self.transform_points(self.end_pos)
        self.calculate_speed()
        if self.truck:
            self.truck.pos = self.transform_points(self.calculate_truck_pos())
        pass

    def calculate_speed(self):
        for change in self.speed_history:
            if self.time < change.end_time:
                self.speed = change.speed
                if change.platooning != self.following and self.truck in self.canvas.children:
                    self.following = change.platooning != -1
                    self.canvas.children.remove(self.truck)
                    with self.canvas:
                        if self.following:
                            Color(0, 0.6, 1.0)
                        else:
                            Color(1., 0, 1.)
                        self.truck = Ellipse(pos=self.truck.pos, size=(10, 10))
                # self.following = change.platooning
                # if self.truck:
                #     self.canvas.clear()

                break

    def calculate_truck_pos(self):
        current = self.assignment.current_distance(self.time)
        # current = (self.time - self.start_time) * self.speed
        i = 0
        while i < len(self.links) - 1 and current > self.links[i+1]:
            current -= self.links[i+1]
            i += 1

        x = self.lat[i]
        y = self.lon[i]

        if i < len(self.links) - 1:
            x += (self.lat[i + 1] - self.lat[i]) * (current / self.links[i+1])
            y += (self.lon[i + 1] - self.lon[i]) * (current / self.links[i+1])

        return x, y

    def transform_data(self, lat, lon):
        a = (lat + self.parent.scroll_offset_x) * self.parent.zoom
        b = (lon + self.parent.scroll_offset_y) * self.parent.zoom
        return [j for i in zip(a, b) for j in i]

    def transform_points(self, pos):
        a = (pos[0] + self.parent.scroll_offset_x) * self.parent.zoom
        b = (pos[1] + self.parent.scroll_offset_y) * self.parent.zoom
        return a, b
