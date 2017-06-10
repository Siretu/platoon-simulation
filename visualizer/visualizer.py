import numpy as np
from kivy.app import App
from kivy.clock import Clock
from kivy.core.window import Window

from main import VisualizerMain
from route import FRAMERATE

Window.clearcolor = (1, 1, 1, 1)


class VisualizerApp(App):
    def __init__(self, data):
        super(VisualizerApp, self).__init__()
        self.data = data

    def build(self):
        game = VisualizerMain(self.data)
        Clock.schedule_interval(game.update, 1.0 / FRAMERATE)
        return game


def start(routes):
    VisualizerApp(routes).run()


if __name__ == "__main__":
    route_data = {}
    route_data["node_coords_lat"] = np.array([100, 200, 300])
    route_data["node_coords_lon"] = np.array([100, 100, 200])
    route_data["link_lengths"] = np.array([100, 100, 150])
    route_data["v_default"] = 22.2
    route_data["t_s"] = 10
    start([route_data])
