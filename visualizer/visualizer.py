import cProfile

import numpy as np
from kivy.app import App
from kivy.clock import Clock
from kivy.core.window import Window

from main import VisualizerMain
from platooning.platooning_methods import GreedyPlatooning
from route import FRAMERATE
from route_calculation import generate_routes, get_routes, get_path_data_sets
from run_simulation import dynamic_simulation

TEST_FOLDER = '../testing/testroutes/test100-5/'

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
    # generate_routes(100, "../testing/testroutes/test100-4/")
    path_data = {
        0: {
            "arrival_dline":150000,
            "path": np.array([2, 3, 4, 6]),
            "path_set": {2, 3, 4, 6},
            "path_weights": np.array([80000, 1000000, 157100]),
            "start_pos": {"i":0, "x":0},
            "t_s": 1000
        },
        1: {
            "arrival_dline": 150000,
            "path": np.array([1, 3, 4, 5]),
            "path_set": {1, 3, 4, 5},
            "path_weights": np.array([80000, 1000000, 157100]),
            "start_pos": {"i": 0, "x": 0},
            "t_s": 1500
        }
    }
    path_data = get_path_data_sets(TEST_FOLDER)
    routes = get_routes(TEST_FOLDER)
    # route_info = get_route_info(TEST_FOLDER)

    result = dynamic_simulation(GreedyPlatooning(), path_data_sets=path_data, folder=TEST_FOLDER)
    # routes = {
    #     1: {"lat": 0, "lon": 500},
    #     2: {"lat": 0, "lon": 0},
    #     3: {"lat": 500, "lon": 0},
    #     4: {"lat": 5000, "lon": 0},
    #     5: {"lat": 5500, "lon": 500},
    #     6: {"lat": 5500, "lon": -500},
    # }

    start([result, routes])

    # route_data = {}
    # route_data["node_coords_lat"] = np.array([100, 200, 300])
    # route_data["node_coords_lon"] = np.array([100, 100, 200])
    # route_data["link_lengths"] = np.array([100, 100, 150])
    # route_data["v_default"] = 22.2
    # route_data["t_s"] = 10
