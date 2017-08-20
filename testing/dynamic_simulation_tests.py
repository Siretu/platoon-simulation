from __future__ import absolute_import

import unittest
import time
import cProfile

from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from route_calculation import generate_routes, get_path_data_sets, get_routes
from run_simulation import simulation, dynamic_simulation


class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.start = time.time()
        self.pr = cProfile.Profile()
        self.pr.disable()
        self.pr.enable()

    def tearDown(self):
        self.pr.disable()
        self.pr.dump_stats("profile.pstat")
        t = time.time() - self.start
        print "%s: %.3f" % (self.id(), t)

    def test_100_1_greedy(self):
        result = dynamic_simulation(GreedyPlatooning(), folder='./testroutes/test100-1/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.930883023723, fuel_saving, delta=10 ** -10)

    def test_100_1_random(self):
        result = dynamic_simulation(RandomPlatooning(0), folder='./testroutes/test100-1/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.945426474166, fuel_saving, delta=10 ** -10)

    def test_100_1_sub_deterministic(self):
        result = dynamic_simulation(SubModularityPlatooning(), folder='./testroutes/test100-1/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.95890707523, fuel_saving, delta=10 ** -10)

    def test_100_1_sub_stochastic(self):
        result = dynamic_simulation(SubModularityPlatooning(False), folder='./testroutes/test100-1/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.968866387747, fuel_saving, delta=10 ** -10)

    def test_100_2_greedy(self):
        result = dynamic_simulation(GreedyPlatooning(), folder='./testroutes/test100-2/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.949286537386, fuel_saving, delta=10 ** -10)

    def test_100_2_random(self):
        result = dynamic_simulation(RandomPlatooning(0), folder='./testroutes/test100-2/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.949329167891, fuel_saving, delta=10 ** -10)

    def test_100_2_sub_deterministic(self):
        result = dynamic_simulation(SubModularityPlatooning(), folder='./testroutes/test100-2/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.968308771047, fuel_saving, delta=10 ** -10)

    def test_100_2_sub_stochastic(self):
        result = dynamic_simulation(SubModularityPlatooning(False), folder='./testroutes/test100-2/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.971008792877, fuel_saving, delta=10 ** -10)

    def test_100_3_greedy(self):
        result = dynamic_simulation(GreedyPlatooning(), folder='./testroutes/test100-3/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.944508378226, fuel_saving, delta=10 ** -10)

    def test_100_3_random(self):
        result = dynamic_simulation(RandomPlatooning(0), folder='./testroutes/test100-3/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.948770385621, fuel_saving, delta=10 ** -10)

    def test_100_3_sub_deterministic(self):
        result = dynamic_simulation(SubModularityPlatooning(), folder='./testroutes/test100-3/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.978799737386, fuel_saving, delta=10 ** -10)

    def test_100_3_sub_stochastic(self):
        result = dynamic_simulation(SubModularityPlatooning(False), folder='./testroutes/test100-3/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.970021555973, fuel_saving, delta=10 ** -10)

    def test_100_4_greedy(self):
        result = dynamic_simulation(GreedyPlatooning(), folder='./testroutes/test100-4/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.94887673735, fuel_saving, delta=10 ** -10)

    def test_100_4_random(self):
        result = dynamic_simulation(RandomPlatooning(0), folder='./testroutes/test100-4/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.962134051418, fuel_saving, delta=10 ** -10)

    def test_100_4_sub_deterministic(self):
        result = dynamic_simulation(SubModularityPlatooning(), folder='./testroutes/test100-4/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.982172558959, fuel_saving, delta=10 ** -10)

    def test_100_4_sub_stochastic(self):
        result = dynamic_simulation(SubModularityPlatooning(False), folder='./testroutes/test100-4/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.981547641282, fuel_saving, delta=10 ** -10)

    def test_100_5_greedy(self):
        result = dynamic_simulation(GreedyPlatooning(), folder='./testroutes/test100-5/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.939128567203, fuel_saving, delta=10 ** -10)

    def test_100_5_random(self):
        result = dynamic_simulation(RandomPlatooning(0), folder='./testroutes/test100-5/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.947652410128, fuel_saving, delta=10 ** -10)

    def test_100_5_sub_deterministic(self):
        result = dynamic_simulation(SubModularityPlatooning(), folder='./testroutes/test100-5/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.955611271364, fuel_saving, delta=10 ** -10)

    def test_100_5_sub_stochastic(self):
        result = dynamic_simulation(SubModularityPlatooning(False), folder='./testroutes/test100-5/')
        fuel_saving = sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
        print fuel_saving
        self.assertAlmostEqual(0.953373151581, fuel_saving, delta=10 ** -10)
