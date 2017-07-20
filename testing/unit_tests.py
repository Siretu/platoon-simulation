from __future__ import absolute_import

import unittest
import time

from pairwise_planning import DefaultPlan, AdaptedPlan
from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from run_simulation import simulation, dynamic_simulation


class UnitTests(unittest.TestCase):
    def setUp(self):
        self.start = time.time()

    def tearDown(self):
        t = time.time() - self.start
        print "%s: %.3f" % (self.id(), t)

    def test_default_calculate_history(self):
        plan = DefaultPlan(100, 0, 22)
        new_history = plan.calculate_history(0, 10)
        self.assertEquals(22, new_history[0].speed)
        self.assertEquals(0, new_history[0].start_time)
        self.assertFalse(new_history[0].platooning)

    def test_adapted_calculate_history_1(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(0, 8)
        self.assertEquals(30, new_history[0].speed)
        self.assertEquals(0, new_history[0].start_time)
        self.assertFalse(new_history[0].platooning)

    def test_adapted_calculate_history_2(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(0, 15)
        self.assertEquals(30, new_history[0].speed)
        self.assertEquals(0, new_history[0].start_time)
        self.assertFalse(new_history[0].platooning)
        self.assertEquals(22, new_history[1].speed)
        self.assertEquals(10, new_history[1].start_time)
        self.assertTrue(new_history[1].platooning)

    def test_adapted_calculate_history_3(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(0, 30)
        self.assertEquals(30, new_history[0].speed)
        self.assertEquals(0, new_history[0].start_time)
        self.assertFalse(new_history[0].platooning)
        self.assertEquals(22, new_history[1].speed)
        self.assertEquals(10, new_history[1].start_time)
        self.assertTrue(new_history[1].platooning)
        self.assertEquals(35, new_history[2].speed)
        self.assertEquals(20, new_history[2].start_time)
        self.assertFalse(new_history[2].platooning)

    def test_adapted_calculate_history_4(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(11, 15)
        self.assertEquals(22, new_history[0].speed)
        self.assertEquals(11, new_history[0].start_time)
        self.assertTrue(new_history[0].platooning)

    def test_adapted_calculate_history_5(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(12, 22)
        self.assertEquals(22, new_history[0].speed)
        self.assertEquals(12, new_history[0].start_time)
        self.assertTrue(new_history[0].platooning)
        self.assertEquals(35, new_history[1].speed)
        self.assertEquals(20, new_history[1].start_time)
        self.assertFalse(new_history[1].platooning)

    def test_adapted_calculate_history_6(self):
        # Fuel consumption, fuel difference, distance to merge, distance to split, merge time, split time, arrival time
        plan = AdaptedPlan(0, 0, 100, 100, 10, 20, 40, 30, 22, 35)
        new_history = plan.calculate_history(25, 38)
        self.assertEquals(35, new_history[0].speed)
        self.assertEquals(25, new_history[0].start_time)
        self.assertFalse(new_history[0].platooning)