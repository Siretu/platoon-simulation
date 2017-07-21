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
        result = simulation("./testroutes/test100-1/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0311919806273, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0387018044227, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11537.9635517, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11448.5257358, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: 55, 4: -2, 5: -1, 6: -2, 7: 55, 8: -2, 9: -2, 10: -2, 11: 97, 12: -2, 13: 40, 14: 72, 15: -1, 16: 15, 17: -1, 18: 92, 19: 65, 20: -2, 21: -1, 22: 65, 23: 17, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: -1, 32: 97, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: -1, 39: -2, 40: -1, 41: 17, 42: 95, 43: 79, 44: 5, 45: 38, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: 21, 53: 38, 54: -2, 55: -1, 56: -2, 57: 97, 58: -2, 59: 62, 60: -2, 61: -2, 62: -1, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 82, 76: -1, 77: -2, 78: -2, 79: -1, 80: 31, 81: -2, 82: -1, 83: -2, 84: 21, 85: -2, 86: 55, 87: 79, 88: -2, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: 82, 95: -1, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 32604779.089786265, 2: 8496948.2763619889, 3: 7821469.8874862678, 4: 1746276.9944754937}")
        pass


    def test_100_1_random(self):
        result = simulation("./testroutes/test100-1/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0299692776603, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0368439597038, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11552.5252625, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11470.6516309, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: -2, 4: -2, 5: -1, 6: 84, 7: 88, 8: -2, 9: -2, 10: -2, 11: 40, 12: -2, 13: 40, 14: 72, 15: 16, 16: -1, 17: 41, 18: 92, 19: 65, 20: -2, 21: 84, 22: 65, 23: 41, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: 80, 32: 57, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: 53, 39: -2, 40: -1, 41: -1, 42: -1, 43: 79, 44: 5, 45: 53, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: -2, 53: -1, 54: -2, 55: 86, 56: -2, 57: -1, 58: -2, 59: 62, 60: -2, 61: -2, 62: -1, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 82, 76: -1, 77: -2, 78: -2, 79: -1, 80: -1, 81: -2, 82: -1, 83: -2, 84: -1, 85: -2, 86: -1, 87: 79, 88: -1, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: 82, 95: 42, 96: -2, 97: 57, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 33555715.737587988, 2: 7292454.2430634033, 3: 7901970.8447231231, 4: 1919333.4227354936}")
        pass

    def test_100_1_sub_deterministic(self):
        result = simulation("./testroutes/test100-1/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0273863272861, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0338613632423, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11583.2867619, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11506.1726925, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -1, 2: -1, 3: -1, 4: -1, 5: 50, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: -1, 12: -1, 13: -1, 14: -1, 15: 16, 16: -1, 17: -1, 18: -1, 19: 2, 20: -1, 21: 17, 22: 2, 23: 17, 24: -1, 25: -1, 26: -1, 27: 36, 28: -1, 29: -1, 30: -1, 31: 80, 32: 97, 33: -1, 34: -1, 35: -1, 36: -1, 37: -1, 38: -1, 39: -1, 40: 13, 41: 17, 42: -1, 43: -1, 44: -1, 45: 38, 46: -1, 47: -1, 48: 76, 49: 91, 50: -1, 51: -1, 52: -2, 53: 38, 54: -1, 55: 86, 56: -1, 57: 97, 58: -2, 59: -1, 60: -1, 61: -2, 62: -2, 63: 13, 64: 76, 65: 2, 66: -1, 67: 76, 68: -1, 69: 13, 70: -1, 71: 25, 72: 14, 73: -1, 74: -1, 75: -1, 76: -1, 77: -1, 78: -2, 79: 87, 80: -1, 81: -1, 82: 75, 83: -1, 84: 17, 85: -1, 86: -1, 87: -1, 88: 7, 89: -1, 90: -1, 91: -1, 92: 18, 93: -1, 94: 75, 95: 42, 96: -1, 97: -1, 98: -1, 99: 34}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 20600342.78173627, 2: 8440275.6222278755, 3: 5991308.5154216019, 4: 721856.07384094852, 5: 536825.43752831628}")
        pass

    def test_100_1_sub_stochastic(self):
        result = simulation("./testroutes/test100-1/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0291954758373, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0346069214157, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11561.740811, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11497.2935102, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: -1, 4: -2, 5: -2, 6: -1, 7: 88, 8: -2, 9: -2, 10: -2, 11: 97, 12: -2, 13: 40, 14: 72, 15: 16, 16: -1, 17: 23, 18: 92, 19: 65, 20: -2, 21: 23, 22: 65, 23: -1, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: -2, 32: 97, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: 45, 39: -2, 40: -1, 41: 23, 42: 95, 43: -2, 44: -1, 45: -1, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: -2, 53: 45, 54: -2, 55: 86, 56: -2, 57: 97, 58: -2, 59: -1, 60: -2, 61: -2, 62: -2, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 94, 76: -1, 77: -2, 78: -2, 79: 87, 80: -2, 81: -2, 82: 94, 83: -2, 84: -2, 85: -2, 86: -1, 87: -1, 88: -1, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: -1, 95: -1, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 31634554.829269029, 2: 6913786.7017660597, 3: 7032369.4490456125, 4: 2238474.3314653146}")
        pass

    def test_100_2_greedy(self):
        result = simulation("./testroutes/test100-2/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0328894925827, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0413100727447, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12892.5716355, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12780.3166945, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 21, 7: -2, 8: -2, 9: 23, 10: -2, 11: -2, 12: -2, 13: 44, 14: -2, 15: -1, 16: 44, 17: 15, 18: -2, 19: -2, 20: 55, 21: -1, 22: 95, 23: -1, 24: -2, 25: -2, 26: -1, 27: 52, 28: 26, 29: 32, 30: 99, 31: -1, 32: -1, 33: -2, 34: -2, 35: -2, 36: -1, 37: 15, 38: -2, 39: -2, 40: 36, 41: 21, 42: -1, 43: -2, 44: -1, 45: -2, 46: -1, 47: -2, 48: -2, 49: 52, 50: -2, 51: 26, 52: -1, 53: -2, 54: 42, 55: -1, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: -1, 62: 81, 63: -2, 64: -2, 65: -2, 66: 31, 67: 68, 68: -1, 69: -2, 70: 55, 71: -2, 72: 46, 73: -2, 74: -1, 75: -2, 76: -2, 77: -2, 78: 74, 79: 86, 80: 55, 81: -1, 82: 55, 83: -2, 84: 44, 85: -1, 86: -1, 87: -2, 88: 74, 89: -2, 90: 92, 91: 61, 92: -1, 93: 74, 94: -2, 95: -1, 96: -2, 97: 15, 98: 52, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 35559741.018573508, 2: 10601985.881495101, 3: 7742976.8253671452, 4: 2640255.3675283473, 5: 172714.86536590659}")
        pass

    def test_100_2_random(self):
        result = simulation("./testroutes/test100-2/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0315289483684, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0379268897688, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12910.7090806, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12825.4179818, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 41, 7: -2, 8: -2, 9: 23, 10: -2, 11: -2, 12: -2, 13: 84, 14: -2, 15: 37, 16: 84, 17: 37, 18: -2, 19: -2, 20: 80, 21: 41, 22: 95, 23: -1, 24: -2, 25: -2, 26: 28, 27: 52, 28: -1, 29: 32, 30: 99, 31: -1, 32: -1, 33: -2, 34: -2, 35: -2, 36: 90, 37: -1, 38: -2, 39: -2, 40: 90, 41: -1, 42: 54, 43: -2, 44: 84, 45: -2, 46: -1, 47: -2, 48: -2, 49: 52, 50: -2, 51: -2, 52: -1, 53: -2, 54: -1, 55: 80, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: 91, 62: 81, 63: -2, 64: -2, 65: -2, 66: 31, 67: 68, 68: -1, 69: -2, 70: 80, 71: -2, 72: 46, 73: -2, 74: 78, 75: -2, 76: -2, 77: -2, 78: -1, 79: 86, 80: -1, 81: -1, 82: 80, 83: -2, 84: -1, 85: -1, 86: -1, 87: -2, 88: 78, 89: -2, 90: -1, 91: -1, 92: -2, 93: -2, 94: -2, 95: -1, 96: -2, 97: 37, 98: 52, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 37066651.285864204, 2: 8765272.6077286936, 3: 7695888.7309663566, 4: 3074762.7880789074, 5: 115098.54569184608}")
        pass

    def test_100_2_sub_deterministic(self):
        result = simulation("./testroutes/test100-2/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0273089531856, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0354236491422, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12966.96593, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12858.7887381, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -1, 2: -1, 3: -1, 4: -1, 5: -1, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: -1, 12: -1, 13: -1, 14: -2, 15: -1, 16: 13, 17: 15, 18: -1, 19: 6, 20: -1, 21: 6, 22: 4, 23: -2, 24: -1, 25: -1, 26: 28, 27: -1, 28: -1, 29: -1, 30: -1, 31: -2, 32: -2, 33: -1, 34: -1, 35: -1, 36: -1, 37: 15, 38: -2, 39: -1, 40: 36, 41: 6, 42: 54, 43: -1, 44: 13, 45: -1, 46: 72, 47: -1, 48: -1, 49: 27, 50: -1, 51: -1, 52: 27, 53: -1, 54: -1, 55: -1, 56: -1, 57: -1, 58: -1, 59: -1, 60: -1, 61: 91, 62: 81, 63: -1, 64: -1, 65: -1, 66: -1, 67: 68, 68: -1, 69: -1, 70: 55, 71: -1, 72: -1, 73: -1, 74: -1, 75: -1, 76: -1, 77: -1, 78: 74, 79: -1, 80: 55, 81: -1, 82: 55, 83: -1, 84: 13, 85: 3, 86: 79, 87: -1, 88: 74, 89: -2, 90: -1, 91: -1, 92: -2, 93: 74, 94: -1, 95: 4, 96: -2, 97: 15, 98: 27, 99: 30}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 23713844.123447031, 2: 7420319.3830553191, 3: 6587465.0402527712, 4: 3150574.1376828845}")
        pass

    def test_100_2_sub_stochastic(self):
        result = simulation("./testroutes/test100-2/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.030729417679, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0366896249595, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12921.3676419, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12841.9119864, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 41, 7: -2, 8: -2, 9: -1, 10: -2, 11: -2, 12: -2, 13: 44, 14: -2, 15: 37, 16: 44, 17: 37, 18: -2, 19: -1, 20: 80, 21: 41, 22: 95, 23: -2, 24: -2, 25: -2, 26: 28, 27: 98, 28: -1, 29: -1, 30: 99, 31: -2, 32: -2, 33: -2, 34: -2, 35: -2, 36: 90, 37: -1, 38: -2, 39: -2, 40: 90, 41: -1, 42: 54, 43: -2, 44: -1, 45: -2, 46: 72, 47: -2, 48: -2, 49: 98, 50: -2, 51: -2, 52: 98, 53: -2, 54: -1, 55: 80, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: 91, 62: 81, 63: -2, 64: -2, 65: -2, 66: -1, 67: 68, 68: -1, 69: -2, 70: 80, 71: -2, 72: -1, 73: -2, 74: 88, 75: -2, 76: -2, 77: -2, 78: 88, 79: 86, 80: -1, 81: -1, 82: 80, 83: -2, 84: 44, 85: -1, 86: -1, 87: -2, 88: -1, 89: -2, 90: -1, 91: -1, 92: -2, 93: 88, 94: -2, 95: -1, 96: -2, 97: 37, 98: -1, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 35758096.643980756, 2: 7755905.8848997885, 3: 8152817.3966050958, 4: 2897775.1861005165, 5: 115098.54569184608}")
        pass

    def test_100_3_greedy(self):
        result = simulation("./testroutes/test100-3/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.029016718588, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0342242315794, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00154438410258, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11996.2976474, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11931.9599013, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 12354.7932051, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 41, 3: -2, 4: -2, 5: 17, 6: -2, 7: 22, 8: 33, 9: -1, 10: 28, 11: -2, 12: 53, 13: -2, 14: 27, 15: 64, 16: 73, 17: -1, 18: -1, 19: 88, 20: -2, 21: -2, 22: -1, 23: -2, 24: -2, 25: -2, 26: -2, 27: -1, 28: -1, 29: -2, 30: -1, 31: -2, 32: 28, 33: -1, 34: -2, 35: -2, 36: -1, 37: -2, 38: -2, 39: 81, 40: -2, 41: -1, 42: -1, 43: -1, 44: -1, 45: 43, 46: 41, 47: 56, 48: -2, 49: -2, 50: -2, 51: -2, 52: -2, 53: -1, 54: -1, 55: 22, 56: -1, 57: 22, 58: 33, 59: 43, 60: -2, 61: 80, 62: -2, 63: -1, 64: -1, 65: 44, 66: 42, 67: -2, 68: -2, 69: 81, 70: 64, 71: -1, 72: 54, 73: -1, 74: 9, 75: -2, 76: 63, 77: 71, 78: 99, 79: 81, 80: -1, 81: -1, 82: -2, 83: 18, 84: 93, 85: -2, 86: -2, 87: 53, 88: -1, 89: 18, 90: -2, 91: 36, 92: -2, 93: -1, 94: 54, 95: 30, 96: -2, 97: -2, 98: 9, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 33817541.353592508, 2: 13300820.579831602, 3: 5195430.8873951668, 4: 250453.40064170401}")
        pass

    def test_100_3_random(self):
        result = simulation("./testroutes/test100-3/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0288203663405, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0359200287834, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00154438410258, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11998.7235388, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11911.0086775, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 12354.7932051, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 41, 3: -2, 4: -2, 5: 17, 6: -2, 7: 22, 8: 58, 9: 98, 10: 28, 11: -2, 12: -2, 13: -2, 14: 83, 15: 70, 16: 73, 17: -1, 18: 89, 19: 88, 20: -2, 21: -2, 22: -1, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: -1, 29: -2, 30: 95, 31: -2, 32: 28, 33: 58, 34: -2, 35: -2, 36: 91, 37: -2, 38: -2, 39: 81, 40: -2, 41: -1, 42: 66, 43: 59, 44: 65, 45: 59, 46: 41, 47: 56, 48: -2, 49: -2, 50: 59, 51: -2, 52: -2, 53: 87, 54: 94, 55: 22, 56: -1, 57: 22, 58: -1, 59: -1, 60: -2, 61: 70, 62: -2, 63: -1, 64: 70, 65: -1, 66: -1, 67: -2, 68: -2, 69: 81, 70: -1, 71: 77, 72: 94, 73: -1, 74: 75, 75: -1, 76: 63, 77: -1, 78: -1, 79: 81, 80: 70, 81: -1, 82: -2, 83: -1, 84: 93, 85: -2, 86: -2, 87: -1, 88: -1, 89: -1, 90: -2, 91: -1, 92: -2, 93: -1, 94: -1, 95: -1, 96: -2, 97: -2, 98: -1, 99: 78}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 34706545.020762295, 2: 11661019.352851547, 3: 3334840.4599001831, 4: 2232216.198861172, 5: 629625.18908578646}")
        pass

    def test_100_3_sub_deterministic(self):
        result = simulation("./testroutes/test100-3/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0258677692977, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0318540683147, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00154438410258, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12035.2022647, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11961.2427783, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 12354.7932051, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -2, 2: -1, 3: -1, 4: -1, 5: 17, 6: -2, 7: -1, 8: -1, 9: -1, 10: -1, 11: -2, 12: -1, 13: -1, 14: -1, 15: -1, 16: -1, 17: -1, 18: 89, 19: -1, 20: -1, 21: -2, 22: 7, 23: -1, 24: -2, 25: -1, 26: -1, 27: 83, 28: 10, 29: -1, 30: -1, 31: -1, 32: -1, 33: 8, 34: -1, 35: -1, 36: 91, 37: -1, 38: -1, 39: -1, 40: -1, 41: 2, 42: 66, 43: -1, 44: 65, 45: 43, 46: 2, 47: -1, 48: -1, 49: -1, 50: -2, 51: -1, 52: -1, 53: 87, 54: -1, 55: -1, 56: 47, 57: 7, 58: 8, 59: 43, 60: -1, 61: -1, 62: -1, 63: -2, 64: 15, 65: -1, 66: -1, 67: -1, 68: -1, 69: -1, 70: 61, 71: -1, 72: 54, 73: 16, 74: 9, 75: -1, 76: -1, 77: 71, 78: -1, 79: 69, 80: 61, 81: 69, 82: -1, 83: -1, 84: -1, 85: -1, 86: -1, 87: -1, 88: 19, 89: -1, 90: -1, 91: -1, 92: -1, 93: -2, 94: 54, 95: 30, 96: -1, 97: -1, 98: 9, 99: 78}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25766567.702515189, 2: 11550479.778420774, 3: 5309360.7035690071}")
        pass

    def test_100_3_sub_stochastic(self):
        result = simulation("./testroutes/test100-3/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0285522448279, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0350465994424, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00154438410258, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12002.0361247, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11921.7997164, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 12354.7932051, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 41, 3: -2, 4: -2, 5: 17, 6: -2, 7: 22, 8: 58, 9: 74, 10: 28, 11: -2, 12: -2, 13: -2, 14: 83, 15: 70, 16: 73, 17: -1, 18: 83, 19: 88, 20: -2, 21: -2, 22: -1, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: -1, 29: -2, 30: 95, 31: -2, 32: 28, 33: 58, 34: -2, 35: -2, 36: 91, 37: -2, 38: -2, 39: 81, 40: -2, 41: -1, 42: 66, 43: 45, 44: 65, 45: -1, 46: 41, 47: 56, 48: -2, 49: -2, 50: -2, 51: -2, 52: -2, 53: 87, 54: 94, 55: 22, 56: -1, 57: 22, 58: -1, 59: 45, 60: -2, 61: 70, 62: -2, 63: -2, 64: 70, 65: -1, 66: -1, 67: -2, 68: -2, 69: 81, 70: -1, 71: 77, 72: 94, 73: -1, 74: -1, 75: 74, 76: -1, 77: -1, 78: 99, 79: 81, 80: 70, 81: -1, 82: -2, 83: -1, 84: -1, 85: -2, 86: -2, 87: -1, 88: -1, 89: -2, 90: -2, 91: -1, 92: -2, 93: -2, 94: -1, 95: -1, 96: -2, 97: -2, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 34117317.054063261, 2: 11598995.187892754, 3: 3801114.0503829536, 4: 1727890.3481982246, 5: 629625.18908578646}")
        pass