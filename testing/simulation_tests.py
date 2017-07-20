from __future__ import absolute_import

import unittest
import time

from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from route_calculation import generate_routes, get_path_data_sets, get_routes
from run_simulation import simulation, dynamic_simulation


class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.start = time.time()

    def tearDown(self):
        t = time.time() - self.start
        print "%s: %.3f" % (self.id(), t)

    def test_100_1_greedy(self):
        result = simulation("./testroutes/test100-1-new/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0312056293506, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0387170567773, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11537.801003, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11448.3440888, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: 55, 4: -2, 5: -1, 6: -2, 7: 55, 8: -2, 9: -2, 10: -2, 11: 97, 12: -2, 13: 40, 14: 72, 15: -1, 16: 15, 17: -1, 18: 92, 19: 65, 20: -2, 21: -1, 22: 65, 23: 17, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: -1, 32: 97, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: -1, 39: -2, 40: -1, 41: 17, 42: 95, 43: 79, 44: 5, 45: 38, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: 21, 53: 38, 54: -2, 55: -1, 56: -2, 57: 97, 58: -2, 59: 62, 60: -2, 61: -2, 62: -1, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 82, 76: -1, 77: -2, 78: -2, 79: -1, 80: 31, 81: -2, 82: -1, 83: -2, 84: 21, 85: -2, 86: 55, 87: 79, 88: -2, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: 82, 95: -1, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 32600409.10980824, 2: 8488208.3164058179, 3: 7834579.827420461, 4: 1746276.9944754844}")
        pass

    def test_100_1_greedy_dynamic(self):
        # generate_routes(10000, './testroutes/test10000-1/')
        # result = dynamic_simulation("./testroutes/test100-5/", GreedyPlatooning())
        # self.assertAlmostEqual(result["f_relat_before_convex"], 0.0193179148343, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_after_convex"], 0.0231643496297, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10 ** -10)
        #
        # self.assertAlmostEqual(result["f_total_before_convex"], 7798.8310376, delta=10 ** -5)
        # self.assertAlmostEqual(result["f_total_after_convex"], 7768.24243451, delta=10 ** -5)
        # self.assertAlmostEqual(result["f_total_default"], 7952.45590347, delta=10 ** -5)
        # self.assertEqual(str(result["leaders"]),
        #                  "{0: -2, 1: -2, 2: -2, 3: 12, 4: -1, 5: -2, 6: -2, 7: -2, 8: -2, 9: 88, 10: -2, 11: -1, 12: -1, 13: 92, 14: -2, 15: -2, 16: -2, 17: 88, 18: -2, 19: -2, 20: 88, 21: -2, 22: 76, 23: 43, 24: -2, 25: 33, 26: -2, 27: -1, 28: -2, 29: 79, 30: 64, 31: -1, 32: -2, 33: -1, 34: -2, 35: 68, 36: 4, 37: -2, 38: -2, 39: -2, 40: -2, 41: -1, 42: -2, 43: -1, 44: -1, 45: -2, 46: -2, 47: -2, 48: 31, 49: -2, 50: -2, 51: -1, 52: 63, 53: 27, 54: -2, 55: -2, 56: 51, 57: -2, 58: -2, 59: 75, 60: -2, 61: -1, 62: 68, 63: -1, 64: -1, 65: -2, 66: 44, 67: -2, 68: -1, 69: 76, 70: -2, 71: 74, 72: 68, 73: -2, 74: -1, 75: -1, 76: -1, 77: 51, 78: -2, 79: -1, 80: -2, 81: -2, 82: 61, 83: -2, 84: 68, 85: -2, 86: -2, 87: 11, 88: -1, 89: 41, 90: -2, 91: 74, 92: -1, 93: -2, 94: -2, 95: -2, 96: -2, 97: -2, 98: -2, 99: -2}")
        # self.assertEqual(str(result["size_stats"]),
        #                  "{1: 25601725.859428469, 2: 6248163.8194618141, 3: 622693.01303148433, 4: 590861.73708113644, 5: 770780.33196208626}")

        path_data = get_path_data_sets('./testroutes/test1000-1/')
        routes = get_routes('./testroutes/test1000-1/')
        # route_info = get_route_info(TEST_FOLDER)

        result = dynamic_simulation(GreedyPlatooning(), path_data_sets=path_data, folder='./testroutes/test1000-1/')
        # result = simulation('./testroutes/test1000-1/', GreedyPlatooning())
        pass

    def test_100_1_random(self):
        result = simulation("./testroutes/test100-1-new/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0299829263836, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0368592120584, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11552.3627138, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11470.4699839, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: -2, 4: -2, 5: -1, 6: 84, 7: 88, 8: -2, 9: -2, 10: -2, 11: 40, 12: -2, 13: 40, 14: 72, 15: 16, 16: -1, 17: 41, 18: 92, 19: 65, 20: -2, 21: 84, 22: 65, 23: 41, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: 80, 32: 57, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: 53, 39: -2, 40: -1, 41: -1, 42: -1, 43: 79, 44: 5, 45: 53, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: -2, 53: -1, 54: -2, 55: 86, 56: -2, 57: -1, 58: -2, 59: 62, 60: -2, 61: -2, 62: -1, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 82, 76: -1, 77: -2, 78: -2, 79: -1, 80: -1, 81: -2, 82: -1, 83: -2, 84: -1, 85: -2, 86: -1, 87: 79, 88: -1, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: 82, 95: 42, 96: -2, 97: 57, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 33551345.757609989, 2: 7283714.2831072155, 3: 7915080.7846573247, 4: 1919333.4227354845}")
        pass

    def test_100_1_sub_deterministic(self):
        result = simulation("./testroutes/test100-1-new/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0273863272861, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0338613632423, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11583.2867619, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11506.1726925, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -1, 2: -1, 3: -1, 4: -1, 5: 50, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: -1, 12: -1, 13: -1, 14: -1, 15: 16, 16: -1, 17: -1, 18: -1, 19: 2, 20: -1, 21: 17, 22: 2, 23: 17, 24: -1, 25: -1, 26: -1, 27: 36, 28: -1, 29: -1, 30: -1, 31: 80, 32: 97, 33: -1, 34: -1, 35: -1, 36: -1, 37: -1, 38: -1, 39: -1, 40: 13, 41: 17, 42: -1, 43: -1, 44: -1, 45: 38, 46: -1, 47: -1, 48: 76, 49: 91, 50: -1, 51: -1, 52: -2, 53: 38, 54: -1, 55: 86, 56: -1, 57: 97, 58: -2, 59: -1, 60: -1, 61: -2, 62: -2, 63: 13, 64: 76, 65: 2, 66: -1, 67: 76, 68: -1, 69: 13, 70: -1, 71: 25, 72: 14, 73: -1, 74: -1, 75: -1, 76: -1, 77: -1, 78: -2, 79: 87, 80: -1, 81: -1, 82: 75, 83: -1, 84: 17, 85: -1, 86: -1, 87: -1, 88: 7, 89: -1, 90: -1, 91: -1, 92: 18, 93: -1, 94: 75, 95: 42, 96: -1, 97: -1, 98: -1, 99: 34}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 20600342.781736106, 2: 8440275.622228004, 3: 5991308.5154216057, 4: 721856.07384091278, 5: 536825.43752836657}")
        pass

    def test_100_1_sub_stochastic(self):
        result = simulation("./testroutes/test100-1-new/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0291954758373, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0346069214157, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00570467709984, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11561.740811, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 11497.2935102, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 11909.4426563, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 65, 3: -1, 4: -2, 5: -2, 6: -1, 7: 88, 8: -2, 9: -2, 10: -2, 11: 97, 12: -2, 13: 40, 14: 72, 15: 16, 16: -1, 17: 23, 18: 92, 19: 65, 20: -2, 21: 23, 22: 65, 23: -1, 24: -2, 25: 76, 26: -2, 27: 36, 28: -2, 29: -2, 30: -2, 31: -2, 32: 97, 33: -2, 34: 99, 35: 91, 36: -1, 37: -2, 38: 45, 39: -2, 40: -1, 41: 23, 42: 95, 43: -2, 44: -1, 45: -1, 46: -2, 47: -2, 48: 76, 49: 91, 50: -2, 51: -2, 52: -2, 53: 45, 54: -2, 55: 86, 56: -2, 57: 97, 58: -2, 59: -1, 60: -2, 61: -2, 62: -2, 63: 40, 64: 76, 65: -1, 66: -2, 67: 76, 68: -2, 69: 40, 70: -2, 71: -2, 72: -1, 73: -2, 74: -2, 75: 94, 76: -1, 77: -2, 78: -2, 79: 87, 80: -2, 81: -2, 82: 94, 83: -2, 84: -2, 85: -2, 86: -1, 87: -1, 88: -1, 89: -2, 90: -2, 91: -1, 92: -1, 93: -2, 94: -1, 95: -1, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 31634554.829268999, 2: 6913786.7017658725, 3: 7032369.4490458332, 4: 2238474.3314653104}")
        pass

    def test_100_2_greedy(self):
        result = simulation("./testroutes/test100-2-new/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0329111531244, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0413314160261, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12892.2828783, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12780.0321667, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 21, 7: -2, 8: -2, 9: 23, 10: -2, 11: -2, 12: -2, 13: 44, 14: -2, 15: -1, 16: 44, 17: 15, 18: -2, 19: -2, 20: 55, 21: -1, 22: 95, 23: -1, 24: -2, 25: -2, 26: -1, 27: 52, 28: 26, 29: 32, 30: 99, 31: -1, 32: -1, 33: -2, 34: -2, 35: -2, 36: -1, 37: 15, 38: -2, 39: -2, 40: 36, 41: 21, 42: -1, 43: -2, 44: -1, 45: -2, 46: -1, 47: -2, 48: -2, 49: 52, 50: -2, 51: 26, 52: -1, 53: -2, 54: 42, 55: -1, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: -1, 62: 81, 63: -2, 64: -2, 65: -2, 66: 31, 67: 68, 68: -1, 69: -2, 70: 55, 71: -2, 72: 46, 73: -2, 74: -1, 75: -2, 76: -2, 77: -2, 78: 74, 79: 86, 80: 55, 81: -1, 82: 55, 83: -2, 84: 44, 85: -1, 86: -1, 87: -2, 88: 74, 89: -2, 90: 92, 91: 61, 92: -1, 93: 74, 94: -2, 95: -1, 96: -2, 97: 15, 98: 52, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 35539154.507064484, 2: 10561366.556359138, 3: 7803075.2662481386, 4: 2641362.7632923117, 5: 172714.86536592984}")
        pass

    def test_100_2_random(self):
        result = simulation("./testroutes/test100-2-new/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0315503176175, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0379461191624, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12910.4242066, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12825.1616343, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 41, 7: -2, 8: -2, 9: 23, 10: -2, 11: -2, 12: -2, 13: 84, 14: -2, 15: 37, 16: 84, 17: 37, 18: -2, 19: -2, 20: 80, 21: 41, 22: 95, 23: -1, 24: -2, 25: -2, 26: 28, 27: 52, 28: -1, 29: 32, 30: 99, 31: -1, 32: -1, 33: -2, 34: -2, 35: -2, 36: 90, 37: -1, 38: -2, 39: -2, 40: 90, 41: -1, 42: 54, 43: -2, 44: 84, 45: -2, 46: -1, 47: -2, 48: -2, 49: 52, 50: -2, 51: -2, 52: -1, 53: -2, 54: -1, 55: 80, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: 91, 62: 81, 63: -2, 64: -2, 65: -2, 66: 31, 67: 68, 68: -1, 69: -2, 70: 80, 71: -2, 72: 46, 73: -2, 74: 78, 75: -2, 76: -2, 77: -2, 78: -1, 79: 86, 80: -1, 81: -1, 82: 80, 83: -2, 84: -1, 85: -1, 86: -1, 87: -2, 88: 78, 89: -2, 90: -1, 91: -1, 92: -2, 93: -2, 94: -2, 95: -1, 96: -2, 97: 37, 98: 52, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 37046341.623296097, 2: 8724653.2825926654, 3: 7756817.718670412, 4: 3074762.7880789414, 5: 115098.54569187196}")
        pass

    def test_100_2_sub_deterministic(self):
        result = simulation("./testroutes/test100-2-new/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.027335652231, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0354834489811, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12966.6100044, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12857.9915451, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -1, 2: -1, 3: -1, 4: -1, 5: -1, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: -1, 12: -1, 13: -1, 14: -2, 15: -1, 16: 13, 17: 15, 18: -1, 19: 6, 20: -1, 21: 6, 22: 4, 23: -2, 24: -1, 25: -1, 26: 28, 27: -1, 28: -1, 29: -1, 30: -1, 31: -2, 32: -2, 33: -1, 34: -1, 35: -1, 36: -1, 37: 15, 38: -2, 39: -1, 40: 36, 41: 6, 42: 54, 43: -1, 44: 13, 45: -1, 46: 72, 47: -1, 48: -1, 49: 27, 50: -1, 51: -1, 52: 27, 53: -1, 54: -1, 55: -1, 56: -1, 57: -1, 58: -1, 59: -1, 60: -1, 61: 91, 62: 81, 63: -1, 64: -1, 65: -1, 66: -1, 67: 68, 68: -1, 69: -1, 70: 55, 71: -1, 72: -1, 73: -1, 74: -1, 75: -1, 76: -1, 77: -1, 78: 74, 79: -1, 80: 55, 81: -1, 82: 55, 83: -1, 84: 13, 85: 3, 86: 79, 87: -1, 88: 74, 89: -2, 90: -1, 91: -1, 92: -2, 93: 74, 94: -1, 95: 4, 96: -2, 97: 15, 98: 27, 99: 30}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 24023441.811232001, 2: 7439358.3779092357, 3: 6587465.0402527936, 4: 3150574.1376829529}")
        pass

    def test_100_2_sub_stochastic(self):
        result = simulation("./testroutes/test100-2-new/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0307297089716, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0366917388473, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00231776588647, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 12921.3637587, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 12841.8838061, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 13331.0221909, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 81, 1: -2, 2: -2, 3: 85, 4: 95, 5: -2, 6: 41, 7: -2, 8: -2, 9: -1, 10: -2, 11: -2, 12: -2, 13: 44, 14: -2, 15: 37, 16: 44, 17: 37, 18: -2, 19: -1, 20: 80, 21: 41, 22: 95, 23: -2, 24: -2, 25: -2, 26: 28, 27: 98, 28: -1, 29: -1, 30: 99, 31: -2, 32: -2, 33: -2, 34: -2, 35: -2, 36: 90, 37: -1, 38: -2, 39: -2, 40: 90, 41: -1, 42: 54, 43: -2, 44: -1, 45: -2, 46: 72, 47: -2, 48: -2, 49: 98, 50: -2, 51: -2, 52: 98, 53: -2, 54: -1, 55: 80, 56: -2, 57: 68, 58: -2, 59: -2, 60: -2, 61: 91, 62: 81, 63: -2, 64: -2, 65: -2, 66: -1, 67: 68, 68: -1, 69: -2, 70: 80, 71: -2, 72: -1, 73: -2, 74: 88, 75: -2, 76: -2, 77: -2, 78: 88, 79: 86, 80: -1, 81: -1, 82: 80, 83: -2, 84: 44, 85: -1, 86: -1, 87: -2, 88: -1, 89: -2, 90: -1, 91: -1, 92: -2, 93: 88, 94: -2, 95: -1, 96: -2, 97: 37, 98: -1, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 35757819.795039743, 2: 7755905.8848997764, 3: 8151986.8497821111, 4: 2898882.5818644846, 5: 115098.54569187196}")
        pass

    def test_100_3_greedy(self):
        result = simulation("./testroutes/test100-3-new/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0290610499886, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_after_convex"], 0.0179869580535, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00154438410258, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 11995.7499421, delta=10 ** -5)
        # self.assertAlmostEqual(result["f_total_after_convex"], 7332.56280913, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 12354.7932051, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: 41, 3: -2, 4: -2, 5: 17, 6: -2, 7: 22, 8: 33, 9: -1, 10: 28, 11: -2, 12: 53, 13: -2, 14: 27, 15: 64, 16: 73, 17: -1, 18: -1, 19: 88, 20: -2, 21: -2, 22: -1, 23: -2, 24: -2, 25: -2, 26: -2, 27: -1, 28: -1, 29: -2, 30: -1, 31: -2, 32: 28, 33: -1, 34: -2, 35: -2, 36: -1, 37: -2, 38: -2, 39: 81, 40: -2, 41: -1, 42: -1, 43: -1, 44: -1, 45: 43, 46: 41, 47: 56, 48: -2, 49: -2, 50: -2, 51: -2, 52: -2, 53: -1, 54: -1, 55: 22, 56: -1, 57: 22, 58: 33, 59: 43, 60: -2, 61: 80, 62: -2, 63: -1, 64: -1, 65: 44, 66: 42, 67: -2, 68: -2, 69: 81, 70: 64, 71: -1, 72: 54, 73: -1, 74: 9, 75: -2, 76: 63, 77: 71, 78: 99, 79: 81, 80: -1, 81: -1, 82: -2, 83: 18, 84: 93, 85: -2, 86: -2, 87: 53, 88: -1, 89: 18, 90: -2, 91: 36, 92: -2, 93: -1, 94: 54, 95: 30, 96: -2, 97: -2, 98: 9, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 33783187.189807586, 2: 13359300.135947535, 3: 5195430.8873951742, 4: 250453.40064170465}")
        pass

    def test_100_3_random(self):
        result = simulation("./testroutes/test100-3/", RandomPlatooning(0), False)
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0141753186322, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_after_convex"], 0.0172957237641, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7361.02382163, delta=10 ** -5)
        # self.assertAlmostEqual(result["f_total_after_convex"], 7337.72416506, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: 49, 2: 6, 3: -2, 4: -2, 5: -2, 6: -1, 7: -2, 8: -2, 9: 64, 10: -2, 11: -2, 12: -2, 13: -2, 14: 51, 15: -2, 16: 46, 17: -1, 18: -1, 19: -2, 20: -2, 21: -2, 22: -2, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: 35, 29: -2, 30: -2, 31: 92, 32: -2, 33: 18, 34: -2, 35: -1, 36: 74, 37: -2, 38: -2, 39: 90, 40: 41, 41: -1, 42: 82, 43: -2, 44: 67, 45: -2, 46: -1, 47: -2, 48: 64, 49: -1, 50: -2, 51: -1, 52: -2, 53: -2, 54: -2, 55: -2, 56: 17, 57: -2, 58: -2, 59: -2, 60: -2, 61: -2, 62: 74, 63: -2, 64: -1, 65: -2, 66: -2, 67: -1, 68: 85, 69: -2, 70: -2, 71: 80, 72: 46, 73: -1, 74: -1, 75: -2, 76: -2, 77: -2, 78: -2, 79: -2, 80: -1, 81: -2, 82: -1, 83: -1, 84: -2, 85: -1, 86: -2, 87: -2, 88: -2, 89: -1, 90: -1, 91: 73, 92: -1, 93: -2, 94: -1, 95: -2, 96: -2, 97: 89, 98: -2, 99: 94}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25420637.948255584, 2: 5481474.7609709045, 3: 866152.26922851591}")
        pass

    def test_100_3_sub_deterministic(self):
        result = simulation("./testroutes/test100-3/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0125678473015, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_after_convex"], 0.0151644964774, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7373.02659959, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7353.63776017, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: 49, 2: 6, 3: -1, 4: -1, 5: -1, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: -1, 12: -1, 13: -1, 14: 51, 15: -1, 16: -1, 17: 56, 18: -2, 19: -1, 20: -1, 21: -1, 22: -1, 23: -1, 24: -1, 25: -1, 26: -1, 27: 83, 28: 35, 29: -1, 30: -1, 31: -1, 32: -1, 33: -1, 34: -1, 35: -1, 36: 9, 37: -1, 38: -1, 39: -1, 40: -1, 41: 40, 42: -2, 43: -1, 44: 67, 45: -1, 46: 16, 47: -1, 48: 9, 49: -1, 50: -1, 51: -1, 52: -1, 53: -1, 54: -1, 55: -1, 56: -1, 57: -1, 58: -1, 59: -1, 60: -1, 61: -1, 62: -1, 63: -1, 64: 9, 65: -1, 66: -1, 67: -1, 68: 85, 69: -1, 70: 56, 71: 80, 72: -1, 73: -1, 74: 62, 75: -1, 76: -1, 77: -1, 78: -1, 79: -1, 80: -1, 81: -1, 82: 99, 83: -1, 84: -1, 85: -1, 86: -1, 87: -1, 88: -1, 89: -1, 90: 39, 91: 73, 92: -2, 93: -1, 94: 99, 95: -1, 96: -1, 97: 89, 98: -1, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 9159060.6495675184, 2: 4459110.2924609901, 3: 795497.05823349243}")
        pass

    def test_100_3_sub_stochastic(self):
        result = simulation("./testroutes/test100-3/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0137687486105, delta=10 ** -10)
        # self.assertAlmostEqual(result["f_relat_after_convex"], 0.0167303773043, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7364.05962675, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7341.94553306, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: 49, 2: 6, 3: -2, 4: -2, 5: -2, 6: -1, 7: -2, 8: -2, 9: 64, 10: -2, 11: -2, 12: -2, 13: -2, 14: 51, 15: -2, 16: 46, 17: 56, 18: -2, 19: -2, 20: -2, 21: -2, 22: -2, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: 35, 29: -2, 30: -1, 31: -1, 32: -2, 33: -1, 34: -2, 35: -1, 36: 74, 37: -2, 38: -2, 39: 90, 40: 41, 41: -1, 42: -2, 43: -2, 44: 67, 45: -2, 46: -1, 47: -2, 48: 64, 49: -1, 50: -2, 51: -1, 52: -2, 53: -2, 54: -2, 55: -2, 56: -1, 57: -2, 58: -2, 59: -2, 60: -2, 61: -2, 62: 74, 63: -2, 64: -1, 65: -2, 66: -2, 67: -1, 68: 85, 69: -2, 70: 56, 71: 80, 72: 46, 73: 91, 74: -1, 75: -2, 76: -2, 77: -2, 78: -2, 79: -2, 80: -1, 81: -2, 82: 99, 83: -1, 84: -2, 85: -1, 86: -2, 87: -2, 88: -2, 89: 97, 90: -1, 91: -1, 92: -2, 93: -2, 94: 99, 95: -2, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 24651793.843420655, 2: 4951254.0764002008, 3: 996005.33835114969}")
        pass