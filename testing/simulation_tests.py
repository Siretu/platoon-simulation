from __future__ import absolute_import

import unittest
import time

from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from run_simulation import simulation


class TestSimulation(unittest.TestCase):
    def setUp(self):
        self.start = time.time()

    def tearDown(self):
        t = time.time() - self.start
        print "%s: %.3f" % (self.id(), t)

    def test_100_1_greedy(self):
        result = simulation("./testroutes/test100-1/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0193179148343, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0231643496297, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7798.8310376, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7768.24243451, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7952.45590347, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: -2, 3: 12, 4: -1, 5: -2, 6: -2, 7: -2, 8: -2, 9: 88, 10: -2, 11: -1, 12: -1, 13: 92, 14: -2, 15: -2, 16: -2, 17: 88, 18: -2, 19: -2, 20: 88, 21: -2, 22: 76, 23: 43, 24: -2, 25: 33, 26: -2, 27: -1, 28: -2, 29: 79, 30: 64, 31: -1, 32: -2, 33: -1, 34: -2, 35: 68, 36: 4, 37: -2, 38: -2, 39: -2, 40: -2, 41: -1, 42: -2, 43: -1, 44: -1, 45: -2, 46: -2, 47: -2, 48: 31, 49: -2, 50: -2, 51: -1, 52: 63, 53: 27, 54: -2, 55: -2, 56: 51, 57: -2, 58: -2, 59: 75, 60: -2, 61: -1, 62: 68, 63: -1, 64: -1, 65: -2, 66: 44, 67: -2, 68: -1, 69: 76, 70: -2, 71: 74, 72: 68, 73: -2, 74: -1, 75: -1, 76: -1, 77: 51, 78: -2, 79: -1, 80: -2, 81: -2, 82: 61, 83: -2, 84: 68, 85: -2, 86: -2, 87: 11, 88: -1, 89: 41, 90: -2, 91: 74, 92: -1, 93: -2, 94: -2, 95: -2, 96: -2, 97: -2, 98: -2, 99: -2}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25601725.859428469, 2: 6248163.8194618141, 3: 622693.01303148433, 4: 590861.73708113644, 5: 770780.33196208626}")
        pass

    def test_100_1_random(self):
        result = simulation("./testroutes/test100-1/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0183496072971, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0219162685127, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7806.53146059, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7778.16774455, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7952.45590347, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: -2, 3: 12, 4: 36, 5: -2, 6: -2, 7: -2, 8: -2, 9: 88, 10: -2, 11: 87, 12: -1, 13: 92, 14: -2, 15: -2, 16: -2, 17: 88, 18: -2, 19: -2, 20: 88, 21: -2, 22: 76, 23: 43, 24: -2, 25: 33, 26: -2, 27: 53, 28: -2, 29: 79, 30: 64, 31: 48, 32: -2, 33: -1, 34: -2, 35: 68, 36: -1, 37: -2, 38: -2, 39: -2, 40: -2, 41: -1, 42: -2, 43: -1, 44: 66, 45: -2, 46: -2, 47: -2, 48: -1, 49: -2, 50: -2, 51: 56, 52: 63, 53: -1, 54: -2, 55: -2, 56: -1, 57: -2, 58: -2, 59: 75, 60: -2, 61: 82, 62: 68, 63: -1, 64: -1, 65: -2, 66: -1, 67: -2, 68: -1, 69: 76, 70: -2, 71: 74, 72: 68, 73: -2, 74: -1, 75: -1, 76: -1, 77: -2, 78: -2, 79: -1, 80: -2, 81: -2, 82: -1, 83: -2, 84: 68, 85: -2, 86: -2, 87: -1, 88: -1, 89: 41, 90: -2, 91: 74, 92: -1, 93: -2, 94: -2, 95: -2, 96: -2, 97: -2, 98: -2, 99: -2}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 26249127.870298259, 2: 5600761.8085920252, 3: 622693.01303148433, 4: 590861.73708113644, 5: 770780.33196208626}")
        pass

    def test_100_1_sub_deterministic(self):
        result = simulation("./testroutes/test100-1/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0157962148698, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0188523614341, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7826.83720128, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7802.53333049, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7952.45590347, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -1, 2: -1, 3: -1, 4: 36, 5: -1, 6: -1, 7: -1, 8: -1, 9: -1, 10: -1, 11: 87, 12: 3, 13: -1, 14: -1, 15: -1, 16: -1, 17: 88, 18: -1, 19: -1, 20: 88, 21: -1, 22: -1, 23: -1, 24: -1, 25: -1, 26: -1, 27: 53, 28: -1, 29: -1, 30: -1, 31: 48, 32: -1, 33: 25, 34: -1, 35: -1, 36: -1, 37: -1, 38: -1, 39: -1, 40: -1, 41: -2, 42: -1, 43: 23, 44: 66, 45: -1, 46: -1, 47: -1, 48: -1, 49: -1, 50: -1, 51: -1, 52: 63, 53: -1, 54: -1, 55: -1, 56: 51, 57: -1, 58: -1, 59: -1, 60: -1, 61: 82, 62: 84, 63: -1, 64: 30, 65: -1, 66: -1, 67: -1, 68: 84, 69: 22, 70: -1, 71: -1, 72: 84, 73: -1, 74: 71, 75: 59, 76: 22, 77: 51, 78: -1, 79: 29, 80: -1, 81: -1, 82: -1, 83: -1, 84: -1, 85: -1, 86: -1, 87: -1, 88: -1, 89: -1, 90: -1, 91: 71, 92: 13, 93: -1, 94: -1, 95: -1, 96: -1, 97: -1, 98: -1, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 9484135.615335064, 2: 4663184.2094001975, 3: 1000113.0460732654, 4: 753295.88636047312}")
        pass

    def test_100_1_sub_stochastic(self):
        result = simulation("./testroutes/test100-1/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.01767129648, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0206507786574, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7811.92569746, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7788.23149682, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7952.45590347, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -2, 2: -2, 3: 12, 4: 36, 5: -2, 6: -2, 7: -2, 8: -2, 9: 88, 10: -2, 11: 87, 12: -1, 13: 92, 14: -2, 15: -2, 16: -2, 17: 88, 18: -2, 19: -2, 20: 88, 21: -2, 22: 69, 23: 43, 24: -2, 25: 33, 26: -2, 27: 53, 28: -2, 29: 79, 30: 64, 31: 48, 32: -2, 33: -1, 34: -2, 35: 84, 36: -1, 37: -2, 38: -2, 39: -2, 40: -2, 41: -2, 42: -2, 43: -1, 44: 66, 45: -2, 46: -2, 47: -2, 48: -1, 49: -2, 50: -2, 51: 56, 52: 63, 53: -1, 54: -2, 55: -2, 56: -1, 57: -2, 58: -2, 59: 75, 60: -2, 61: 82, 62: 84, 63: -1, 64: -1, 65: -2, 66: -1, 67: -2, 68: 84, 69: -1, 70: -2, 71: 74, 72: 84, 73: -2, 74: -1, 75: -1, 76: -2, 77: -2, 78: -2, 79: -1, 80: -2, 81: -2, 82: -1, 83: -2, 84: -1, 85: -2, 86: -2, 87: -1, 88: -1, 89: -1, 90: -2, 91: 74, 92: -1, 93: -2, 94: -2, 95: -2, 96: -2, 97: -2, 98: -2, 99: -2}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 26443622.521254629, 2: 5140502.653726208, 3: 648653.20141033526, 4: 406201.99259330513, 5: 804836.65064051037}")
        pass

    def test_100_2_greedy(self):
        result = simulation("./testroutes/test100-2/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0140874339538, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.016692187673, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00199486926416, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7534.69486377, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7514.78840842, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7642.35604987, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 53, 1: -2, 2: -1, 3: 94, 4: -2, 5: -2, 6: 58, 7: 86, 8: -1, 9: -2, 10: 42, 11: -2, 12: -1, 13: -2, 14: -1, 15: -2, 16: -1, 17: -1, 18: -2, 19: -2, 20: -2, 21: 2, 22: 17, 23: -2, 24: -2, 25: -2, 26: 14, 27: 90, 28: 35, 29: -2, 30: -2, 31: -2, 32: -2, 33: -2, 34: -2, 35: -1, 36: -2, 37: -1, 38: -2, 39: -1, 40: -2, 41: -1, 42: -1, 43: -2, 44: 65, 45: -2, 46: -2, 47: -2, 48: 16, 49: -2, 50: 14, 51: -2, 52: -2, 53: -1, 54: 41, 55: -2, 56: -2, 57: -2, 58: -1, 59: -2, 60: 35, 61: -2, 62: -2, 63: -2, 64: 41, 65: -1, 66: -2, 67: -2, 68: -2, 69: -2, 70: 12, 71: -2, 72: -2, 73: -2, 74: -2, 75: -2, 76: -2, 77: 37, 78: 8, 79: -2, 80: -2, 81: -2, 82: 39, 83: 53, 84: -2, 85: -2, 86: -1, 87: -2, 88: -2, 89: -2, 90: -1, 91: -2, 92: -2, 93: -2, 94: -1, 95: -2, 96: -2, 97: -2, 98: -2, 99: 12}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 26001925.492620397, 2: 5175712.2278538225, 3: 1337247.6725777856}")
        pass

    def test_100_2_random(self):
        result = simulation("./testroutes/test100-2/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0133447954501, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0171897267732, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00199486926416, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7540.37037163, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7510.98603747, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7642.35604987, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 53, 1: -2, 2: 21, 3: 94, 4: -2, 5: -2, 6: 58, 7: 86, 8: 78, 9: -2, 10: 42, 11: -2, 12: 70, 13: -2, 14: 26, 15: -2, 16: -1, 17: -1, 18: -2, 19: -2, 20: -2, 21: -1, 22: 17, 23: -2, 24: -2, 25: -2, 26: -1, 27: 90, 28: 35, 29: -2, 30: -2, 31: -2, 32: -2, 33: -2, 34: -2, 35: -1, 36: -2, 37: -1, 38: -2, 39: 82, 40: -2, 41: 64, 42: -1, 43: -2, 44: 65, 45: -2, 46: -2, 47: -2, 48: 16, 49: -2, 50: 26, 51: -2, 52: -2, 53: -1, 54: 64, 55: -2, 56: -2, 57: -2, 58: -1, 59: -2, 60: 35, 61: -2, 62: -2, 63: -2, 64: -1, 65: -1, 66: -2, 67: -2, 68: -2, 69: -2, 70: -1, 71: -2, 72: -2, 73: -2, 74: -2, 75: -2, 76: -2, 77: 37, 78: -1, 79: -2, 80: -2, 81: -2, 82: -1, 83: 53, 84: -2, 85: -2, 86: -1, 87: -2, 88: -2, 89: -2, 90: -1, 91: -2, 92: -2, 93: -2, 94: -1, 95: -2, 96: -2, 97: -2, 98: -2, 99: 53}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 26156682.491977263, 2: 5465277.1851890385, 3: 447287.03753770294, 4: 445638.67834800022}")
        pass

    def test_100_2_sub_deterministic(self):
        result = simulation("./testroutes/test100-2/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.013142314989, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0168708673621, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00199486926416, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7541.9177994, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7513.42287462, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7642.35604987, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -1, 1: -1, 2: 21, 3: -1, 4: 28, 5: -1, 6: -1, 7: -1, 8: 78, 9: -1, 10: -1, 11: -1, 12: 70, 13: -1, 14: -1, 15: -1, 16: -2, 17: -2, 18: -1, 19: -1, 20: -1, 21: -1, 22: -1, 23: -1, 24: -1, 25: -2, 26: 14, 27: -1, 28: -1, 29: -1, 30: -2, 31: -1, 32: -1, 33: -1, 34: -2, 35: 28, 36: -1, 37: -2, 38: -1, 39: 82, 40: -1, 41: -1, 42: 10, 43: -1, 44: -1, 45: -1, 46: -1, 47: -1, 48: -1, 49: -1, 50: 14, 51: -1, 52: -1, 53: 0, 54: 41, 55: -1, 56: -1, 57: -1, 58: 6, 59: -1, 60: -1, 61: -1, 62: -1, 63: -1, 64: 41, 65: 44, 66: -1, 67: -1, 68: -1, 69: -1, 70: -1, 71: -1, 72: -1, 73: -1, 74: -1, 75: -1, 76: -1, 77: -1, 78: -1, 79: -1, 80: -1, 81: -1, 82: -1, 83: 0, 84: 3, 85: -1, 86: 7, 87: -1, 88: -1, 89: -1, 90: 27, 91: -1, 92: -1, 93: -1, 94: 3, 95: -1, 96: -1, 97: -1, 98: -1, 99: 0}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 8025476.2970562493, 2: 4955134.0016728593, 3: 617625.81965089124, 4: 445638.67834800022}")
        pass

    def test_100_2_sub_stochastic(self):
        result = simulation("./testroutes/test100-2/", SubModularityPlatooning(False))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0125868825984, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0154675850988, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00199486926416, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7546.1626115, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7524.14725731, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7642.35604987, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: 53, 1: -2, 2: 21, 3: 94, 4: -2, 5: -2, 6: 58, 7: 86, 8: 78, 9: -2, 10: 42, 11: -2, 12: 53, 13: -2, 14: 50, 15: -2, 16: -2, 17: -2, 18: -2, 19: -2, 20: -2, 21: -1, 22: -1, 23: -2, 24: -2, 25: -2, 26: -2, 27: 90, 28: 35, 29: -2, 30: -2, 31: -2, 32: -2, 33: -2, 34: -2, 35: -1, 36: -2, 37: -2, 38: -2, 39: 82, 40: -2, 41: 64, 42: -1, 43: -2, 44: 65, 45: -2, 46: -2, 47: -2, 48: -1, 49: -2, 50: -1, 51: -2, 52: -2, 53: -1, 54: 64, 55: -2, 56: -2, 57: -2, 58: -1, 59: -2, 60: 35, 61: -2, 62: -2, 63: -2, 64: -1, 65: -1, 66: -2, 67: -2, 68: -2, 69: -2, 70: -2, 71: -2, 72: -2, 73: -2, 74: -2, 75: -2, 76: -2, 77: -1, 78: -1, 79: -2, 80: -2, 81: -2, 82: -1, 83: 53, 84: -1, 85: -2, 86: -1, 87: -2, 88: -2, 89: -2, 90: -1, 91: -2, 92: -2, 93: -2, 94: -1, 95: -2, 96: -2, 97: -2, 98: -2, 99: 53}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25673163.847221877, 2: 4474448.3671114091, 3: 317843.38186727406, 4: 124470.0891424491, 5: 557048.34793500032}")
        pass

    def test_100_3_greedy(self):
        result = simulation("./testroutes/test100-3/", GreedyPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0152849692589, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0179869580535, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7352.73820568, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7332.56280913, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: -1, 2: -1, 3: -2, 4: -2, 5: -2, 6: 2, 7: -2, 8: -2, 9: -1, 10: -2, 11: -2, 12: -2, 13: -2, 14: -1, 15: -2, 16: 46, 17: 56, 18: -1, 19: -2, 20: -2, 21: -2, 22: -2, 23: -2, 24: -2, 25: -2, 26: -2, 27: -1, 28: -1, 29: -2, 30: 68, 31: 92, 32: -2, 33: 18, 34: -2, 35: 28, 36: 9, 37: -2, 38: -2, 39: 90, 40: 41, 41: -1, 42: -1, 43: -2, 44: -1, 45: -2, 46: -1, 47: -2, 48: 9, 49: 1, 50: -2, 51: 14, 52: -2, 53: -2, 54: -2, 55: -2, 56: -1, 57: -2, 58: -2, 59: -2, 60: -2, 61: -2, 62: 74, 63: -2, 64: 9, 65: -2, 66: -2, 67: 44, 68: -1, 69: -2, 70: 56, 71: -1, 72: 46, 73: 91, 74: -1, 75: -2, 76: -2, 77: -2, 78: -2, 79: -2, 80: 71, 81: -2, 82: 42, 83: 27, 84: -2, 85: 68, 86: -2, 87: -2, 88: -2, 89: 97, 90: -1, 91: -1, 92: -1, 93: -2, 94: -1, 95: -2, 96: -2, 97: -1, 98: -2, 99: 94}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25224379.390895441, 2: 5716197.1640679054, 3: 827688.42349165864}")
        pass

    def test_100_3_random(self):
        result = simulation("./testroutes/test100-3/", RandomPlatooning(0))
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0141753186322, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0172957237641, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7361.02382163, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7337.72416506, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: 49, 2: 6, 3: -2, 4: -2, 5: -2, 6: -1, 7: -2, 8: -2, 9: 64, 10: -2, 11: -2, 12: -2, 13: -2, 14: 51, 15: -2, 16: 46, 17: -1, 18: -1, 19: -2, 20: -2, 21: -2, 22: -2, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: 35, 29: -2, 30: -2, 31: 92, 32: -2, 33: 18, 34: -2, 35: -1, 36: 74, 37: -2, 38: -2, 39: 90, 40: 41, 41: -1, 42: 82, 43: -2, 44: 67, 45: -2, 46: -1, 47: -2, 48: 64, 49: -1, 50: -2, 51: -1, 52: -2, 53: -2, 54: -2, 55: -2, 56: 17, 57: -2, 58: -2, 59: -2, 60: -2, 61: -2, 62: 74, 63: -2, 64: -1, 65: -2, 66: -2, 67: -1, 68: 85, 69: -2, 70: -2, 71: 80, 72: 46, 73: -1, 74: -1, 75: -2, 76: -2, 77: -2, 78: -2, 79: -2, 80: -1, 81: -2, 82: -1, 83: -1, 84: -2, 85: -1, 86: -2, 87: -2, 88: -2, 89: -1, 90: -1, 91: 73, 92: -1, 93: -2, 94: -1, 95: -2, 96: -2, 97: 89, 98: -2, 99: 94}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 25420637.948255584, 2: 5481474.7609709045, 3: 866152.26922851591}")
        pass

    def test_100_3_sub_deterministic(self):
        result = simulation("./testroutes/test100-3/", SubModularityPlatooning())
        self.assertAlmostEqual(result["f_relat_before_convex"], 0.0125678473015, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0151644964774, delta=10 ** -10)
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
        self.assertAlmostEqual(result["f_relat_after_convex"], 0.0167303773043, delta=10 ** -10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.000189382683238, delta=10 ** -10)

        self.assertAlmostEqual(result["f_total_before_convex"], 7364.05962675, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_after_convex"], 7341.94553306, delta=10 ** -5)
        self.assertAlmostEqual(result["f_total_default"], 7466.86906988, delta=10 ** -5)
        self.assertEqual(str(result["leaders"]),
                         "{0: -2, 1: 49, 2: 6, 3: -2, 4: -2, 5: -2, 6: -1, 7: -2, 8: -2, 9: 64, 10: -2, 11: -2, 12: -2, 13: -2, 14: 51, 15: -2, 16: 46, 17: 56, 18: -2, 19: -2, 20: -2, 21: -2, 22: -2, 23: -2, 24: -2, 25: -2, 26: -2, 27: 83, 28: 35, 29: -2, 30: -1, 31: -1, 32: -2, 33: -1, 34: -2, 35: -1, 36: 74, 37: -2, 38: -2, 39: 90, 40: 41, 41: -1, 42: -2, 43: -2, 44: 67, 45: -2, 46: -1, 47: -2, 48: 64, 49: -1, 50: -2, 51: -1, 52: -2, 53: -2, 54: -2, 55: -2, 56: -1, 57: -2, 58: -2, 59: -2, 60: -2, 61: -2, 62: 74, 63: -2, 64: -1, 65: -2, 66: -2, 67: -1, 68: 85, 69: -2, 70: 56, 71: 80, 72: 46, 73: 91, 74: -1, 75: -2, 76: -2, 77: -2, 78: -2, 79: -2, 80: -1, 81: -2, 82: 99, 83: -1, 84: -2, 85: -1, 86: -2, 87: -2, 88: -2, 89: 97, 90: -1, 91: -1, 92: -2, 93: -2, 94: 99, 95: -2, 96: -2, 97: -1, 98: -2, 99: -1}")
        self.assertEqual(str(result["size_stats"]),
                         "{1: 24651793.843420655, 2: 4951254.0764002008, 3: 996005.33835114969}")
        pass