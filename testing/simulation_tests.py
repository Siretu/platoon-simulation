import unittest

from run_simulation import one_simulation


class TestSimulation(unittest.TestCase):
    def test_greedy_1(self):
        result = one_simulation(100, "./testroutes/test100-1/", load=True, save=False)
        self.assertAlmostEqual(result["f_relat_before_convex_gr"], 0.0193179148343, delta=10**-10)
        self.assertAlmostEqual(result["f_relat_after_convex_gr"], 0.0231643496297, delta=10**-10)
        self.assertAlmostEqual(result["f_relat_before_convex_sub"], 0.0157962148698, delta=10**-10)
        self.assertAlmostEqual(result["f_relat_after_convex_sub"], 0.0188523614341, delta=10**-10)
        self.assertAlmostEqual(result["f_relat_spont_plat"], 0.00415249686699, delta=10**-10)

        self.assertAlmostEqual(result["f_total_before_convex_gr"], 7798.8310376, delta=10**-5)
        self.assertAlmostEqual(result["f_total_after_convex_gr"], 7768.24243451, delta=10**-5)
        self.assertAlmostEqual(result["f_total_before_convex_sub"], 7826.83720128, delta=10**-5)
        self.assertAlmostEqual(result["f_total_after_convex_sub"], 7802.53333049, delta=10**-5)
        pass