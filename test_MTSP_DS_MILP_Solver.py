from unittest import TestCase

from Location import Location
from MTSP_DS_MILP_Solver import MTSP_DS_MILP_Solver


class TestMTSP_DS_MILP_Solver(TestCase):

    @classmethod
    def setUpClass(self):
        self.model = MTSP_DS_MILP_Solver(6, 1, 2, 2)

    # def setUp(self):
    #     self.model = MTSP_DS_MILP_Solver(6, 1, 2, 2)

    def test_num_of_customers(self):
        self.assertEqual(self.model.n, 6, 'Wrong number of Customers')

    def test_custom_setup(self):
        custom_location_list = [Location(100, 130), Location(100, 90), Location(120, 100),
                                Location(100, 100), Location(49, 49)]
        model = MTSP_DS_MILP_Solver(4, 1, 2, 2, custom_locations=custom_location_list)
        loc = model.get_nodes_location()
        self.assertEqual(loc[1:-1], custom_location_list, "Custom location went wrong")
