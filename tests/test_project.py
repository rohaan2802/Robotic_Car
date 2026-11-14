import unittest
from unittest.mock import Mock

import numpy as np

from robotic_car.__main__ import SAVED_MAP, demo_map
from robotic_car.client import CarClient
from robotic_car.mapping import OccupancyMap, Pose, heading_delta


class MappingTests(unittest.TestCase):
    def test_saved_grid(self):
        self.assertEqual(OccupancyMap.load(SAVED_MAP).counts(), (1195, 248801, 4))

    def test_ray_does_not_clear_hit_or_space_behind_it(self):
        grid = OccupancyMap(30)
        self.assertTrue(grid.update(Pose(), 4))
        self.assertAlmostEqual(grid.grid[grid.cell(3, 0)], 0.1)
        self.assertAlmostEqual(grid.grid[grid.cell(4, 0)], 0.9)
        self.assertEqual(grid.grid[grid.cell(5, 0)], 0.5)

    def test_missing_and_invalid_ranges_leave_map_untouched(self):
        grid = OccupancyMap()
        for value in (None, -1, 0, float("nan"), 101):
            self.assertFalse(grid.update(Pose(), value))
        self.assertTrue(np.all(grid.grid == 0.5))

    def test_negative_coordinates_and_edge_clipping(self):
        grid = OccupancyMap(8)
        self.assertEqual(grid.cell(-0.1, -0.1), (3, 3))
        self.assertTrue(grid.update(Pose(-2, 0, 180), 10))
        self.assertAlmostEqual(grid.grid[4, 0], 0.1)
        self.assertEqual(grid.grid[4, -1], 0.5)

    def test_shortest_turn_wrap(self):
        self.assertEqual(heading_delta(359, 1), 2)
        self.assertEqual(heading_delta(1, 359), -2)

    def test_demo_is_deterministic(self):
        self.assertTrue(np.array_equal(demo_map().grid, demo_map().grid))


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.session = Mock()
        self.client = CarClient("http://car/", session=self.session)

    def test_command_and_stop_contract(self):
        self.client.command("RL", 350)
        self.session.get.assert_called_with("http://car/command", timeout=(1, 1),
                                            params={"cmd": "RL", "ms": 350})
        self.client.command("S")
        self.session.get.assert_called_with("http://car/command", timeout=(1, 1),
                                            params={"cmd": "S"})

    def test_invalid_commands_never_reach_network(self):
        for command, duration in (("X", 250), ("F", 0), ("F", 1001), ("F", 250.5)):
            with self.assertRaises(ValueError):
                self.client.command(command, duration)
        self.session.get.assert_not_called()

    def test_telemetry(self):
        response = self.session.get.return_value
        response.text = "-1"
        self.assertIsNone(self.client.distance())
        response.text = "36.5"
        self.assertEqual(self.client.distance(), 36.5)
        response.json.return_value = dict(estimated_yaw=-1, estimated_pitch=2, estimated_roll=3)
        self.assertEqual(self.client.attitude().yaw, 359)


if __name__ == "__main__":
    unittest.main()
