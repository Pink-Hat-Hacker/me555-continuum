"""Quick tests for controller.py - no hardware. Run: python -m unittest test_controller"""
import unittest

import controller as c


class Recorder(c.MockBus):
    def __init__(self):
        super().__init__([1, 2, 3])
        self.calls, self.released = [], 0

    def set_force_n(self, sid, newtons):
        self.calls.append((sid, newtons))

    def release_all(self):
        self.released += 1


class ControllerTests(unittest.TestCase):
    def make(self):
        return c.Controller(Recorder(), base=2.0, tmin=0.5, tmax=15, max_delta=1.0)

    def test_clamping(self):
        ctrl = self.make()
        ctrl.set_forces([100, -100, 7])
        self.assertEqual(ctrl.targets, [13.0, -1.5, 7.0])   # offsets: base 2, floor 0.5, ceiling 15

    def test_rate_limit(self):
        ctrl = self.make()
        ctrl.set_forces([10, 0, 0])
        ctrl.tick()
        self.assertEqual(ctrl.sent[0], 1.0)       # not 12
        for _ in range(20):
            ctrl.tick()
        self.assertEqual(ctrl.sent[0], 12.0)      # base 2 + 10

    def test_zero_is_baseline_and_negative_is_reduced_not_limp(self):
        ctrl = self.make()
        ctrl.set_forces([0, -1, -3])
        for _ in range(5):
            ctrl.tick()
        self.assertEqual([f for _, f in ctrl.bus.calls[-3:]], [2.0, 1.0, 0.5])  # base, reduced, floor

    def test_validation(self):
        ctrl = self.make()
        for bad in ([1, 2], [1, 2, 3, 4], [1, 2, "x"], [1, None, 3], [1, 2, float("nan")], 5):
            with self.assertRaises((ValueError, TypeError), msg=repr(bad)):
                ctrl.set_forces(bad)
        ctrl.set_forces((1, 2.5, -1))             # tuples of ints/floats are fine

    def test_release_all(self):
        ctrl = self.make()
        ctrl.set_forces([5, 5, 5])
        ctrl.release_all()
        self.assertEqual(ctrl.targets, [0.0, 0.0, 0.0])
        self.assertEqual(ctrl.bus.released, 1)

    def test_run_loop_releases_on_exception(self):
        class Boom(c.InputSource):
            def poll(self, targets):
                raise RuntimeError("boom")
        ctrl = self.make()
        with self.assertRaises(RuntimeError):
            c.run_loop(ctrl, Boom())
        self.assertEqual(ctrl.bus.released, 1)


if __name__ == "__main__":
    unittest.main()
