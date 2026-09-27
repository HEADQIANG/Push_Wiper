import unittest

import numpy as np

from airbot_ie.force_control.core import AdmittanceConfig, AdmittanceController


class AdmittanceDynamicsTests(unittest.TestCase):
    def test_default_100_hz_converges_to_force_spring_equilibrium(self):
        for force_error_n in (0.5, -0.5):
            with self.subTest(force_error_n=force_error_n):
                controller = AdmittanceController(AdmittanceConfig())
                equilibrium_m = force_error_n / controller.config.stiffness_n_m
                errors = []
                for _ in range(500):
                    delta_m = controller.update(force_error_n, 0.01)
                    errors.append(abs(delta_m - equilibrium_m))
                    self.assertFalse(controller.displacement_limited)

                # A loose displacement tolerance can hide a speed-limited
                # numerical limit cycle. Check both the final error and drift.
                self.assertLess(errors[-1], 1e-12)
                self.assertLess(abs(controller.velocity_m_s), 1e-12)
                self.assertTrue(np.all(np.diff(errors) <= 1e-15))

    def test_small_unforced_perturbation_loses_energy(self):
        config = AdmittanceConfig()
        controller = AdmittanceController(config)
        controller.delta_m = 1e-6
        previous_energy = 0.5 * config.stiffness_n_m * controller.delta_m**2

        for _ in range(120):
            controller.update(0.0, 0.01)
            energy = (
                0.5 * config.mass_kg * controller.velocity_m_s**2
                + 0.5 * config.stiffness_n_m * controller.delta_m**2
            )
            self.assertLess(energy, previous_energy)
            previous_energy = energy

        self.assertLess(abs(controller.delta_m), 1e-14)
        self.assertLess(abs(controller.velocity_m_s), 1e-14)

    def test_variable_timesteps_including_maximum_remain_stable(self):
        controller = AdmittanceController(AdmittanceConfig())
        equilibrium_m = 0.5 / controller.config.stiffness_n_m
        for dt_s in [0.001, 0.02, 0.005, 0.1, 0.5, 0.003] * 40:
            delta_m = controller.update(0.5, dt_s)
            self.assertTrue(np.isfinite([delta_m, controller.velocity_m_s]).all())
            self.assertGreaterEqual(delta_m, 0.0)
            self.assertLessEqual(delta_m, equilibrium_m + 1e-15)
            self.assertFalse(controller.displacement_limited)

        self.assertAlmostEqual(controller.delta_m, equilibrium_m, places=12)
        self.assertLess(abs(controller.velocity_m_s), 1e-12)

    def test_bidirectional_speed_displacement_limits_and_reset(self):
        config = AdmittanceConfig()
        for sign, speed_limit in (
            (1.0, config.max_press_speed_m_s),
            (-1.0, config.max_retract_speed_m_s),
        ):
            with self.subTest(direction=sign):
                controller = AdmittanceController(config)
                controller.update(sign * 100.0, 0.01)
                self.assertAlmostEqual(controller.velocity_m_s, sign * speed_limit)
                self.assertAlmostEqual(controller.delta_m, sign * speed_limit * 0.01)

                for _ in range(20):
                    previous_delta_m = controller.delta_m
                    controller.update(sign * 100.0, 0.5)
                    self.assertLessEqual(abs(controller.delta_m), config.max_displacement_m)
                    self.assertLessEqual(
                        abs(controller.delta_m - previous_delta_m),
                        speed_limit * 0.5 + 1e-15,
                    )

                self.assertEqual(controller.delta_m, sign * config.max_displacement_m)
                self.assertEqual(controller.velocity_m_s, 0.0)
                self.assertTrue(controller.displacement_limited)

                # A change of force must let the state leave a saturated limit.
                controller.update(-sign * 100.0, 0.01)
                self.assertLess(abs(controller.delta_m), config.max_displacement_m)
                self.assertFalse(controller.displacement_limited)

                controller.update(sign * 100.0, 0.5)
                self.assertTrue(controller.displacement_limited)
                controller.reset()
                self.assertEqual(controller.delta_m, 0.0)
                self.assertEqual(controller.velocity_m_s, 0.0)
                self.assertFalse(controller.displacement_limited)
                self.assertEqual(controller.update(0.0, 0.01), 0.0)

    def test_zero_stiffness_converges_to_damped_terminal_velocity(self):
        config = AdmittanceConfig(stiffness_n_m=0.0)
        controller = AdmittanceController(config)
        previous_delta_m = 0.0
        for _ in range(100):
            controller.update(0.01, 0.01)
            self.assertGreater(controller.delta_m, previous_delta_m)
            previous_delta_m = controller.delta_m
            self.assertFalse(controller.displacement_limited)

        self.assertAlmostEqual(controller.velocity_m_s, 0.01 / config.damping_ns_m, places=12)


if __name__ == "__main__":
    unittest.main()
