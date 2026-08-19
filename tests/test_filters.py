import numpy as np

from assignment1_answer import KalmanFilterModel as VehicleFilter
from assignment2_filter import KalmanFilterModel as PendulumFilter
from pendulum import PendulumModel


def test_vehicle_filter_prediction_and_update():
    kf = VehicleFilter()
    kf.initialise(0.1, accel_std=0.0, meas_std=1.0, init_on_measurement=False)
    initial = kf.state.copy()
    kf.prediction_step()
    kf.update_step([1.0, -2.0])

    assert kf.state.shape == (4,)
    assert not np.array_equal(kf.state, initial)
    assert np.all(np.isfinite(kf.covariance))
    assert np.allclose(kf.covariance, kf.covariance.T)


def test_pendulum_model_update_is_physical():
    model = PendulumModel()
    model.initialise({'initial_position': 0.2, 'initial_velocity': 0.0,
                      'mass': 1.0, 'length': 0.5})
    model.update(0.01, 0.0)

    assert model.get_position() < 0.2
    assert model.get_velocity() < 0.0


def test_pendulum_filter_initializes_from_first_measurement():
    kf = PendulumFilter()
    kf.initialise(0.01, torque_std=0.01, meas_std=0.02,
                  init_on_measurement=True)
    assert kf.state is None

    kf.update_step(0.25)
    kf.prediction_step()
    assert kf.state.shape == (2, 1)
    assert kf.get_last_innovation() is None
