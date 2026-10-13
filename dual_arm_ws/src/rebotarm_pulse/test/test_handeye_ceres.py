"""Synthetic data ONLY: no disk datasets, calibration files, ROS, or robot."""

from dataclasses import replace
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from rebotarm_pulse.adaptive_handeye import (
    PosePair, Transform, rotation_exp, rotation_log, rotation_to_quaternion, solve_handeye,
)
from rebotarm_pulse.handeye_ceres import optimize_handeye


HAS_BACKEND = importlib.util.find_spec("_handeye_ceres") is not None
requires_ceres = pytest.mark.skipif(not HAS_BACKEND, reason="build standalone offline Ceres module first")


def bounded_noise(rng, limit):
    vector = rng.normal(size=3)
    return vector / np.linalg.norm(vector) * rng.uniform(0, limit)


def synthetic(seed=37, noisy=False, count=50):
    rng = np.random.default_rng(seed)
    x = Transform(rotation_exp([0.3, -0.2, 0.1]), [0.04, -0.025, 0.035])
    y = Transform(rotation_exp([-0.2, 0.1, 0.3]), [0.35, 0.1, 0.15])
    pairs = []
    for _ in range(count):
        a = Transform(rotation_exp(rng.uniform(-0.7, 0.7, 3)), rng.uniform(-0.2, 0.2, 3))
        c = x.inverse() @ a.inverse() @ y
        if noisy:
            c = Transform(c.rotation @ rotation_exp(bounded_noise(rng, np.deg2rad(0.1))),
                          c.translation + bounded_noise(rng, 0.0005))
            a = Transform(a.rotation @ rotation_exp(bounded_noise(rng, np.deg2rad(0.01))),
                          a.translation + bounded_noise(rng, 0.00005))
        pairs.append(PosePair(a, c))
    return x, y, pairs


def closed_initial(pairs):
    # Existing implementation, not a replacement/reimplementation of AX=XB.
    candidates = solve_handeye(pairs)
    return next(item.link5_T_camera for item in candidates if item.solver == "PARK")


def optimize(pairs, initial, config=None):
    return optimize_handeye([p.base_T_link5 for p in pairs],
                            [p.camera_T_target for p in pairs], initial, config)


def errors(estimate, truth):
    return (float(np.linalg.norm(estimate.translation - truth.translation)),
            float(np.rad2deg(np.linalg.norm(rotation_log(truth.rotation.T @ estimate.rotation)))))


@requires_ceres
def test_noiseless_recovers_x_and_shared_y_from_perturbed_closed_initial():
    x, y, pairs = synthetic()
    initial = closed_initial(pairs)
    initial = Transform(initial.rotation @ rotation_exp([0.03, -0.02, 0.01]),
                        initial.translation + [0.008, -0.004, 0.006])
    result = optimize(pairs, initial)
    assert result["success"], result["failure_reason"]
    print(f"noiseless: error={errors(result['extrinsic'], x)} (m,deg), "
          f"cost={result['initial_cost']:.6g}->{result['final_cost']:.6g}, "
          f"iterations={result['iterations']}")
    assert errors(result["extrinsic"], x)[0] < 1e-8
    assert errors(result["extrinsic"], x)[1] < 1e-6
    assert errors(result["board_pose"], y)[0] < 1e-8
    assert errors(result["board_pose"], y)[1] < 1e-6
    assert result["final_cost"] < result["initial_cost"]
    assert result["translation_rms"] < 1e-8 and result["rotation_rms"] < 1e-8


@requires_ceres
@pytest.mark.parametrize("seed", [37, 81, 152])
def test_bounded_visual_and_robot_noise_recovers_extrinsic(seed):
    x, _y, pairs = synthetic(seed, noisy=True)
    result = optimize(pairs, closed_initial(pairs))
    assert result["success"], result["failure_reason"]
    translation, rotation = errors(result["extrinsic"], x)
    print(f"noise seed={seed}: translation_error={translation:.9f} m, rotation_error={rotation:.6f} deg")
    assert translation < 0.001 and rotation < 0.15
    assert result["final_cost"] <= result["initial_cost"] + 1e-10


@requires_ceres
def test_huber_suppresses_corrupted_camera_observations():
    x, _y, clean = synthetic(noisy=True)
    initial = closed_initial(clean)
    contaminated = list(clean)
    for index in range(5):
        c = clean[index].camera_T_target
        contaminated[index] = replace(clean[index], camera_T_target=Transform(
            c.rotation @ rotation_exp([0.2, -0.15, 0.1]), c.translation + [0.08, -0.06, 0.04]))
    robust = optimize(contaminated, initial)
    least_squares = optimize(contaminated, initial, {"huber_delta": 1e9})
    assert robust["success"], robust["failure_reason"]
    assert least_squares["success"], least_squares["failure_reason"]
    robust_error, ls_error = errors(robust["extrinsic"], x), errors(least_squares["extrinsic"], x)
    print(f"outliers: Huber={robust_error}, near-LS={ls_error} (m,deg)")
    assert robust_error[0] < 0.001 and robust_error[1] < 0.15
    assert robust_error[0] < ls_error[0] and robust_error[1] < ls_error[1]


@requires_ceres
def test_fixed_position_single_axis_rotation_is_unobservable():
    x, y, _pairs = synthetic()
    poses = [Transform(rotation_exp([0, 0, angle]), [0, 0, 0]) for angle in np.linspace(-0.7, 0.7, 20)]
    pairs = [PosePair(a, x.inverse() @ a.inverse() @ y) for a in poses]
    result = optimize(pairs, x)
    assert not result["success"]
    assert "DEGENERATE_MOTION" in result["failure_reason"]
    np.testing.assert_array_equal(result["extrinsic"].translation, x.translation)


@requires_ceres
def test_quaternion_sign_does_not_change_result():
    _x, _y, pairs = synthetic(noisy=True)
    initial = closed_initial(pairs)
    quaternion = np.asarray(rotation_to_quaternion(initial.rotation))
    results = [optimize(pairs, {"translation": initial.translation,
                               "quaternion": quaternion * sign}) for sign in (1, -1)]
    assert all(result["success"] for result in results)
    np.testing.assert_allclose(results[0]["extrinsic"].rotation, results[1]["extrinsic"].rotation, atol=1e-12)
    np.testing.assert_allclose(results[0]["extrinsic"].translation, results[1]["extrinsic"].translation, atol=1e-12)


@requires_ceres
def test_backend_quaternion_sign_invariance_in_all_parameter_blocks():
    import _handeye_ceres
    from rebotarm_pulse.handeye_ceres import _pack, _unpack
    x, _y, pairs = synthetic(noisy=True)
    a = [_pack(p.base_T_link5) for p in pairs]
    c = [_pack(p.camera_T_target) for p in pairs]
    initial = _pack(x)
    positive = _handeye_ceres.optimize_handeye(a, c, initial)
    negate = lambda p: p[:3] + [-v for v in p[3:]]
    negative = _handeye_ceres.optimize_handeye([negate(p) for p in a],
                                              [negate(p) for p in c], negate(initial))
    assert positive["success"] and negative["success"]
    np.testing.assert_allclose(_unpack(positive["extrinsic"]).rotation,
                               _unpack(negative["extrinsic"]).rotation, atol=1e-10)
    np.testing.assert_allclose(positive["extrinsic"][:3], negative["extrinsic"][:3], atol=1e-10)


def test_missing_backend_returns_initial(monkeypatch):
    import rebotarm_pulse.handeye_ceres as module
    def unavailable(_name):
        raise ImportError("offline backend unavailable")
    monkeypatch.setattr(module.importlib, "import_module", unavailable)
    initial = Transform.identity()
    result = optimize_handeye([], [], initial)
    assert not result["success"] and "CERES_REJECTED" in result["failure_reason"]
    np.testing.assert_array_equal(result["extrinsic"].rotation, initial.rotation)


@requires_ceres
def test_visual_noise_cannot_hide_robot_degeneracy():
    x, y, _pairs = synthetic()
    rng = np.random.default_rng(57)
    poses = [Transform(rotation_exp([0, 0, angle]), [0, 0, 0]) for angle in np.linspace(-0.7, 0.7, 20)]
    pairs = []
    for a in poses:
        c = x.inverse() @ a.inverse() @ y
        c = Transform(c.rotation @ rotation_exp(bounded_noise(rng, np.deg2rad(0.1))),
                      c.translation + bounded_noise(rng, 0.0005))
        pairs.append(PosePair(a, c))
    assert "DEGENERATE_MOTION" in optimize(pairs, x)["failure_reason"]


@requires_ceres
def test_multi_axis_pure_rotation_is_not_misclassified_as_degenerate():
    x, y, generated = synthetic()
    poses = [Transform(p.base_T_link5.rotation, [0, 0, 0]) for p in generated]
    pairs = [PosePair(a, x.inverse() @ a.inverse() @ y) for a in poses]
    result = optimize(pairs, x)
    assert result["success"], result["failure_reason"]


@requires_ceres
def test_solver_nonconvergence_returns_initial_not_partial_result():
    _x, _y, pairs = synthetic()
    initial = Transform(rotation_exp([0.6, -0.4, 0.5]), [0.1, -0.08, 0.07])
    result = optimize(pairs, initial, {"max_iterations": 1})
    assert not result["success"] and "SOLVER_DID_NOT_CONVERGE" in result["failure_reason"]
    np.testing.assert_array_equal(result["extrinsic"].translation, initial.translation)
    np.testing.assert_array_equal(result["extrinsic"].rotation, initial.rotation)


@pytest.mark.parametrize("initial", [
    {"rotation": np.zeros((3, 3)), "translation": [0, 0, 0]},
    {"quaternion": [0, 0, 0, 0], "translation": [0, 0, 0]},
    {"quaternion": [0, 0, 0, 1], "translation": [np.nan, 0, 0]},
])
def test_invalid_initial_fails_before_backend(initial):
    result = optimize_handeye([], [], initial)
    assert not result["success"] and "INVALID_INITIAL_EXTRINSIC" in result["failure_reason"]
    assert result["extrinsic"] is initial


@requires_ceres
@pytest.mark.parametrize("config", [{"max_mount_distance_m": 0.01}, {"translation_weight": -1},
                                    {"unknown": 1}, {"max_iterations": 0}])
def test_invalid_configuration_or_mount_distance_falls_back(config):
    x, _y, pairs = synthetic()
    result = optimize(pairs, x, config)
    assert not result["success"] and "CERES_REJECTED" in result["failure_reason"]
    np.testing.assert_array_equal(result["extrinsic"].translation, x.translation)


@requires_ceres
def test_offline_yaml_configuration_loads():
    x, _y, pairs = synthetic()
    config = Path(__file__).resolve().parents[3] / "tools/handeye_ceres_optimizer/config.yaml"
    assert optimize(pairs, x, config)["success"]


@requires_ceres
def test_optimized_mount_distance_rejection_returns_initial():
    x, y, generated = synthetic()
    far_x = Transform(x.rotation, [0.3, -0.1, 0.15])
    pairs = [PosePair(p.base_T_link5, far_x.inverse() @ p.base_T_link5.inverse() @ y)
             for p in generated]
    result = optimize(pairs, x)
    assert not result["success"]
    assert "OPTIMIZED_MOUNT_DISTANCE" in result["failure_reason"]
    np.testing.assert_array_equal(result["extrinsic"].translation, x.translation)
    assert result["final_cost"] == result["initial_cost"]


@requires_ceres
def test_missing_or_mismatched_samples_are_rejected():
    x, _y, pairs = synthetic()
    assert not optimize(pairs[:3], x)["success"]
    result = optimize_handeye([p.base_T_link5 for p in pairs], [], x)
    assert "INSUFFICIENT_OR_MISMATCHED_SAMPLES" in result["failure_reason"]
