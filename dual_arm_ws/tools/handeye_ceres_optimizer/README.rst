Offline Ceres hand-eye refinement (Step 3-A)
==========================================

This directory is NOT a ROS package and is NOT loaded by any launch file.
No calibration session, hardware dataset, historical .calib file, controller,
GUI, or TF publisher is accessed. The Python wrapper is also not imported by
the automatic calibration node. Build outputs stay outside the source tree.

Build and synthetic regression tests (from repository root)::

    source activate.sh
    cmake -S dual_arm_ws/tools/handeye_ceres_optimizer \
      -B /tmp/antbot_handeye_ceres_step3a_build \
      -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
    cmake --build /tmp/antbot_handeye_ceres_step3a_build -j2
    PYTHONPATH=/tmp/antbot_handeye_ceres_step3a_build:$PYTHONPATH \
      python3 -m pytest -q -s dual_arm_ws/src/rebotarm_pulse/test/test_handeye_ceres.py

Requires Ceres >= 2.1, Eigen, pybind11, CMake, C++17, Python development headers.
Backend-dependent tests explicitly skip when the extension is unavailable;
run the command above to actually verify Ceres rather than a Python substitute.

Python API::

    from rebotarm_pulse.handeye_ceres import optimize_handeye
    result = optimize_handeye(robot_poses, camera_board_poses, initial_extrinsic,
                              config="dual_arm_ws/tools/handeye_ceres_optimizer/config.yaml")

Pass lists of existing Transform objects, or mappings with rotation/translation
or quaternion/translation. Mapping quaternion order is x,y,z,w. The internal
C++ interface uses [tx,ty,tz,qw,qx,qy,qz]; both inputs and outputs are validated.
The initial extrinsic must come from the existing closed-form solvers in normal
use. The module does not implement or select a closed-form solver itself.
Tests use the existing Park implementation on synthetic data, and perturb its
output in the noiseless recovery test to exercise refinement.

Model and units
---------------

A_i = Base_T_Link5; X = Link5_T_Camera; C_i = Camera_T_Board;
Y = Base_T_Board. Only X and one shared Y are optimized:

    r_t = translation(A_i X C_i) - translation(Y)             [metres]
    r_R = Log(R_Y^T R_(A_i X C_i))                            [radians]
    minimize 1/2 sum_i Huber(||[w_t*r_t, w_R*r_R]||^2)

Both transforms use unit quaternion manifolds and Euclidean translation blocks
(SE(3) represented on SO(3) x R^3). The six-dimensional residual is a split
translation/SO(3)-log residual, not a full coupled se(3) matrix logarithm.
QuaternionToAngleAxis implements the principal SO(3) log; q/-q are equivalent.
Huber acts on each complete weighted 6-vector, not independently on coordinates.

Weights, Huber scale, iteration limit, mount-distance cap and rejection checks
are configurable in the standalone YAML. Weights are inverse metres/radians.
RMS output is unweighted, includes ALL observations (including outliers), and
rotation_rms is radians. final_cost is the dimensionless robust Ceres objective.

Y is initialized from A_i X_initial C_i using median translation and a
quaternion scatter-matrix mean. Robot-only relative-motion rank is checked
before optimization; the full 12-column tangent Jacobian is checked before
and after optimization, with column scaling and a relative singular-value
threshold. These are local numerical observability checks, not a covariance
estimate or a guarantee against all poor experiment designs. Single-axis
rotation at a fixed position is degenerate, but rich multi-axis pure rotation
can be observable and must not automatically be rejected.

Acceptance and fallback
-----------------------

Reject invalid input, rank deficiency, nonconvergence, invalid quaternion,
excessive camera mount distance, increasing robust cost or excessive raw RMS
degradation. Equal cost is accepted for an already-optimal zero-noise initial
solution. Rejected attempts return the INITIAL extrinsic and CERES_REJECTED.
For a valid baseline, board_pose/RMS/final_cost describe that returned baseline;
attempted_final_cost is a separate diagnostic. Invalid input or unavailable
backend may have None for metrics. No fallback is silently published or saved
as a calibration. No installation into the live ROS runtime is performed here.

Future hardware preparation (not performed in Step 3-A)
------------------------------------------------------

Prepare version-matched observation/FK timestamps, a rigid fixed target,
multi-axis collision-safe motions, independent held-out validation, realistic
noise weights and mount-distance limits. Reconfirm J6 locking, cancellation
and physical emergency stop before any future live integration. Synthetic
poses test mathematics only: they are not reachable/visible robot targets.
