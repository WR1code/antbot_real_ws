// Offline-only joint refinement of X=Link5_T_Camera and Y=Base_T_Board.
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace py = pybind11;
using Pose = std::array<double, 7>; // tx,ty,tz,qw,qx,qy,qz

struct Config {
  double translation_weight = 2000.0;
  double rotation_weight = 572.9577951308232;
  double huber_delta = 2.5;
  int max_iterations = 100;
  double max_mount_distance_m = 0.25;
  double observability_relative_threshold = 1e-6;
  double max_rms_degradation_fraction = 0.05;
  double translation_degradation_allowance_m = 0.0005;
  double rotation_degradation_allowance_rad = 0.0017453292519943296;
};

bool Valid(const Pose& p) {
  for (double v : p) if (!std::isfinite(v)) return false;
  double norm = 0;
  for (int k = 3; k < 7; ++k) norm += p[k] * p[k];
  return std::abs(norm - 1.0) < 1e-8;
}

Eigen::Quaterniond Q(const Pose& p) { return {p[3], p[4], p[5], p[6]}; }
Eigen::Vector3d T(const Pose& p) { return {p[0], p[1], p[2]}; }
Pose Pack(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  return {t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z()};
}
Pose Compose(const Pose& a, const Pose& b) {
  return Pack(Q(a) * Q(b), T(a) + Q(a) * T(b));
}

struct Residual {
  Pose a, c;
  double wt, wr;
  template <typename S>
  bool operator()(const S* qx, const S* tx, const S* qy, const S* ty, S* r) const {
    S qa[4], qc[4], tc[3], ta[3];
    for (int j = 0; j < 4; ++j) { qa[j] = S(a[j+3]); qc[j] = S(c[j+3]); }
    for (int j = 0; j < 3; ++j) { ta[j] = S(a[j]); tc[j] = S(c[j]); }
    S qax[4], predicted_q[4], x_tc[3], predicted_t[3];
    ceres::QuaternionProduct(qa, qx, qax);
    ceres::QuaternionProduct(qax, qc, predicted_q);
    ceres::UnitQuaternionRotatePoint(qx, tc, x_tc);
    for (int j = 0; j < 3; ++j) x_tc[j] += tx[j];
    ceres::UnitQuaternionRotatePoint(qa, x_tc, predicted_t);
    for (int j = 0; j < 3; ++j) r[j] = S(wt) * (predicted_t[j] + ta[j] - ty[j]);
    S inverse_y[4] = {qy[0], -qy[1], -qy[2], -qy[3]}, error_q[4], angle[3];
    ceres::QuaternionProduct(inverse_y, predicted_q, error_q);
    // Principal SO(3) logarithm: q and -q represent the same residual.
    if (error_q[0] < S(0)) for (int j = 0; j < 4; ++j) error_q[j] = -error_q[j];
    ceres::QuaternionToAngleAxis(error_q, angle);
    for (int j = 0; j < 3; ++j) r[j+3] = S(wr) * angle[j];
    return true;
  }
};

Pose InitialBoard(const std::vector<Pose>& a, const std::vector<Pose>& c, const Pose& x) {
  Eigen::Matrix4d scatter = Eigen::Matrix4d::Zero();
  Eigen::Vector3d translation;
  std::vector<Pose> boards;
  for (size_t i = 0; i < a.size(); ++i) {
    Pose b = Compose(Compose(a[i], x), c[i]);
    boards.push_back(b);
    Eigen::Vector4d v(b[3], b[4], b[5], b[6]);
    scatter += v * v.transpose();
  }
  for (int k = 0; k < 3; ++k) {
    std::vector<double> values;
    for (const auto& b : boards) values.push_back(b[k]);
    std::sort(values.begin(), values.end());
    translation[k] = values[values.size()/2];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(scatter);
  Eigen::Vector4d q = eigen.eigenvectors().col(3);
  return Pack(Eigen::Quaterniond(q[0], q[1], q[2], q[3]), translation);
}

std::array<double, 2> Rms(const std::vector<Pose>& a, const std::vector<Pose>& c,
                          const Pose& x, const Pose& y) {
  double st = 0, sr = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    Pose b = Compose(Compose(a[i], x), c[i]);
    st += (T(b) - T(y)).squaredNorm();
    Eigen::Quaterniond dq = Q(y).conjugate() * Q(b);
    double angle = 2 * std::atan2(dq.vec().norm(), std::abs(dq.w()));
    sr += angle * angle;
  }
  return {std::sqrt(st/a.size()), std::sqrt(sr/a.size())};
}

double RankRatio(ceres::Problem& problem) {
  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = false;
  ceres::CRSMatrix crs;
  if (!problem.Evaluate(options, nullptr, nullptr, nullptr, &crs)) return 0;
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(crs.num_rows, crs.num_cols);
  for (int row = 0; row < crs.num_rows; ++row)
    for (int k = crs.rows[row]; k < crs.rows[row+1]; ++k) j(row, crs.cols[k]) = crs.values[k];
  if (j.cols() != 12 || j.rows() < 12) return 0;
  // Remove parameter-unit scale before judging rank of the 12-DoF joint model.
  for (int k = 0; k < j.cols(); ++k) {
    double norm = j.col(k).norm();
    if (norm > 0) j.col(k) /= norm;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  return svd.singularValues()[11] / svd.singularValues()[0];
}

double RobotRankRatio(const std::vector<Pose>& poses) {
  // A separate robot-only guard prevents visual noise from creating artificial
  // full rank in an otherwise unobservable single-axis motion dataset.
  Eigen::MatrixXd j = Eigen::MatrixXd::Zero(6*(poses.size()-1), 6);
  Eigen::Quaterniond reference = Q(poses[0]).conjugate();
  for (size_t i = 1; i < poses.size(); ++i) {
    Eigen::Matrix3d r = (reference * Q(poses[i])).toRotationMatrix();
    Eigen::Vector3d t = reference * (T(poses[i])-T(poses[0])) / 0.1;
    Eigen::Matrix3d cross;
    cross << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
    j.block<3,3>(6*(i-1), 0) = r - Eigen::Matrix3d::Identity();
    j.block<3,3>(6*(i-1), 3) = cross * r;
    j.block<3,3>(6*(i-1)+3, 3) = r - Eigen::Matrix3d::Identity();
  }
  for (int k = 0; k < 6; ++k) {
    double norm = j.col(k).norm();
    if (norm > 0) j.col(k) /= norm;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  return svd.singularValues()[5] / svd.singularValues()[0];
}

py::dict Optimize(const std::vector<Pose>& a, const std::vector<Pose>& c,
                  const Pose& initial, const Config& cfg) {
  py::dict out;
  out["success"] = false; out["extrinsic"] = initial; out["board_pose"] = py::none();
  out["translation_rms"] = py::none(); out["rotation_rms"] = py::none();
  out["iterations"] = 0; out["final_cost"] = py::none(); out["initial_cost"] = py::none();
  auto reject = [&](const std::string& reason) {
    out["failure_reason"] = "CERES_REJECTED: " + reason; return out;
  };
  if (!Valid(initial)) return reject("INVALID_INITIAL_EXTRINSIC");
  if (T(initial).norm() > cfg.max_mount_distance_m) return reject("INITIAL_MOUNT_DISTANCE");
  if (a.size() != c.size() || a.size() < 4) return reject("INSUFFICIENT_OR_MISMATCHED_SAMPLES");
  for (size_t i = 0; i < a.size(); ++i)
    if (!Valid(a[i]) || !Valid(c[i])) return reject("INVALID_SAMPLE");
  for (double value : {cfg.translation_weight, cfg.rotation_weight, cfg.huber_delta,
       cfg.max_mount_distance_m, cfg.observability_relative_threshold})
    if (!std::isfinite(value) || value <= 0) return reject("INVALID_CONFIG");
  for (double value : {cfg.max_rms_degradation_fraction, cfg.translation_degradation_allowance_m,
       cfg.rotation_degradation_allowance_rad})
    if (!std::isfinite(value) || value < 0) return reject("INVALID_CONFIG");
  if (cfg.max_iterations < 1) return reject("INVALID_CONFIG");
  double robot_ratio = RobotRankRatio(a);
  out["robot_observability_ratio"] = robot_ratio;
  if (!std::isfinite(robot_ratio) || robot_ratio < cfg.observability_relative_threshold)
    return reject("DEGENERATE_MOTION: robot-only rank deficiency");
  Pose x = initial, y = InitialBoard(a, c, initial), initial_y = y;
  out["board_pose"] = initial_y;
  const auto initial_rms = Rms(a, c, initial, initial_y);
  out["translation_rms"] = initial_rms[0]; out["rotation_rms"] = initial_rms[1];
  ceres::Problem problem;
  problem.AddParameterBlock(x.data()+3, 4, new ceres::QuaternionManifold());
  problem.AddParameterBlock(x.data(), 3);
  problem.AddParameterBlock(y.data()+3, 4, new ceres::QuaternionManifold());
  problem.AddParameterBlock(y.data(), 3);
  for (size_t i = 0; i < a.size(); ++i) {
    auto* cost = new ceres::AutoDiffCostFunction<Residual, 6, 4, 3, 4, 3>(
      new Residual{a[i], c[i], cfg.translation_weight, cfg.rotation_weight});
    problem.AddResidualBlock(cost, new ceres::HuberLoss(cfg.huber_delta),
      x.data()+3, x.data(), y.data()+3, y.data());
  }
  double initial_cost = 0;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &initial_cost, nullptr, nullptr, nullptr);
  out["initial_cost"] = initial_cost; out["final_cost"] = initial_cost;
  double ratio = RankRatio(problem);
  out["observability_ratio"] = ratio;
  if (!std::isfinite(ratio) || ratio < cfg.observability_relative_threshold)
    return reject("DEGENERATE_MOTION");
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = cfg.max_iterations;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  { py::gil_scoped_release release; ceres::Solve(options, &problem, &summary); }
  out["iterations"] = summary.iterations.size();
  out["attempted_final_cost"] = summary.final_cost;
  if (!summary.IsSolutionUsable() || summary.termination_type != ceres::CONVERGENCE)
    return reject("SOLVER_DID_NOT_CONVERGE: " + summary.BriefReport());
  if (!Valid(x) || !Valid(y)) return reject("INVALID_OPTIMIZED_QUATERNION");
  if (T(x).norm() > cfg.max_mount_distance_m) return reject("OPTIMIZED_MOUNT_DISTANCE");
  if (!std::isfinite(summary.final_cost) || summary.final_cost > initial_cost + 1e-10)
    return reject("COST_INCREASED");
  double final_ratio = RankRatio(problem);
  if (!std::isfinite(final_ratio) || final_ratio < cfg.observability_relative_threshold)
    return reject("OPTIMIZED_DEGENERATE_MOTION");
  const auto final_rms = Rms(a, c, x, y);
  if (final_rms[0] > initial_rms[0]*(1+cfg.max_rms_degradation_fraction)
                       + cfg.translation_degradation_allowance_m
      || final_rms[1] > initial_rms[1]*(1+cfg.max_rms_degradation_fraction)
                       + cfg.rotation_degradation_allowance_rad)
    return reject("RMS_DEGRADED");
  out["success"] = true; out["extrinsic"] = x; out["board_pose"] = y;
  out["translation_rms"] = final_rms[0]; out["rotation_rms"] = final_rms[1];
  out["final_cost"] = summary.final_cost; out["failure_reason"] = "";
  return out;
}

PYBIND11_MODULE(_handeye_ceres, m) {
  py::class_<Config>(m, "Config").def(py::init<>())
    .def_readwrite("translation_weight", &Config::translation_weight)
    .def_readwrite("rotation_weight", &Config::rotation_weight)
    .def_readwrite("huber_delta", &Config::huber_delta)
    .def_readwrite("max_iterations", &Config::max_iterations)
    .def_readwrite("max_mount_distance_m", &Config::max_mount_distance_m)
    .def_readwrite("observability_relative_threshold", &Config::observability_relative_threshold)
    .def_readwrite("max_rms_degradation_fraction", &Config::max_rms_degradation_fraction)
    .def_readwrite("translation_degradation_allowance_m", &Config::translation_degradation_allowance_m)
    .def_readwrite("rotation_degradation_allowance_rad", &Config::rotation_degradation_allowance_rad);
  m.def("optimize_handeye", &Optimize, py::arg("robot_poses"), py::arg("camera_board_poses"),
        py::arg("initial_extrinsic"), py::arg("config") = Config());
}
