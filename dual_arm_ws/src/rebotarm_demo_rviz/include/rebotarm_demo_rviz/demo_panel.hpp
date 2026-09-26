#ifndef REBOTARM_DEMO_RVIZ__DEMO_PANEL_HPP_
#define REBOTARM_DEMO_RVIZ__DEMO_PANEL_HPP_

#include <atomic>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <QImage>
#include <QMap>
#include <QPointer>
#include <QStringList>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rebot_teach_msgs/srv/list_action_groups.hpp"
#include "rebot_teach_msgs/srv/check_shape_reachability.hpp"
#include "rebot_teach_msgs/srv/configure_forbidden_zone.hpp"
#include "rebot_teach_msgs/srv/configure_shape_marker.hpp"
#include "rebot_teach_msgs/srv/configure_trace.hpp"
#include "rebot_teach_msgs/srv/copy_action_group.hpp"
#include "rebot_teach_msgs/srv/create_shape_action.hpp"
#include "rebot_teach_msgs/srv/delete_action_group.hpp"
#include "rebot_teach_msgs/srv/list_action_sequences.hpp"
#include "rebot_teach_msgs/srv/preview_action_group.hpp"
#include "rebot_teach_msgs/srv/preview_action_sequence.hpp"
#include "rebot_teach_msgs/srv/replay_action_group.hpp"
#include "rebot_teach_msgs/srv/replay_action_sequence.hpp"
#include "rebot_teach_msgs/srv/rename_action_group.hpp"
#include "rebot_teach_msgs/srv/save_action_sequence.hpp"
#include "rebot_teach_msgs/srv/select_action_group.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QDialog;
class QDockWidget;
class QGroupBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QProcess;
class QProgressBar;
class QPushButton;
class QSlider;
class QSpinBox;
class QTimer;
class QTabWidget;
class QTableWidget;
class QVBoxLayout;
class QWidget;

namespace rviz_common
{
class Display;
}

namespace rebotarm_demo_rviz
{

class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DemoPanel(QWidget * parent = nullptr);
  ~DemoPanel() override;
  void onInitialize() override;
  void setActiveRobot(const QString & robot);

private Q_SLOTS:
  void startPickPlace();
  void goHome();
  void startSingularityEscape();
  void resetSingularityEscapeTarget();
  void syncPlanningState();
  void stopDemo();
  void toggleXboxControl();
  void toggleCamera();
  void toggleHandVision();
  void startPulseApproach();
  void refreshHandVisionTopics();
  void selectHandVisionTopic(const QString & topic);
  void toggleHandeyeCalibration();
  void refreshCameraTopics();
  void selectCameraTopic(const QString & topic);
  void reloadForbiddenZones();
  void selectForbiddenZoneMesh();
  void applyForbiddenZone();
  void removeForbiddenZone();
  void saveForbiddenZoneGroup();
  void removeForbiddenZoneGroup();
  void updateReadiness();
  void readProcessOutput();
  void processFinished(int exit_code);
  void refreshTeachActions();
  void createShapeAction();
  void startNewShapeAction();
  void checkShapeReachability();
  void selectShapeImage();
  void toggleShapeMarker();
  void resetShapeMarker();
  void applyShapePose();
  void setTraceLineWidth();
  void copyTeachAction();
  void renameTeachAction();
  void deleteTeachAction();
  void startTeaching();
  void stopTeaching();
  void previewTeachAction();
  void previewTeachPosition();
  void replayTeachAction();
  void pauseTeachPreview();
  void cancelTeaching();
  void refreshTeachSequences();
  void addTeachSequenceAction();
  void removeTeachSequenceAction();
  void moveTeachSequenceActionUp();
  void moveTeachSequenceActionDown();
  void saveTeachSequence();
  void playTeachSequence();
  void previewTeachSequencePosition();
  void replayTeachSequence();
  void integrateMotionPlanningPanel();

private:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GetStateValidity = moveit_msgs::srv::GetStateValidity;
  using ListActionGroups = rebot_teach_msgs::srv::ListActionGroups;
  using CheckShapeReachability = rebot_teach_msgs::srv::CheckShapeReachability;
  using ConfigureForbiddenZone = rebot_teach_msgs::srv::ConfigureForbiddenZone;
  using ConfigureShapeMarker = rebot_teach_msgs::srv::ConfigureShapeMarker;
  using ConfigureTrace = rebot_teach_msgs::srv::ConfigureTrace;
  using CopyActionGroup = rebot_teach_msgs::srv::CopyActionGroup;
  using CreateShapeAction = rebot_teach_msgs::srv::CreateShapeAction;
  using DeleteActionGroup = rebot_teach_msgs::srv::DeleteActionGroup;
  using ListActionSequences = rebot_teach_msgs::srv::ListActionSequences;
  using PreviewActionGroup = rebot_teach_msgs::srv::PreviewActionGroup;
  using PreviewActionSequence = rebot_teach_msgs::srv::PreviewActionSequence;
  using ReplayActionGroup = rebot_teach_msgs::srv::ReplayActionGroup;
  using ReplayActionSequence = rebot_teach_msgs::srv::ReplayActionSequence;
  using RenameActionGroup = rebot_teach_msgs::srv::RenameActionGroup;
  using SaveActionSequence = rebot_teach_msgs::srv::SaveActionSequence;
  using SelectActionGroup = rebot_teach_msgs::srv::SelectActionGroup;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  struct ValidityState
  {
    std::atomic<bool> pending{false};
    std::atomic<bool> valid{false};
    std::atomic<std::int64_t> response_ms{0};
  };

  void startDemo(
    const QString & executable, const QString & title,
    bool require_confirmation = true);
  void setStatus(const QString & zh, const QString & en, bool error = false);
  void retranslateUi();
  QString localized(const QString & zh, const QString & en) const;
  QString operationTitle(const QString & executable, bool chinese) const;
  bool feedbackFresh() const;
  bool stateValidityFresh() const;
  QString detectedModel() const;
  QString configPath(const QString & executable) const;
  QString pulseConfigPath() const;
  QString jointTargetConfigPath() const;
  void configureSingularityEscapeControls();
  bool handeyeEyeInHand() const;
  bool handeyeUsesMoveItCalibration() const;
  bool showMoveItCalibrationDisplay(const QStringList & fields, bool eye_in_hand);
  void configureMoveItCalibrationDisplay(const QStringList & fields, bool eye_in_hand);
  void removeMoveItCalibrationDisplay();
  QString automaticHandeyeSequenceName() const;
  QString handeyeCalibrationFile() const;
  void restartHandeyePublisher();
  QString selectedTeachAction() const;
  QString selectedDrawingSource() const;
  QString selectedTeachSequence() const;
  void loadSelectedShapeParameters();
  void loadSelectedTeachSequence();
  void updateTeachSequenceNumbering();
  void updateHardwareReplayProgress(
    const QString & action_name, const QString & sequence_name,
    int action_index, int total_actions, double progress, bool active,
    int state, const QString & state_message);
  void setTeachStatus(const QString & zh, const QString & en, bool error = false);
  void handleTriggerResponse(
    rclcpp::Client<Trigger>::SharedFuture future,
    const QString & success_zh, const QString & success_en,
    bool refresh_after = false);
  void restoreMotionPlanningPanel();
  void configureShapeMarker(
    bool visible, bool reset_pose, bool report_success = true, bool set_pose = false);
  void updateShapePoseControls(
    double x, double y, double z,
    double qx, double qy, double qz, double qw,
    double width, double height, double pen_length_m, double pen_lift_m);
  void updateShapePreview();
  void ensureShapeMarkerDisplay();
  void ensureForbiddenZoneMarkerDisplay();
  void ensurePulseMarkerDisplay();
  void setMoveItGoalMarkerVisible(bool visible);
  void configureRobotDisplays();
  void updateFreePlanningVisibility();
  void updateMotionPlanningResult();
  void clearPlannedPathDisplay();
  bool cameraNodeRunning() const;
  bool cameraStreamPublished() const;
  void showCameraImage(const QImage & image);
  void showHandVisionImage(const QImage & image);
  void showHandVisionLargeView();
  void subscribeCameraTopic(const QString & topic);
  void updateForbiddenZoneUi();
  void configureRobotInterfaces();
  void configureForbiddenZoneInterfaces();
  void updateForbiddenZoneEditor();
  void clearTeachActionSelection();
  void resetNewActionParameters();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr hand_vision_image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pulse_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pulse_base_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr piper_pulse_base_point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr xbox_armed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teach_status_sub_;
  std::function<void(const std_msgs::msg::String::SharedPtr)> teach_status_handler_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr auto_handeye_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forbidden_zone_status_sub_;
  std::function<void(
      const QString &, const std_msgs::msg::String::SharedPtr)>
  forbidden_zone_status_handler_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_sub_;
  std::function<void(moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr)>
  display_trajectory_handler_;
  rclcpp::Client<GetStateValidity>::SharedPtr validity_client_;
  rclcpp::Client<SetBool>::SharedPtr xbox_mode_client_;
  rclcpp::Client<ListActionGroups>::SharedPtr teach_list_client_;
  rclcpp::Client<ConfigureShapeMarker>::SharedPtr teach_shape_marker_client_;
  rclcpp::Client<ConfigureTrace>::SharedPtr teach_trace_client_;
  rclcpp::Client<CreateShapeAction>::SharedPtr teach_shape_client_;
  rclcpp::Client<CheckShapeReachability>::SharedPtr teach_shape_reachability_client_;
  rclcpp::Client<ListActionSequences>::SharedPtr teach_sequence_list_client_;
  rclcpp::Client<CopyActionGroup>::SharedPtr teach_copy_client_;
  rclcpp::Client<RenameActionGroup>::SharedPtr teach_rename_client_;
  rclcpp::Client<DeleteActionGroup>::SharedPtr teach_delete_client_;
  rclcpp::Client<SelectActionGroup>::SharedPtr teach_select_client_;
  rclcpp::Client<PreviewActionGroup>::SharedPtr teach_preview_client_;
  rclcpp::Client<PreviewActionSequence>::SharedPtr teach_sequence_preview_client_;
  rclcpp::Client<ReplayActionGroup>::SharedPtr teach_replay_client_;
  rclcpp::Client<SaveActionSequence>::SharedPtr teach_sequence_save_client_;
  rclcpp::Client<ReplayActionSequence>::SharedPtr teach_sequence_replay_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_start_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_stop_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_cancel_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_reset_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_pause_client_;
  rclcpp::Client<Trigger>::SharedPtr teach_clear_selection_client_;
  rclcpp::Client<Trigger>::SharedPtr forbidden_zone_reload_client_;
  rclcpp::Client<ConfigureForbiddenZone>::SharedPtr forbidden_zone_configure_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr update_start_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr update_goal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr rviz_stop_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  std::shared_ptr<ValidityState> validity_state_{std::make_shared<ValidityState>()};
  mutable std::mutex joint_state_mutex_;
  mutable std::mutex pulse_point_mutex_;
  mutable std::mutex teach_preview_source_mutex_;
  std::string teach_preview_sequence_name_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  geometry_msgs::msg::PointStamped::SharedPtr latest_pulse_point_;
  geometry_msgs::msg::PointStamped::SharedPtr latest_piper_pulse_point_;
  std::atomic<std::int64_t> last_feedback_ms_{0};
  std::atomic<bool> xbox_state_known_{false};
  std::atomic<bool> xbox_armed_{true};
  std::atomic<bool> xbox_request_pending_{false};
  std::atomic<bool> camera_detected_{false};
  std::atomic<bool> camera_stream_published_{false};
  std::atomic<bool> camera_stop_requested_{false};
  std::atomic<std::uint64_t> free_planning_display_generation_{0};
  bool free_planning_visible_{false};
  QString last_motion_planning_result_;
  std::atomic<std::int64_t> last_camera_image_ms_{0};
  std::atomic<std::int64_t> last_camera_render_ms_{0};
  std::atomic<std::int64_t> last_camera_frame_ms_{0};
  std::atomic<std::int64_t> last_hand_vision_image_ms_{0};
  std::atomic<std::int64_t> last_pulse_point_ms_{0};
  std::atomic<std::int64_t> last_piper_pulse_point_ms_{0};
  std::atomic<int> teach_state_{0};
  std::atomic<bool> teach_hardware_allowed_{false};
  std::atomic<bool> teach_normal_feedback_ready_{false};
  std::atomic<bool> teach_passive_recording_ready_{false};
  std::atomic<bool> teach_offline_preview_{false};
  std::atomic<bool> teach_request_pending_{false};
  std::atomic<bool> return_home_after_teach_cancel_pending_{false};
  std::atomic<bool> shape_reachability_request_pending_{false};
  std::atomic<bool> trace_width_request_pending_{false};
  std::atomic<std::int64_t> trace_width_request_generation_{0};
  std::atomic<bool> shape_marker_visible_{true};
  std::atomic<bool> teach_preview_active_{false};
  std::atomic<bool> teach_preview_paused_{false};
  std::atomic<std::int64_t> teach_preview_generation_{0};
  std::atomic<std::int64_t> teach_selection_generation_{0};
  std::atomic<std::int64_t> teach_preview_started_ms_{0};
  std::atomic<std::int64_t> teach_preview_duration_ms_{0};
  std::atomic<std::int64_t> teach_preview_offset_milli_{0};
  std::int64_t last_validity_request_ms_{0};
  std::int64_t last_camera_graph_check_ms_{0};

  QComboBox * language_combo_;
  QLabel * language_label_;
  QTabWidget * main_tabs_;
  QTabWidget * teach_tabs_;
  QWidget * planning_page_;
  QVBoxLayout * planning_layout_;
  QLabel * planning_placeholder_;
  QLabel * planning_error_label_;
  QPointer<QDockWidget> motion_planning_dock_;
  QPointer<QWidget> embedded_motion_planning_widget_;
  QPointer<QWidget> motion_planning_dock_placeholder_;
  QGroupBox * connection_box_;
  QLabel * model_label_;
  QLabel * feedback_label_;
  QLabel * validity_label_;
  QLabel * xbox_label_;
  QLabel * camera_label_;
  QGroupBox * safety_box_;
  QGroupBox * forbidden_zone_box_;
  QLabel * forbidden_zone_state_label_;
  QLabel * forbidden_zone_areas_label_;
  QLabel * forbidden_zone_groups_label_;
  QLabel * forbidden_zone_config_label_;
  QLabel * forbidden_zone_hint_label_;
  QPushButton * forbidden_zone_reload_button_;
  QLineEdit * forbidden_zone_name_edit_;
  QComboBox * forbidden_zone_shape_combo_;
  QLabel * forbidden_zone_dimensions_label_;
  QDoubleSpinBox * forbidden_zone_dimension_1_spin_;
  QDoubleSpinBox * forbidden_zone_dimension_2_spin_;
  QDoubleSpinBox * forbidden_zone_dimension_3_spin_;
  QLabel * forbidden_zone_pose_label_;
  QDoubleSpinBox * forbidden_zone_x_spin_;
  QDoubleSpinBox * forbidden_zone_y_spin_;
  QDoubleSpinBox * forbidden_zone_z_spin_;
  QDoubleSpinBox * forbidden_zone_roll_spin_;
  QDoubleSpinBox * forbidden_zone_pitch_spin_;
  QDoubleSpinBox * forbidden_zone_yaw_spin_;
  QWidget * forbidden_zone_mesh_row_widget_;
  QLabel * forbidden_zone_mesh_path_label_;
  QPushButton * forbidden_zone_mesh_button_;
  QLabel * forbidden_zone_mesh_scale_label_;
  QDoubleSpinBox * forbidden_zone_mesh_scale_spin_;
  QPushButton * forbidden_zone_apply_button_;
  QComboBox * forbidden_zone_user_combo_;
  QPushButton * forbidden_zone_remove_button_;
  QLineEdit * forbidden_zone_group_name_edit_;
  QListWidget * forbidden_zone_group_members_list_;
  QComboBox * forbidden_zone_group_combo_;
  QPushButton * forbidden_zone_group_save_button_;
  QPushButton * forbidden_zone_group_remove_button_;
  QGroupBox * teach_box_;
  QLabel * warning_label_;
  QLabel * speed_label_;
  QLabel * status_label_;
  QDoubleSpinBox * speed_spin_;
  QPushButton * home_button_;
  QGroupBox * singularity_escape_box_;
  QLabel * singularity_escape_hint_label_;
  std::array<QDoubleSpinBox *, 6> singularity_escape_joint_spins_{};
  QPushButton * singularity_escape_reset_button_;
  QPushButton * singularity_escape_button_;
  QPushButton * sync_button_;
  QPushButton * pick_place_button_;
  QPushButton * stop_button_;
  QPushButton * xbox_mode_button_;
  QPushButton * camera_button_;
  QLabel * camera_tab_status_label_;
  QLabel * camera_stream_info_label_;
  QLabel * camera_view_label_;
  QPushButton * camera_tab_button_;
  QCheckBox * camera_color_checkbox_;
  QCheckBox * camera_depth_checkbox_;
  QCheckBox * camera_ir_checkbox_;
  QLabel * camera_topic_label_;
  QComboBox * camera_topic_combo_;
  QPushButton * camera_topic_refresh_button_;
  QLabel * hand_vision_status_label_;
  QLabel * pulse_region_label_;
  QLabel * hand_vision_view_label_;
  QPushButton * hand_vision_button_;
  QPushButton * hand_vision_large_button_;
  QDialog * hand_vision_large_dialog_{nullptr};
  QLabel * hand_vision_large_label_{nullptr};
  QPushButton * pulse_approach_button_;
  QDoubleSpinBox * pulse_standoff_spin_;
  QLabel * hand_vision_topic_label_;
  QComboBox * hand_vision_topic_combo_;
  QPushButton * hand_vision_topic_refresh_button_;
  QLabel * handeye_status_label_;
  QPushButton * handeye_button_;
  QComboBox * handeye_backend_combo_;
  QComboBox * handeye_mode_combo_;
  QComboBox * handeye_target_combo_;
  QLineEdit * handeye_name_edit_;
  QLineEdit * handeye_robot_base_edit_;
  QLineEdit * handeye_robot_effector_edit_;
  QLineEdit * handeye_tracking_base_edit_;
  QLineEdit * handeye_tracking_marker_edit_;
  QComboBox * handeye_tag_family_combo_;
  QSpinBox * handeye_tag_id_spin_;
  QDoubleSpinBox * handeye_tag_size_spin_;
  QPointer<rviz_common::Display> moveit_calibration_display_;
  QPushButton * xbox_help_button_;
  QWidget * xbox_help_container_;
  QTableWidget * xbox_help_table_;
  QComboBox * teach_action_combo_;
  QTabWidget * teach_action_mode_tabs_;
  QGroupBox * teach_shape_box_;
  QWidget * teach_recording_widget_;
  QLabel * teach_shape_edit_status_label_;
  QPushButton * teach_shape_new_button_;
  QComboBox * teach_shape_combo_;
  QLineEdit * teach_shape_source_edit_;
  QPushButton * teach_shape_image_button_;
  QLabel * teach_shape_source_label_;
  QPushButton * teach_shape_create_button_;
  QPushButton * teach_shape_reachability_button_;
  QLabel * teach_shape_reachability_label_;
  QPushButton * teach_shape_marker_button_;
  QPushButton * teach_shape_reset_button_;
  QPushButton * teach_shape_apply_pose_button_;
  QLabel * teach_shape_selected_label_;
  QLabel * teach_shape_preview_label_;
  QLabel * teach_shape_position_label_;
  QLabel * teach_shape_orientation_label_;
  QLabel * teach_shape_size_label_;
  QLabel * teach_shape_pen_label_;
  QDoubleSpinBox * teach_shape_x_spin_;
  QDoubleSpinBox * teach_shape_y_spin_;
  QDoubleSpinBox * teach_shape_z_spin_;
  QDoubleSpinBox * teach_shape_roll_spin_;
  QDoubleSpinBox * teach_shape_pitch_spin_;
  QDoubleSpinBox * teach_shape_yaw_spin_;
  QDoubleSpinBox * teach_shape_width_spin_;
  QDoubleSpinBox * teach_shape_height_spin_;
  QDoubleSpinBox * teach_shape_pen_length_spin_;
  QDoubleSpinBox * teach_shape_pen_lift_spin_;
  QLabel * teach_shape_marker_hint_label_;
  QString teach_shape_image_path_;
  QString editing_shape_action_name_;
  QLabel * teach_trace_legend_label_;
  QLabel * teach_trace_width_label_;
  QDoubleSpinBox * teach_trace_width_spin_;
  QTimer * teach_trace_width_timer_;
  QPushButton * teach_refresh_button_;
  QPushButton * teach_copy_button_;
  QPushButton * teach_rename_button_;
  QPushButton * teach_delete_button_;
  QPushButton * teach_start_button_;
  QPushButton * teach_stop_button_;
  QLabel * teach_replay_speed_label_;
  QDoubleSpinBox * teach_replay_speed_spin_;
  QPushButton * teach_preview_button_;
  QPushButton * teach_replay_button_;
  QPushButton * teach_pause_button_;
  QPushButton * teach_cancel_button_;
  QLabel * teach_hardware_action_progress_label_;
  QProgressBar * teach_hardware_action_progress_bar_;
  QLabel * teach_hardware_overall_progress_label_;
  QProgressBar * teach_hardware_overall_progress_bar_;
  QLabel * teach_slider_label_;
  QSlider * teach_slider_;
  QLabel * teach_progress_label_;
  QLabel * teach_status_label_;
  QGroupBox * teach_sequence_box_;
  QComboBox * teach_sequence_combo_;
  QPushButton * teach_sequence_refresh_button_;
  QListWidget * teach_sequence_list_;
  QPushButton * teach_sequence_add_button_;
  QPushButton * teach_sequence_remove_button_;
  QPushButton * teach_sequence_up_button_;
  QPushButton * teach_sequence_down_button_;
  QPushButton * teach_sequence_save_button_;
  QLabel * teach_sequence_selection_hint_label_;
  QLabel * teach_sequence_replay_speed_label_;
  QDoubleSpinBox * teach_sequence_replay_speed_spin_;
  QPushButton * teach_sequence_play_button_;
  QPushButton * teach_sequence_replay_button_;
  QPushButton * teach_sequence_pause_button_;
  QPushButton * teach_sequence_cancel_button_;
  QLabel * teach_sequence_hardware_action_progress_label_;
  QProgressBar * teach_sequence_hardware_action_progress_bar_;
  QLabel * teach_sequence_hardware_overall_progress_label_;
  QProgressBar * teach_sequence_hardware_overall_progress_bar_;
  QLabel * teach_sequence_slider_label_;
  QSlider * teach_sequence_slider_;
  QLabel * teach_sequence_progress_label_;
  QProcess * process_;
  QProcess * camera_process_;
  QProcess * hand_vision_process_;
  QProcess * handeye_process_;
  QProcess * handeye_publish_process_;
  QTimer * readiness_timer_;
  QImage latest_camera_image_;
  QImage latest_hand_vision_image_;
  QString camera_image_topic_{QStringLiteral("/camera/color/image_raw")};
  QString hand_vision_color_topic_{QStringLiteral("/camera/color/image_raw")};
  bool camera_image_visible_{false};
  bool hand_vision_image_visible_{false};
  bool forbidden_zone_status_seen_{false};
  bool forbidden_zone_request_pending_{false};
  QString forbidden_zone_config_path_;
  QString forbidden_zone_user_config_path_;
  QString forbidden_zone_mesh_path_;
  QStringList forbidden_zone_loaded_areas_;
  QStringList forbidden_zone_defined_areas_;
  QStringList forbidden_zone_enabled_areas_;
  QStringList forbidden_zone_active_groups_;
  QStringList forbidden_zone_defined_groups_;
  QStringList forbidden_zone_user_areas_;
  QStringList forbidden_zone_draggable_areas_;
  QStringList forbidden_zone_user_groups_;
  QMap<QString, QStringList> forbidden_zone_user_group_members_;
  QString model_{"dm"};
  QString active_executable_;
  bool pulse_dummy_pressure_stopped_{false};
  QString pulse_process_output_tail_;
  QString active_robot_{QStringLiteral("rebotarm")};
  QString status_zh_;
  QString status_en_;
  bool status_error_{false};
  QString teach_status_zh_;
  QString teach_status_en_;
  bool teach_status_error_{false};
  bool shape_pose_dirty_{false};
  bool shape_reachability_available_{false};
  bool shape_reachability_feasible_{false};
  bool shape_pose_syncing_{false};
  bool shape_editor_new_mode_{true};
  bool selected_legacy_shape_action_{false};
  bool initialized_{false};
  bool integrate_motion_planning_{true};
  bool connection_ready_announced_{false};
  bool chinese_{true};
  bool xbox_state_observed_{false};
  bool previous_xbox_armed_{true};
};

}  // namespace rebotarm_demo_rviz

#endif  // REBOTARM_DEMO_RVIZ__DEMO_PANEL_HPP_
