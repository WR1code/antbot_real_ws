#ifndef REBOTARM_DEMO_RVIZ__MOTION_PLANNING_I18N_HPP_
#define REBOTARM_DEMO_RVIZ__MOTION_PLANNING_I18N_HPP_

class QDockWidget;

namespace rebotarm_demo_rviz
{

// Translate the already-created MoveIt MotionPlanning panel in place.  MoveIt
// Jazzy does not install Qt translation catalogs for this plugin.
bool translateMotionPlanningUi(bool chinese);

// In the integrated reBot workspace, keep only the native pages that add
// functionality not already provided by the safety/action UI.  Passing false
// restores every MoveIt page before returning the widget to its own dock.
bool setMotionPlanningIntegratedMode(bool integrated);

// Locate the dock created by MoveIt's MotionPlanning display.  The reBot
// panel embeds the original widget instead of duplicating MoveIt controls.
QDockWidget * motionPlanningDockWidget();

}  // namespace rebotarm_demo_rviz

#endif  // REBOTARM_DEMO_RVIZ__MOTION_PLANNING_I18N_HPP_
