#include "CartesianMovingController.h"
#include <SpaceVecAlg/PTransform.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>

CartesianMovingController::CartesianMovingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  // leftandrightTarget = {{"orientation", {0.71, 0, 0.71, 0}}, {"position", {0.4, 0.4, 1.0}}};


  leftandrightTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"), 3.0, 3);
  // leftandrightTask->reset();
  // leftandrightTask->set_ef_pose(leftandrightTarget);
  leftandrightTask->positionTask->position(Eigen::Vector3d(0.3, 0.3, 0.3));

  solver().addTask(leftandrightTask);

  datastore().make<std::string>("ControlMode", "Torque"); // entree dans le datastore

  gui()->addElement(this, {"Control Mode"},
                    mc_rtc::gui::Label("Current Control :", [this]() { return this->datastore().get<std::string>("ControlMode"); }),
                    mc_rtc::gui::Button("Position", [this]() { datastore().assign<std::string>("ControlMode", "Position"); }),
                    mc_rtc::gui::Button("Torque", [this]() { datastore().assign<std::string>("ControlMode", "Torque"); }));
  
  logger().addLogEntry("ControlMode",
                       [this]()
                       {
                         auto mode = datastore().get<std::string>("ControlMode");
                         if(mode.compare("") == 0) return 0;
                         if(mode.compare("Position") == 0) return 1;
                         if(mode.compare("Velocity") == 0) return 2;
                         if(mode.compare("Torque") == 0) return 3;
                         return 0;
                       });

  mc_rtc::log::success("CartesianMovingController init done");
}

bool CartesianMovingController::run()
{ 
  if(leftandrightTask->eval().norm() < 0.01) { switch_target(); }

  auto ctrl_mode = datastore().get<std::string>("ControlMode");

  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  return false;
}

void CartesianMovingController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void CartesianMovingController::switch_target()
{
  // leftandrightTask->reset();
  if (goingLeft){leftandrightTask->positionTask->position(Eigen::Vector3d(0.3, 0.3, 0.3));}
  else {leftandrightTask->positionTask->position(Eigen::Vector3d(0.2, 0.3, 0.3));}
  goingLeft = !goingLeft;
}

CONTROLLER_CONSTRUCTOR("CartesianMovingController", CartesianMovingController)