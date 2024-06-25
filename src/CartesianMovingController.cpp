#include "CartesianMovingController.h"
#include <SpaceVecAlg/PTransform.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/OrientationTask.h>


CartesianMovingController::CartesianMovingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  start_moving_ = false;
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 1, 1);
  postureTask->stiffness(1);
  postureTask->damping(5);
  solver().addTask(postureTask);


  leftandrightTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"), 10.0, 500);
  Eigen::VectorXd dimweight(6); 
  dimweight << 1., 1., 1., 1., 1., 1. ; 
  leftandrightTask -> dimWeight(dimweight);
  leftandrightTask->reset();
  // leftandrightTask->positionTask->position(Eigen::Vector3d(0.4, 0.0, 0.4));

  solver().addTask(leftandrightTask);

  datastore().make<std::string>("ControlMode", "Torque"); // entree dans le datastore
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  gui()->addElement(this, {"Control Mode"},
                    mc_rtc::gui::Label("Current Control :", [this]() { return this->datastore().get<std::string>("ControlMode"); }),
                    mc_rtc::gui::Button("Position", [this]() { datastore().assign<std::string>("ControlMode", "Position"); }),
                    mc_rtc::gui::Button("Torque", [this]() { datastore().assign<std::string>("ControlMode", "Torque"); }));

  gui()->addElement({"Tasks"},
    mc_rtc::gui::Checkbox("Moving from left to right", this->start_moving_)
  );

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
  if(leftandrightTask->eval().norm() < 0.02) { switch_target(); }

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
  if (start_moving_) {
  /// leftandrightTask->reset();
  if (goingLeft){leftandrightTask->positionTask->position(Eigen::Vector3d(0.4, 0.2, 0.4)),leftandrightTask->orientationTask->orientation(Eigen::Quaterniond(1, 4, 1, 4).normalized().toRotationMatrix());
;}
  else {leftandrightTask->positionTask->position(Eigen::Vector3d(0.4, -0.2, 0.4)),leftandrightTask->orientationTask->orientation(Eigen::Quaterniond(-1, 4, 1, 4).normalized().toRotationMatrix());
;}
  goingLeft = !goingLeft;
  }
}

CONTROLLER_CONSTRUCTOR("CartesianMovingController", CartesianMovingController)