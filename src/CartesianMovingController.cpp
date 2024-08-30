#include "CartesianMovingController.h"
#include <Eigen/src/Core/Matrix.h>
#include <SpaceVecAlg/PTransform.h>
#include <cmath>
#include <ctime>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/OrientationTask.h>


CartesianMovingController::CartesianMovingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  start_moving_ = false;
  ctlTime_=0;
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  R_=0.25;
  omega_= 2.0;
  init_ = false;
  // jacobian = rbd::Jacobian(robot().mb(), "end_effector_link");
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 5, 1);
  // postureTask->stiffness(3);
  // postureTask->damping(5);
  solver().addTask(postureTask);


  circularTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"), 60.0, 10000);
  Eigen::VectorXd dimweight(6); 
  dimweight << 1., 1., 1., 1., 1., 1. ; 
  circularTask -> dimWeight(dimweight);
  circularTask->reset();

  solver().addTask(circularTask);

  datastore().make<std::string>("ControlMode", "Position"); // entree dans le datastore
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  gui()->addElement(this, {"Control Mode"},
                    mc_rtc::gui::Label("Current Control :", [this]() { return this->datastore().get<std::string>("ControlMode"); }),
                    mc_rtc::gui::Button("Position", [this]() { datastore().assign<std::string>("ControlMode", "Position"); }),
                    mc_rtc::gui::Button("Torque", [this]() { datastore().assign<std::string>("ControlMode", "Torque"); }));

  gui()->addElement({"Tasks"},
    mc_rtc::gui::Checkbox("Circular Moving", this->start_moving_)
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
  ctlTime_ += timeStep;

  if (ctlTime_ > 3) {init_=true; datastore().assign<std::string>("ControlMode", "Torque");}
  // if (ctlTime_ > 3) {init_=true;}

  if (ctlTime_ > 6.00) { 
    circularTask->positionTask->position(Eigen::Vector3d(0.60, 0, 0.25 + R_));
  }
  if (ctlTime_ > 9.4) {start_moving_=true;}
  // if (ctlTime_ > 20) {datastore().assign<std::string>("Coriolis", "no");}
  if (start_moving_ && init_) { 
    circularTask->positionTask->position(Eigen::Vector3d(0.60, R_*std::sin(omega_*ctlTime_), 0.25 + R_*std::cos(omega_*ctlTime_))),
    circularTask->positionTask->refVel(Eigen::Vector3d(0, R_*omega_*std::cos(omega_*ctlTime_), -R_*omega_*std::sin(omega_*ctlTime_))),
    circularTask->positionTask->refAccel(Eigen::Vector3d(0, -R_*R_*omega_*std::sin(omega_*ctlTime_), R_*R_*omega_*std::cos(omega_*ctlTime_))), 
    circularTask->orientationTask->orientation(Eigen::Quaterniond(0, 1, 0, 1).normalized().toRotationMatrix());}

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

CONTROLLER_CONSTRUCTOR("CartesianMovingController", CartesianMovingController)