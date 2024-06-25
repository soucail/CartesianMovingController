#pragma once

#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include <memory>

#include "api.h"

struct CartesianMovingController_DLLAPI CartesianMovingController : public mc_control::MCController
{
  CartesianMovingController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void switch_target();

  std::shared_ptr<mc_tasks::EndEffectorTask> leftandrightTask;  
  std::shared_ptr<mc_tasks::PostureTask> postureTask;


private:
  mc_rtc::Configuration config_;
  bool goingLeft = true;
  sva::PTransformd leftandrightTarget;
  bool start_moving_;
};

