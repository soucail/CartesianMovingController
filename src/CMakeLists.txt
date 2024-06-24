set(controller_SRC
  CartesianMovingController.cpp
)

set(controller_HDR
  CartesianMovingController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DCartesianMovingController_EXPORTS")

