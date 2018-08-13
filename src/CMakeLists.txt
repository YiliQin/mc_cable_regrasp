set(controller_SRC
    mc_cable_regrasp_controller.cpp
    mc_cable_regrasp_globalfsm.cpp
    mc_cable_regrasp_basicPrimitive.cpp
    mc_cable_regrasp_primitive1.cpp
    mc_cable_regrasp_primitive1_fsm.cpp
    mc_cable_regrasp_primitive2.cpp
    mc_cable_regrasp_primitive2_fsm.cpp
    mc_cable_regrasp_primitive3.cpp
    mc_cable_regrasp_primitive3_fsm.cpp
    mc_cable_regrasp_primitive4.cpp
    mc_cable_regrasp_primitive4_fsm.cpp
    mc_cable_regrasp_primitive5.cpp
    mc_cable_regrasp_primitive5_fsm.cpp
    mc_cable_regrasp_primitive6.cpp
    mc_cable_regrasp_primitive6_fsm.cpp
    mc_cable_regrasp_primitive11.cpp
    mc_cable_regrasp_primitive11_fsm.cpp
    mc_cable_regrasp_primitive12.cpp
    mc_cable_regrasp_primitive12_fsm.cpp
    mc_cable_regrasp_primitive13.cpp
    mc_cable_regrasp_primitive13_fsm.cpp
    mc_cable_regrasp_primitive14.cpp
    mc_cable_regrasp_primitive14_fsm.cpp
    mc_cable_regrasp_primitive15.cpp
    mc_cable_regrasp_primitive15_fsm.cpp

    mc_cable_regrasp_linear_trajectory.cpp
)

set(controller_HDR
    mc_cable_regrasp_controller.h
    mc_cable_regrasp_globalfsm.h
    mc_cable_regrasp_basicPrimitive.h
    mc_cable_regrasp_primitive1.h
    mc_cable_regrasp_primitive1_fsm.h
    mc_cable_regrasp_primitive2.h
    mc_cable_regrasp_primitive2_fsm.h
    mc_cable_regrasp_primitive3.h
    mc_cable_regrasp_primitive3_fsm.h
    mc_cable_regrasp_primitive4.h
    mc_cable_regrasp_primitive4_fsm.h
    mc_cable_regrasp_primitive5.h
    mc_cable_regrasp_primitive5_fsm.h
    mc_cable_regrasp_primitive6.h
    mc_cable_regrasp_primitive6_fsm.h
    mc_cable_regrasp_primitive11.h
    mc_cable_regrasp_primitive11_fsm.h
    mc_cable_regrasp_primitive12.h
    mc_cable_regrasp_primitive12_fsm.h
    mc_cable_regrasp_primitive13.h
    mc_cable_regrasp_primitive13_fsm.h
    mc_cable_regrasp_primitive14.h
    mc_cable_regrasp_primitive14_fsm.h
    mc_cable_regrasp_primitive15.h
    mc_cable_regrasp_primitive15_fsm.h

    mc_cable_regrasp_linear_trajectory.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)

