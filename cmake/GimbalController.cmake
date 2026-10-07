# Attach the selected C sources to an existing firmware target. No project(),
# host shared libraries, CTest setup, toolchain flags, or simulation parameters.
include_guard(GLOBAL)
set_property(GLOBAL PROPERTY GIMBAL_CONTROLLER_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

function(gimbal_add_to_target target motor)
    if(NOT TARGET "${target}")
        message(FATAL_ERROR "Create the firmware target before gimbal_add_to_target()")
    endif()
    get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)
    if(NOT "C" IN_LIST languages)
        message(FATAL_ERROR "Enable C in the host project; keep the controller sources as C11")
    endif()
    get_target_property(attached "${target}" GIMBAL_CONTROLLER_MOTOR)
    if(attached)
        message(FATAL_ERROR "Target ${target} already has gimbal sources (${attached})")
    endif()
    if(motor STREQUAL "GM6020")
        set(codec gm6020.c)
    elseif(motor STREQUAL "DM4310")
        set(codec dm_mit.c)
    else()
        message(FATAL_ERROR "Select GM6020 or DM4310 in gimbal_add_to_target()")
    endif()
    get_property(root GLOBAL PROPERTY GIMBAL_CONTROLLER_SOURCE_ROOT)
    target_sources("${target}" PRIVATE
        "${root}/src/yaw_controller.c"
        "${root}/src/gimbal_controller.c"
        "${root}/src/gimbal_coordinates.c"
        "${root}/src/${codec}"
        "${root}/examples/stm32/gimbal_periodic.c")
    target_include_directories("${target}" PRIVATE "${root}/include" "${root}/examples/stm32")
    target_compile_features("${target}" PRIVATE c_std_11)
    target_link_libraries("${target}" PRIVATE m)
    set_property(TARGET "${target}" PROPERTY GIMBAL_CONTROLLER_MOTOR "${motor}")
endfunction()
