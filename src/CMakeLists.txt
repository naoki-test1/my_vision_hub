
file(GLOB all_src_files "*.c" "*.cpp")
add_executable(voxl-vision-hub ${all_src_files})

include_directories(
    ../include
)


## these four are dual-architecture libs, point to the 64-bit one
find_library(MODAL_JSON modal_json HINTS  /usr/lib64)
find_library(MODAL_PIPE modal_pipe HINTS  /usr/lib64)
find_library(RC_MATH     rc_math    HINTS  /usr/lib64)
find_library(VOXL_COMMON_CONFIG voxl_common_config HINTS  /usr/lib64)

## just a 64-bit lib, lives in lib64
## libs in /usr/lib/
find_library(MODALCV    modalcv    HINTS  /usr/lib64)

target_link_libraries(voxl-vision-hub
    pthread
    m
    voxl_trajectory
    ${MODAL_JSON}
    ${MODAL_PIPE}
    ${RC_MATH}
    ${VOXL_COMMON_CONFIG}
    ${MODALCV}
)

set_target_properties(voxl-vision-hub PROPERTIES PUBLIC_HEADER "../include/voxl_vision_hub.h;../include/voxl_vision_px4.h")


install(
    TARGETS voxl-vision-hub
    LIBRARY         DESTINATION /usr/lib
    RUNTIME         DESTINATION /usr/bin
    PUBLIC_HEADER   DESTINATION /usr/include
)
