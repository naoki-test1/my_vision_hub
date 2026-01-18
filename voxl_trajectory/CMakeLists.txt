
set(TARGET voxl_trajectory)

add_library(${TARGET} INTERFACE)
target_include_directories(${TARGET} INTERFACE include/)

install(
  DIRECTORY include/
  DESTINATION /usr/include/${TARGET}/
  FILES_MATCHING PATTERN "*.h"
)