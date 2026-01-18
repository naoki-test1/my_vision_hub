

set(TARGET voxl-calibrate-px4-horizon)
add_executable(${TARGET} voxl-calibrate-px4-horizon.c)

include_directories( ../include)

find_library(MODAL_JSON modal_json HINTS /usr/lib64)
find_library(MODAL_PIPE modal_pipe HINTS /usr/lib64)

target_link_libraries( ${TARGET}
	${MODAL_JSON}
	${MODAL_PIPE}
)
list(APPEND ALL_TARGETS ${TARGET})

set(TARGET voxl-inspect-vfc)
add_executable(${TARGET} voxl-inspect-vfc.c)
target_link_libraries( ${TARGET}
	${MODAL_JSON}
	${MODAL_PIPE}
)
list(APPEND ALL_TARGETS ${TARGET})


install(
	TARGETS ${ALL_TARGETS}
	LIBRARY			DESTINATION /usr/lib
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)



