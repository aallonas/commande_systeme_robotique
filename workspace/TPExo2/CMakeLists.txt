cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(INSAProject)
set(XENO_DIR /usr/xenomai)
set(XENO_INCLUDE_DIR ${XENO_DIR}/include)
set(XENO_BIN_DIR ${XENO_DIR}/bin)
set(XENO_SKIN native)
execute_process(COMMAND ${XENO_BIN_DIR}/xeno-config --skin=${XENO_SKIN}
	--cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${XENO_BIN_DIR}/xeno-config --skin=${XENO_SKIN}
	--ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
include_directories(${XENO_INCLUDE_DIR})
file(GLOB_RECURSE
	src_files
	src/*)
set(EXTRA_LIBS ${XENO_LDFLAGS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
add_executable(my_exe ${src_files})
target_link_libraries(my_exe ${EXTRA_LIBS})
