file(GLOB_RECURSE INC "*.h")
file(GLOB_RECURSE SRC "*.c")

include_directories(${ETHERCAT_INCLUDE_DIR})
include_directories(${XENOMAI_INCLUDE_DIR})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")

set(EXTRA_LIBS
    ${XENO_LDFLAGS}
    ${ECAT_LIBRARY}
    ${BECKHOFF_MODULES_PATH}
    m
)

add_executable(rtController ${INC} ${SRC})
target_link_libraries(rtController ${EXTRA_LIBS})

