add_library( udacity SHARED
    robot.cc
    pathplan.cc
    particlefilter.cc
    control.cc
    config.cc
)

target_link_libraries( udacity
    ${THIRD_PARTY_LIBS}
)

add_executable(slam slam.cc)
target_link_libraries( slam udacity )