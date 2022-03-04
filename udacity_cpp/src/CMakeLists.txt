add_library( udacity SHARED
    robot.cc
    pathplan.cc
    particlefilter.cc
)

target_link_libraries( udacity
    ${THIRD_PARTY_LIBS}
)