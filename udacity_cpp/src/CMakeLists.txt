add_library( udacity SHARED
    robot.cc
)

target_link_libraries( udacity
    ${THIRD_PARTY_LIBS}
)