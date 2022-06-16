cmake_minimum_required(VERSION 3.0.2)
project(rostak)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/rostak_path.py
    scripts/rostak_goto.py
    scripts/rostak_bridge
    scripts/roscot_fix
    src/rostak/roscot_obj.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY secrets DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
