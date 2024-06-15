# Install script for directory: /home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/src/diff_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/build/diff_robot/catkin_generated/installspace/diff_robot.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_robot/cmake" TYPE FILE FILES
    "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/build/diff_robot/catkin_generated/installspace/diff_robotConfig.cmake"
    "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/build/diff_robot/catkin_generated/installspace/diff_robotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_robot" TYPE FILE FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/src/diff_robot/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/diff_robot" TYPE PROGRAM FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/build/diff_robot/catkin_generated/installspace/spawn_car.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/diff_robot" TYPE DIRECTORY FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/src/diff_robot/include/diff_robot/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_robot" TYPE DIRECTORY FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/src/diff_robot/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_robot" TYPE DIRECTORY FILES "/home/leandro/Leandro/Mestrado/Planejamento/Projetos/differentialrobot_ws/src/diff_robot/worlds")
endif()

