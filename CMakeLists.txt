# Copyright (c) 2017 CNRS
# Author: Florent Lamiraux
#
# this file is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# this file is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# this file.  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp_practicals)
SET(PROJECT_DESCRIPTION "Practicals for Humanoid Path Planner software.")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON(2.7 EXACT REQUIRED)

SET(URDF_UR_BENCHMARK_FILES
  urdf/ur_benchmark/obstacles.urdf
  urdf/ur_benchmark/table.urdf
  urdf/ur_benchmark/wall.urdf
  urdf/ur_benchmark/box.urdf
  urdf/ur_benchmark/ground.urdf
  urdf/ur_benchmark/pokeball.urdf
  )

SET(URDF_UR_FILES
  urdf/ur5_gripper.urdf
  urdf/ur5_joint_limited_robot.urdf
)

SET(SRDF_UR_FILES
  srdf/ur5_gripper.srdf
  srdf/ur5_joint_limited_robot.srdf
)

SET(SRDF_UR_BENCHMARK_FILES
  srdf/ur_benchmark/box.srdf
  srdf/ur_benchmark/ground.srdf
  srdf/ur_benchmark/pokeball.srdf
  )

# Install environment and robot model files
#
INSTALL(FILES ${URDF_UR_BENCHMARK_FILES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/urdf/ur_benchmark)

INSTALL(FILES ${SRDF_UR_BENCHMARK_FILES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/srdf/ur_benchmark)

INSTALL(FILES package.xml
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

INSTALL(FILES ${URDF_UR_FILES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/urdf)

INSTALL(FILES ${SRDF_UR_FILES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/srdf)

INSTALL(DIRECTORY meshes
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

# Install python client files
#
INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/ur5/robot.py
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/ur5/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals/ur5
)

INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals
)

INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals/manipulation
)

INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/ur5/robot.py
  ${PROJECT_SOURCE_DIR}/src/hpp/corbaserver/practicals/manipulation/ur5/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/practicals/manipulation/ur5
)

SETUP_PROJECT_FINALIZE()
