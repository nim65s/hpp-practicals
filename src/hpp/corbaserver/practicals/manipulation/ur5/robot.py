#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-universal-robot.
# hpp-universal-robot is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-universal-robot is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-universal-robot.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.manipulation.robot import Robot as Parent

class Robot (Parent):
    packageName = "hpp_practicals"
    urdfName = "ur5_joint_limited_robot"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__ (self, compositeName, robotName, load = True,
                  rootJointType = "anchor"):
        Parent.__init__ (self, compositeName, robotName, rootJointType, load)
        self.rightWrist = "wrist_3_joint"
        self.leftWrist  = "wrist_3_joint"
        self.endEffector = "ee_fixed_joint"

    def getInitialConfig (self):
        q = 6*[0]
        return q
