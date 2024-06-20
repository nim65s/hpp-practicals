# BSD 2-Clause License

# Copyright (c) 2023, Hannes Van Overloop
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from hpp import Transform
import pinocchio

# To view gripper in gepetto - It retrieve the joint and pose information of the gripper within the robot model and
# view the same in the gepetto viewer
def displayGripper(v, name):
    robot = v.robot
    joint, pose = robot.getGripperPositionInJoint(name)
    gname = 'gripper__' + name.replace('/', '_')
    v.client.gui.addXYZaxis(gname, [0, 1, 0, 1], 0.005, 0.015)
    if joint != "universe":
        link = robot.getLinkNames(joint)[0]
        v.client.gui.addToGroup(gname, robot.name + '/' + link)
    else:
        v.client.gui.addToGroup(gname, robot.name)
    v.client.gui.applyConfiguration(gname, pose)

# To view  handles in gepetto - It retrieve the joint and pose information of the handle within the robot model and
# view the same in the gepetto viewer
def displayHandle(v, name):
    robot = v.robot
    joint, pose = robot.getHandlePositionInJoint(name)
    hname = 'handle__' + name.replace('/', '_')
    v.client.gui.addXYZaxis(hname, [0, 1, 0, 1], 0.005, 0.015)
    if joint != "universe":
        link = robot.getLinkNames(joint)[0]
        v.client.gui.addToGroup(hname, robot.name + '/' + link)
    else:
        v.client.gui.addToGroup(hname, robot.name)
    v.client.gui.applyConfiguration(hname, pose)


def fromWorldToPart(coordsInWorld, partPose):
    """
    \param coordsInWorld list of the coordinates in the coord syst of the world 
    \param partPose of type Transform, to change coordinate system from world to piece
    \retval coordsInPart list of the coordinates in the coord syst of the part
    """
    coordsInPart = list()
    for vhw in coordsInWorld:
        coordsInPart.append((partPose.inverse()*Transform(vhw)).toTuple())
    return coordsInPart
