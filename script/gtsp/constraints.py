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

import sys
import hpp_idl
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraphFactory, ConstraintGraph, Rule, Constraints


def lockJoint(robot, ps, jname, cname=None, constantRhs=True):
    """
    \param robot instance of the Robot class (c.f. robot.py)  containing the model of the robot
    \param ps instance of the ProblemSolver class
    \param jname string of the name of the joint to lock
    \param cname string of the name of the constraint corresponding to the locked joint
    \param constantRhs boolean
    \retval cname the constraint for the locked joint
    """
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    try:
        ps.createLockedJoint(cname, jname, robot.q0[s:e])
    except hpp_idl.hpp.Error as exc:
        print(f"failed to lock joint '{jname}'")
        raise exc
    ps.setConstantRightHandSide(cname, constantRhs)
    return cname

def addAlignmentConstrainttoEdge(ps, graph, allHandles, handle, toolGripper):
    """
    \param ps ProblemSolver instance
    \param graph ConsGraphFactory instance
    \param allHandles list of all handles
    \param handle string of the name of the handle to align (of the form "part/handle_i")
    \param toolGripper string of the name of the node of the gripper of the tool
    adds the constraint aligning the tool with the handle
    """
    #recover id of handle
    handleId = allHandles.index(handle)
    J1, gripperPose = ps.robot.getGripperPositionInJoint(toolGripper)
    J2, handlePose = ps.robot.getHandlePositionInJoint(handle)
    T1 = Transform(gripperPose)
    T2 = Transform(handlePose)
    constraintName = handle + '/alignment'
    ps.client.basic.problem.createTransformationConstraint2\
        (constraintName, J1, J2, T1.toTuple(), T2.toTuple(),
         [False, True, True, False, False, False])
    # Set constraint
    edgeName = toolGripper + ' > ' + handle + ' | 0-0_12'
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))
    edgeName = toolGripper + ' < ' + handle + ' | 0-0:1-{}_21'.format(handleId)
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))

def createConstraints(ps, robot):
    """
    \param ps ProblemSolver instance
    \param robot Robot instance (c.f. robot.py)
    \retval ljs list of all joints
    \retval lock_arm constraint locking the arm
    \retval lock_head constraint locking the head
    \retval look_at_gripper constraint forcing the robot to look at the gripper
    \retval toolGripper string of the name of the gripper at the end of the driller in the model
    """
    # joint list
    ljs = list()
    ps.createLockedJoint("tiago_base", "tiago/root_joint", [0,0,1,0])
    lockJoint(robot, ps, "part/root_joint", "lock_part", constantRhs=False)
    # fill joint list
    for n in robot.jointNames:
        if n.startswith('tiago/gripper_') or n.startswith('tiago/hand_'):
            ljs.append(lockJoint(robot, ps, n))
    # ARM
    lock_arm = [ lockJoint(robot, ps, n) for n in robot.jointNames
                 if n.startswith("tiago/arm") or n.startswith('tiago/torso')]
    # HEAD
    lock_head = [ lockJoint(robot, ps, n) for n in robot.jointNames
                  if n.startswith("tiago/head")]
    # "LOOK AT GRIPPER"
    # for (X,Y,Z) the position of the gripper in the camera frame, (X, Y) = 0 and Z >= 0
    toolGripper = "driller/drill_tip"
    ps.createPositionConstraint("look_at_gripper", "tiago/xtion_rgb_optical_frame", toolGripper,
                                (0,0,0), (0,0,0), (True,True,True))
    look_at_gripper = ps.hppcorba.problem.getConstraint("look_at_gripper")
    look_at_gripper.setComparisonType([hpp_idl.hpp.EqualToZero,hpp_idl.hpp.EqualToZero,hpp_idl.hpp.Superior])
    return ljs, lock_arm, lock_head, look_at_gripper, toolGripper

def ConsGraphFactory(robot, ps, allHandles, partHandles,
                     ljs, lock_arm, lock_head, look_at_gripper, toolGripper):
    """
    \param robot Robot instance (c.f. robot.py)
    \param ps ProblemSolver instance
    \param allHandles list of all the handles
    \param partHandles list of the handles on the part
    \param ljs list of all joints
    \param lock_arm constraint locking the arm
    \param lock_head constraint locking the head
    \param look_at_gripper constraint forcing the robot to look at the gripper
    \param toolGripper string of the name of the gripper at the end of the driller in the model
    \retval graph ConstraintGraph instance i.e. the constraint graph
    """
    graph = ConstraintGraph(robot, 'graph')
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([ "tiago/gripper", "driller/drill_tip", ])
    # OBJECTS
    factory.setObjects([ "driller", "part", ], [ [ "driller/handle", ], partHandles, ], [ [], [] ])
    # RULES
    factory.setRules([
        # Forbid driller to grasp itself.
    Rule([ "driller/drill_tip", ], [ "driller/handle", ], False),
        # Tiago always holds the gripper.
    Rule([ "tiago/gripper", ], [ "", ], False), Rule([ "tiago/gripper", ], [ "part/.*", ], False), 
        # Allow to associate drill_tip with part holes only.
    Rule([ "tiago/gripper", "driller/drill_tip", ], [ "driller/handle", ".*", ], True), ])
    factory.generate()
    # FREE NODE
    graph.addConstraints(graph=True, constraints=Constraints(numConstraints=ljs))
    # other nodes
    for n in graph.nodes.keys():
        if n == robot.free: continue
        graph.addConstraints(node=n,
                             constraints=Constraints(numConstraints=["look_at_gripper"]))
    for e in graph.edges.keys():
        graph.addConstraints(edge=e,
                             constraints=Constraints(numConstraints=["tiago_base"]))
    for handle in partHandles: # alignment
        addAlignmentConstrainttoEdge(ps, graph, allHandles, handle, toolGripper)
        # extra edge for configuration generation
        graph.createEdge(nodeFrom="tiago/gripper grasps driller/handle",
                         nodeTo="driller/drill_tip > " + handle + " | 0-0_pregrasp",
                         name="driller/drill_tip pregrasps "+handle,
                         weight=-1,
                         isInNode="tiago/gripper grasps driller/handle") # create the edge
        graph.addConstraints(edge="driller/drill_tip pregrasps "+handle,
                             constraints=Constraints(numConstraints=["part/root_joint"])) # fix the table
    # HOME NODE
    graph.createNode('home', priority=1000)
    graph.createEdge('home', 'home', 'move_base')
    graph.createEdge('home',  robot.free , 'start_arm', isInNode="home")
    graph.createEdge( robot.free , 'home', 'end_arm', isInNode=robot.free)
    graph.addConstraints(node="home",
                         constraints=Constraints(numConstraints=lock_arm+lock_head + ['tiago/gripper grasps driller/handle', ]))
    graph.addConstraints(edge="end_arm",
                         constraints=Constraints(numConstraints=["tiago_base", "lock_part"]))
    graph.addConstraints(edge="move_base",
                         constraints=Constraints(numConstraints=["lock_part"]))
    graph.addConstraints(edge="start_arm",
                         constraints=Constraints(numConstraints=['tiago/gripper grasps driller/handle', "lock_part"]))
    return graph

def ConsGraphValidation(ps, cgraph):
    """
    \param ps ProblemSolver instance
    \param cgraph corba client of the constraint graph
    checks if the constraint graph is valid in order to validate it
    """
    graphValidation = ps.client.manipulation.problem.createGraphValidation()
    graphValidation.validate(cgraph)
    if graphValidation.hasErrors():
        print(graphValidation.str())
        print("Graph has infeasibilities")
        sys.exit(1)
    elif graphValidation.hasWarnings():
        print(graphValidation.str())
        print("Graph has only warnings")
    else:
        print("Graph *seems* valid.")

def addExtraHandles(graph, indexOffset, nbNewHandles):
    """
    \param graph ConstraintGraph instance
    \param indexOffset int corresponding to the nb of handles already in the model
    \param nbNewHandles int corresponding to the number of handles to add
    adds nbNewHandles to the constraint graph, with the necessary constraints
    initializes the graph again when the addition is completed
    """
    for i in range(indexOffset, indexOffset+nbNewHandles):
        handle = 'part/virtual_'+str(i)
        nodeName = 'driller/drill_tip > '+handle+' | 0-0_pregrasp'
        edgeName = "driller/drill_tip pregrasps "+handle
        # node
        graph.createPreGrasp("driller/drill_tip pregrasps "+handle,
                             'driller/drill_tip', handle)
        graph.createNode(nodeName)
        graph.addConstraints(node=nodeName,
                             constraints=Constraints(
                                 numConstraints=["driller/drill_tip pregrasps "+handle,
                                                 "tiago/gripper grasps driller/handle"]))
        # edge
        graph.createEdge(nodeFrom="tiago/gripper grasps driller/handle",
                         nodeTo=nodeName,
                         name=edgeName,
                         weight=-1,
                         isInNode="tiago/gripper grasps driller/handle")
        graph.addConstraints(edge=edgeName,
                             constraints=Constraints(numConstraints=["part/root_joint"]))
    graph.initialize()
