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

from math import sqrt
from CORBA import Any, TC_long, TC_float
from hpp.corbaserver import wrap_delete as wd
from hpp.corbaserver.manipulation import createContext, ProblemSolver, ConstraintGraph, Rule, Constraints, loadServerPlugin
from hpp.gepetto.manipulation import ViewerFactory
from hpp_idl.hpp import Error as HppError
from hpp.corbaserver.task_sequencing import Client as SolverClient
import pinocchio
import os, sys, argparse, numpy as np, time

from robot import Robot
from constraints import *
from resolution import *
from display import displayHandle, displayGripper
from gtsp_laas import Algorithm, Gtsp, Tour, readGtspFromFile

# PARSE ARGUMENTS
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
p.add_argument ('--n-random-handles', type=int, default=None,
                help="Generate a random model with N handles.")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)
isSimulation = args.context == "simulation"

# DEFINE CLASSES FOR THE PARTS
class Driller:
    urdfFilename = "package://gerard_bauzil/urdf/driller_with_qr_drill.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/driller.srdf"
    rootJointType = "freeflyer"

class Part:
    urdfFilename = "package://hpp_practicals/urdf/part.urdf"
    srdfFilename = "package://hpp_practicals/srdf/part.srdf"
    rootJointType = "freeflyer"

# THE ROBOT
robot = Robot('./tiago.urdf', args) # CREATE THE ROBOT
robot.setNeutralPosition() # SET ITS NEUTRAL POSITION

# PROBLEM SOLVER
print("Loading model")
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")

# LOAD THE ROBOT INTO THE GUI
vf = ViewerFactory(ps)
vf.loadRobotModel (Driller, "driller")
vf.loadRobotModel (Part, "part")

robot.readSRDF() # LOAD SRDF DATA AND SET SOME JOINT BOUNDS
robot.disableCollisions() # DISABLE COLLISIONS

try:
    v = vf.createViewer()
except:
    print("Did not find viewer")


### SETTING A FEW VARIABLES AND PARAMETERS
print("Setting some parameters")

# SETTING JOINT BOUNDS
robot.defineVariousJointBounds()

# HPP PARAMETERS
ps.selectPathValidation("Graph-Discretized", 0.05)
#ps.selectPathValidation("Graph-Dichotomy", 0)
#ps.selectPathValidation("Graph-Progressive", 0.02)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setParameter("SteeringMethod/Carlike/turningRadius", 0.05)

### GENERATE VIRTUAL HANDLES
# ps.robot.client.manipulation.robot.addGripper
# ps.robot.client.manipulation.robot.addHandle

# GET REAL HANDLES
all_handles = ps.getAvailable('handle')
part_handles = part_handles = list(filter(lambda x: x.startswith("part/"), all_handles))
holeCoords = robot.getHandlesCoords(part_handles)

# STARTING POSITION
robot.setStartingPosition()

### DEFINING CONSTRAINTS
ljs, lock_arm, lock_head, look_at_gripper, tool_gripper = createConstraints(ps, robot)

### BUILDING THE CONSTRAINT GRAPH

# CONSTRAINT GRAPH FACTORY
print("Building graph")
graph = ConsGraphFactory(robot, ps, all_handles, part_handles,
                         ljs, lock_arm, lock_head, look_at_gripper, tool_gripper)
# INITIALIZATION
cproblem = ps.hppcorba.problem.getProblem()
cgraph = cproblem.getConstraintGraph()
graph.initialize()
# a little test
res, robot.q0, err = graph.generateTargetConfig('move_base', robot.q0, robot.q0)
assert(res)
# GRAPH VALIDATION
# print("Validating graph")
# ConsGraphValidation(ps, cgraph)


### PROBLEM RESOLUTION

# INSTATEPLANNER
basePlanner = BasePlanner(ps, graph, robot)
armPlanner = ArmPlanner(ps, graph, robot, part_handles = part_handles,
                        croadmap = basePlanner.croadmap)

### TAKE 8 ARBITRARY BASE POSES AROUND THE PART ###
a = sqrt(2)/2
base_poses = [
    [-1.3,  0.0,   1.0,  0.0,],
    [-0.8, -0.65,    a,    a,],
    [ 0.0, -1.2,   0.0,  1.0,],
    [ 0.8, -0.65,   -a,    a,],
    [ 1.3,  0.0,  -1.0,  0.0,],
    [ 0.8,  0.65,   -a,   -a,],
    [ 0.0,  1.2,   0.0, -1.0,],
    [-0.8,  0.65,    a,   -a,],
]
home_configs = list()
q = robot.q0[:]
for bp in base_poses:
    q[:4] = bp
    res, q1, err = graph.applyNodeConstraints("tiago/gripper grasps driller/handle", q)
    assert(res)
    home_configs.append(q1)

### BUILD ROADMAP WITH ALL PREGRASP AND GRASP CONFIGURATIONS
if True:
    print("Reading roadmap from file")
    readRoadmap(basePlanner, armPlanner, "./data/roadmap", part_handles, home_configs)
else:
    print("Building roadmap")
    basePlanner.buildRoadmap(home_configs)
    armPlanner.buildRoadmap(home_configs)
    armPlanner.saveRoadmap("./data/roadmap")

armPlanner.computeCostMatrix()
armPlanner.writeGtspInFile("./data/tiago.txt")

gtsp = Gtsp(len(armPlanner.nodes), len(armPlanner.clusters), armPlanner.clusters, armPlanner.cost)

### CREATE THE GTSP PROBLEM

### CREATE A PLANNER TO USE THE ROADMAP
cproblem = ps.client.basic.problem.createProblem(armPlanner.crobot)
cplanner = ps.client.basic.problem.createPathPlanner('DiffusingPlanner', cproblem,
                                                     armPlanner.croadmap)
cplanner.maxIterations(0)

### SOLVE GTSP PROBLEM
gtsp = readGtspFromFile("./data/tiago.txt")
tour = Tour(gtsp)
algorithm = Algorithm(gtsp)
algorithm.LKHheur(tour)

# Reorder node ids so that 0 is at the first place
t = list(tour.nodes)
while t[0] != 0:
    # move first element to the end of the list
    t = t[1:] + t[:1]

configs = [armPlanner.nodes[i] for i in t]

# Here we make the asumption that the first configuration is a home configuration,
# the other configurations are pre-grasp configurations.
p = None
for q0, q1 in zip(configs, configs[1:]):
    cproblem.setInitConfig(q0)
    cproblem.resetGoalConfigs()
    cproblem.addGoalConfig(q1)
    if p:
        p1 = cplanner.solve()
        p.concatenate(p1)
        p1.deleteThis()
    else:
        p = cplanner.solve()
    q2 = armPlanner.pregraspToGrasp[tuple(q1)]
    cproblem.setInitConfig(q1)
    cproblem.resetGoalConfigs()
    cproblem.addGoalConfig(q2)
    p1 = cplanner.solve()
    p.concatenate(p1);
    p2 = p1.reverse()
    p.concatenate(p2);
    p1.deleteThis(); p2.deleteThis()

# return to initial configuration
q0 = q1[:]
q1 = configs[0]
cproblem.setInitConfig(q0)
cproblem.resetGoalConfigs()
cproblem.addGoalConfig(q1)
p1 = cplanner.solve()
p.concatenate(p1)
p1.deleteThis()
ps.client.basic.problem.addPath(p)
p.deleteThis()
