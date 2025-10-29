from pyhpp.manipulation import Device, urdf, Graph, Problem, ProgressiveProjector, ManipulationPlanner
from pyhpp.core import ConfigurationShooter, Dichotomy  # noqa: F401
import numpy as np
from pinocchio import SE3, StdVec_Bool as Mask, Quaternion

from pyhpp.gepetto.viewer import Viewer


urdfFilename = (
    "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
)
srdfFilename = (
    "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"
)

urdfFilenameBall = "package://hpp_environments/urdf/ur_benchmark/pokeball.urdf"
srdfFilenameBall = "package://hpp_environments/srdf/ur_benchmark/pokeball.srdf"

r0_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
r1_pose = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))

robot = Device("bot")

urdf.loadModel(robot, 0, "ur5", "anchor", urdfFilename, srdfFilename, r0_pose)
urdf.loadModel(
    robot, 0, "pokeball", "freeflyer", urdfFilenameBall, srdfFilenameBall, r1_pose
)

ballName = "pokeball/root_joint"
robot.setJointBounds(
    ballName,
    [
        -0.4,
        0.4,
        -0.4,
        0.4,
        -0.1,
        1.0,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
    ],
)

urdfFilenameBox = "package://hpp_environments/urdf/ur_benchmark/box.urdf"
srdfFilenameBox = "package://hpp_environments/srdf/ur_benchmark/box.srdf"

urdf.loadModel(
    robot, 0, "box", "anchor", urdfFilenameBox, srdfFilenameBox, r1_pose
)

# vf.moveObstacle("box/base_link_0", [0.3 + 0.04, 0, 0.04, 0, 0, 0, 1])
# vf.moveObstacle("box/base_link_1", [0.3 - 0.04, 0, 0.04, 0, 0, 0, 1])
# vf.moveObstacle("box/base_link_2", [0.3, 0.04, 0.04, 0, 0, 0, 1])
# vf.moveObstacle("box/base_link_3", [0.3, -0.04, 0.04, 0, 0, 0, 1])

problem = Problem(robot)

q1 = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

# Create graph
graph = Graph("graph", robot, problem)
state_placement = graph.createState("placement", False, 0)

problem.pathValidation = Dichotomy(robot.asPinDevice(), 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod(), 0.01
)
graph.initialize()
q1 = np.array(q1)
# Project initial configuration on state 'placement'
res, q_init, error = graph.applyStateConstraints(state_placement, q1)
q2 = q1[::]
q2[7] = 0.2

res, q_goal, error = graph.applyStateConstraints(state_placement, q2)

# Define manipulation planning problem
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
problem.constraintGraph(graph)

manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(5000)
# manipulationPlanner.solve()
#v = Viewer (robot)
# v.playPath (v)
