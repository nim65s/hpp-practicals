from hpp.corbaserver.manipulation.practicals.ur5 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import PathPlayer

Robot.urdfName = "ur5_gripper"
Robot.urdfSuffix = ""
Robot.srdfSuffix = ""

class Pokeball (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  meshPackageName = 'hpp_environments'
  urdfName = 'ur_benchmark/pokeball'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground (object):
  rootJointType = 'anchor'
  packageName = 'hpp_environments'
  urdfName = 'ur_benchmark/ground'
  meshPackageName = 'hpp_environments'
  urdfSuffix = ""
  srdfSuffix = ""

class Box (object):
    rootJointType = 'anchor'
    packageName = 'hpp_environments'
    urdfName = 'ur_benchmark/box'
    meshPackageName = 'hpp_environments'
    urdfSuffix = ""
    srdfSuffix = ""

robot = Robot ('ur5-pokeball', 'ur5')
robot.client.basic.problem.setErrorThreshold (1e-4)
robot.client.basic.problem.setMaxIterations (40)

ps = ProblemSolver (robot)
vf = ViewerFactory (ps)
gripperName = 'ur5/wrist_3_joint'
ballName = 'pokeball/root_joint'
