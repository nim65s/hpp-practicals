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

import security_margins
from os import getcwd, path
from CORBA import Any, TC_long, TC_float
from hpp.corbaserver import wrap_delete
from hpp.corbaserver.manipulation import createContext, ProblemSolver, ConstraintGraph, Rule, Constraints, loadServerPlugin
from typing import List, Dict

configType = Dict[str, List[float]]
configListType = List[configType]

class ArmPlanner:

    def wd(self, o):
        return o
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps, graph, robot, croadmap = None):
        """
        \param ps ProblemSolver instance
        \param graph ConstraintGraph instance (c.f. constraints.py)
        \param robot Robot instance (c.f. robot.py)
        """
        self.ps = ps
        self.robot = robot
        self.graph = graph

        self.cproblem = self.wd(ps.hppcorba.problem.getProblem())
        self.cgraph = self.wd(self.cproblem.getConstraintGraph())

        self.crobot = self.wd(self.cproblem.robot())
        self.cdistance = self.wd(ps.hppcorba.problem.createDistance("Weighed",
            self.cproblem))
        if not croadmap:
            self.croadmap = self.wd(ps.client.manipulation.problem.createRoadmap(self.cdistance,
                                                                                 self.crobot))
            self.croadmap.constraintGraph(self.cgraph)
        else:
            self.croadmap = croadmap
        self.planner = self.wd(ps.client.basic.problem.createPathPlanner(
            "TransitionPlanner", self.cproblem, self.croadmap))
        self.planner.setEdge(graph.edges[robot.loop_free])
        self.planner.setParameter("SimpleTimeParameterization/safety",
                                         Any(TC_float, 1.))
        self.planner.setParameter("SimpleTimeParameterization/order",
                                         Any(TC_long, 2))
        self.planner.setParameter("SimpleTimeParameterization/maxAcceleration",
                                         Any(TC_float, 2.0))
        self.planner.maxIterations(600)
        self.planner.timeOut(10.)
        # Set collision margin between mobile base and the rest because the collision model is
        # not correct.
        bodies = ("tiago/torso_fixed_link_0", "tiago/base_link_0")
        cfgVal = self.wd(self.cproblem.getConfigValidations())
        # set security margins between bodies
        for _, la, lb, _, _ in zip(*robot.distancesToCollision()):
            if la in bodies or lb in bodies:
                cfgVal.setSecurityMarginBetweenBodies(la, lb, 0.07)

    def buildRoadmap(self, home_configs, part_handles):
        """
        For each home config and each handle, try to generate pregrasp and graps configs
        with the same base pose. Then try to connect the pregrasps configurations to
        one another.
        """
        # Grasp configuration corresponding to a pre-graps configuration
        self.pregraspToGrasp = dict()
        # Pregrasp configurations reaching a handle
        self.handleToConfigs = dict()
        # Pregrasp configurations with same base pose as home configuration
        self.homeToConfigs = dict()

        for handle in part_handles:
            self.handleToConfigs[handle] = list()
        for q_home in home_configs:
            self.homeToConfigs[tuple(q_home)] = list()
            print("generating pre-grasp and grasp configurations for home configuration:")
            print(q_home)
            pg_configs = list()
            for handle in part_handles:
                edge = f"driller/drill_tip > {handle} | 0-0_01"
                # generate pregrasp config
                res, qpg, err = self.graph.generateTargetConfig(edge, q_home, q_home)
                if not res: continue
                # test collision
                res, msg = self.planner.validateConfiguration(qpg, self.graph.edges[edge])
                if not res: continue
                # generate grasp config
                edge = f"driller/drill_tip > {handle} | 0-0_12"
                self.planner.setEdge(self.graph.edges[edge])
                res, qg, err = self.graph.generateTargetConfig(edge, qpg, qpg)
                if not res: continue
                # test collision
                res, msg = self.planner.validateConfiguration(qg, self.graph.edges[edge])
                if not res: continue
                # build path between pre-grasp and grasp configurations
                p, res, msg = self.planner.directPath(qpg, qg, True)
                if res:
                    p1 = p.asVector()
                    p2 = self.planner.timeParameterization(p1)
                    self.croadmap.addNodeAndEdge(qpg, qg, p2)
                    p3 = p1.reverse()
                    p4 = self.planner.timeParameterization(p3)
                    self.croadmap.addNodeAndEdge(qg, qpg, p4)
                    pg_configs.append(qpg)
                    # Store pregrasps and grasp configurations in dictionaries.
                    self.pregraspToGrasp[tuple(qpg)] = qg
                    self.homeToConfigs[tuple(q_home)].append(qpg)
                    self.handleToConfigs[handle].append(qpg)
                    p1.deleteThis(); p2.deleteThis(); p3.deleteThis(); p4.deleteThis()
                p.deleteThis()
            print(f"{len(pg_configs)} pairs of configurations have been generated.")
            # Try to connect valid pregrasp configurations to one another
            self.planner.setEdge(self.graph.edges[self.robot.loop_free])
            configs = pg_configs; configs.append(q_home)
            for i, q1 in enumerate(configs):
                for q2 in configs[i+1:]:
                    p, res, msg = self.planner.directPath(q1, q2, True);
                    if res:
                        p1 = p.asVector()
                        p2 = self.planner.timeParameterization(p1)
                        self.croadmap.addNodeAndEdge(q1, q2, p2)
                        p3 = p1.reverse()
                        p4 = self.planner.timeParameterization(p3)
                        self.croadmap.addNodeAndEdge(q2, q1, p4)
                        p1.deleteThis(); p2.deleteThis(); p3.deleteThis(); p4.deleteThis()
                    p.deleteThis()



class BasePlanner:

    def wd(self, o):
        return o
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps, graph, robot, croadmap = None):
        """
        \param ps ProblemSolver instance
        \param graph ConstraintGraph instance (c.f. constraints.py)
        \param robot Robot instance (c.f. robot.py)
        """
        self.ps = ps
        self.robot = robot
        self.graph = graph

        self.cproblem = self.wd(ps.hppcorba.problem.getProblem())
        self.crobot = self.wd(self.cproblem.robot())
        self.cgraph = self.wd(self.cproblem.getConstraintGraph())
        self.cdistance = self.wd(ps.hppcorba.problem.createDistance("ReedsShepp",
            self.cproblem))
        if not croadmap:
            self.croadmap = self.wd(ps.client.manipulation.problem.createRoadmap(self.cdistance,
                                                                                 self.crobot))
            self.croadmap.constraintGraph(self.cgraph)
        else:
            self.croadmap = croadmap
        self.croadmap.constraintGraph(self.cgraph)
        self.planner = self.wd(ps.client.basic.problem.createPathPlanner(
            "TransitionPlanner", self.cproblem, self.croadmap))
        # Set parameters
        self.planner.setParameter('RSTimeParameterization/LinearAcceleration', Any(TC_float,.5))
        self.planner.setParameter('RSTimeParameterization/LinearDeceleration', Any(TC_float,.4))
        self.planner.setParameter('RSTimeParameterization/MaxAngularVelocity', Any(TC_float,.1))
        self.planner.setParameter('RSTimeParameterization/MaxLinearVelocity', Any(TC_float,.1))
        self.planner.setParameter('RSTimeParameterization/MinLinearVelocity', Any(TC_float,.05))
        # Set edge before setting Reeds and Shepp steering method since method setEdge puts the
        # steering method of the edge in the inner problem
        self.planner.setEdge(graph.edges["move_base"])

        self.planner.setReedsAndSheppSteeringMethod(.5)
        # set security margins
        smBase = security_margins.SecurityMargins(robot.jointNames)
        margin = 0.1
        for i in [ 0, smBase.jid("part/root_joint") ]:
            smBase.margins[i,:] = margin
            smBase.margins[:,i] = margin
        self.cproblem.setSecurityMargins(smBase.margins.tolist())

    def writeRoadmap(self, filename):
        self.ps.client.manipulation.problem.writeRoadmap\
                       (filename, self.croadmap, self.crobot, self.cgraph)

    def readRoadmap(self, filename):
        self.croadmap = self.ps.client.manipulation.problem.readRoadmap\
                       (filename, self.crobot, self.cgraph)

    def buildRoadmap(self, configs):
        """
        Call the steering method between provided configurations and insert edges
        for each collision free path

        Configurations should be valid
        """
        for i, q1 in enumerate(configs):
            self.croadmap.addNode(q1)
            for q2 in configs[i+1:]:
                p, res, msg = self.planner.directPath(q1, q2, True);
                if res:
                    p1 = p.asVector()
                    p2 = self.planner.timeParameterization(p1)
                    self.croadmap.addNodeAndEdge(q1, q2, p2)
                    p3 = p1.reverse()
                    p4 = self.planner.timeParameterization(p3)
                    self.croadmap.addNodeAndEdge(q2, q1, p4)
                    p1.deleteThis(); p2.deleteThis(); p3.deleteThis(); p4.deleteThis()
                p.deleteThis()

def getMobileBaseRoadmap(basePlanner, roadmapFile):
    """
    \param basePlanner InStatePlanner instance
    \param roadmapFile string of the location of the file with the roadmap
    retrieves a roadmap from roadmapFile
    """
    if path.exists(roadmapFile):
        print("Reading mobile base roadmap", roadmapFile)
        basePlanner.readRoadmap(roadmapFile)
    else:
        print("Building mobile base roadmap")
        try:
            basePlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,300))
            basePlanner.buildRoadmap(q0)
            #sm.margins[:,:] = 0.
            #basePlanner.cproblem.setSecurityMargins(sm.margins.tolist())
        except HppError as e:
            print(e)
        print("Writing mobile base roadmap", roadmapFile)
        basePlanner.writeRoadmap(roadmapFile)

### CONFIGURATION GENERATION

# \goal get a collision-free pregrasp configuration for the given handle
# \param handle name of the handle: should be "part/handle_i" or "part/virtual_i" where i is an
#        integer,
# \param restConfig rest configuration of the robot
# \rval q configuration pregrasping handle
def shootPregraspConfig(handle: str, restConfig: List[float]) -> configType:
    res = False
    tries = 0
    while (not res) and (tries<20):
        try: # get a config
            tries+=1
            if tries%10==0:
                print("attempt ", tries)
            q = robot.shootRandomConfig()
            res, q, err = graph.generateTargetConfig("driller/drill_tip pregrasps "+handle, restConfig, q)
            if (res) and (robot.isConfigValid(q)[0] is False): # check it is collision-free
                # print("config not valid")
                res = False
        except Exception as exc:
            print(exc)
            res = False
            pass
    # if res and tries<=100:
    #     print("config generation successfull")
    return {"name": handle+"_pregrasp", "config": q}

# \goal get multiple collision-free pregrasp configurations for the given handle
# \param handle name of the handle: should be "part/handle_i" or "part/virtual_i" where i is an
#        integer,
# \param restConfig rest configuration of the robot
# \param nbConfigs nb of configurations to be generated
# \param configList list where the generated configurations are to be stored
def shootPregraspConfigs(handle: str, restConfig: List[float], nbConfigs: int, configList: configListType) -> None:
    for i in range(nbConfigs):
        configList.append(shootPregraspConfig(handle, restConfig))
