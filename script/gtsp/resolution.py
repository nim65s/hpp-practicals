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
from csv import reader, writer

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
            q_guesses = [q_home] + [self.robot.shootRandomConfig() for i in range(20)]
            for handle in part_handles:
                edge = f"driller/drill_tip > {handle} | 0-0_01"
                # generate pregrasp config using q_home and previously computed pre-grasp
                # configurations as initial guess
                for q in q_guesses:
                    res, qpg, err = self.graph.generateTargetConfig(edge, q_home, q)
                    if res:
                        q_guesses.insert(0, qpg)
                        break
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

    def saveRoadmap(self, filename):
        """
        Write configurations and edges in a file
        """
        # number of configurations
        N = self.croadmap.getNbNodes()
        nodes = list()
        with open(filename, "w", newline='') as f:
            w = writer(f)
            f.write(f"Number of nodes: {N}\n")
            # write configurations
            for i in range(N):
                q = self.croadmap.getNode(i)
                nodes.append(q)
                w.writerow(q)
            N = self.croadmap.getNbEdges()
            f.write(f"Number of edges: {N}\n")
            for i in range(N):
                p = self.croadmap.getEdge(i)
                q1, d = self.croadmap.nearestNode(p.initial(), False)
                assert(d < 1e-8)
                q2, d = self.croadmap.nearestNode(p.end(), False)
                assert(d < 1e-8)
                i1 = nodes.index(q1); i2 = nodes.index(q2)
                w.writerow([i1, i2])
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

def readRoadmap(basePlanner, armPlanner, filename, part_handles, home_configs):
    """
    Read a roadmap from a file.
    This function requires an ArmPlanner and a BasePlanner to rebuild arm and base motions
    """
    with open(filename, "r") as f:
        line = "#"
        while line[0] == "#":
            line = f.readline()
        # read number of configurations
        if line[:16] != "Number of nodes:":
            raise RuntimeError(f"Expecting line starting with 'Number of nodes: ', got '{line}'")
        N = int(line[16:])
        nodes = list()
        for i in range(N):
            line = f.readline()
            q = list(map(float, line.split(",")))
            nodes.append(q)
            armPlanner.croadmap.addNode(q)
        line = f.readline()
        if line[:16] != "Number of edges:":
            raise RuntimeError(f"Expecting line starting with 'Number of edges: ', got '{line}'")
        N = int(line[16:])
        for i in range(N):
            line = f.readline()
            e = list(map(int, line.split(",")))
            q1 = nodes[e[0]]
            q2 = nodes[e[1]]
            # Check whether the edge corresponds to a base motion or an arm motion
            if q1[:7] == q2[:7]:
                # arm motion
                p, res, msg = armPlanner.planner.directPath(q1, q2, False)
                p1 = p.asVector()
                p2 = armPlanner.planner.timeParameterization(p1)
            else:
                # base motion
                p, res, msg = basePlanner.planner.directPath(q1, q2, False)
                p1 = p.asVector()
                p2 = basePlanner.planner.timeParameterization(p1)
            armPlanner.croadmap.addNodeAndEdge(q1, q2, p2)
            p.deleteThis(); p1.deleteThis();  p2.deleteThis()
        # retrieve dictionaries from roadmap
        armPlanner.pregraspToGrasp = dict()
        # Pregrasp configurations reaching a handle
        armPlanner.handleToConfigs = dict()
        for handle in part_handles:
            armPlanner.handleToConfigs[handle] = list()
        # Pregrasp configurations with same base pose as home configuration
        armPlanner.homeToConfigs = dict()
        for q_home in home_configs:
            armPlanner.homeToConfigs[tuple(q_home)] = list()
        for q1, q2 in zip(nodes, nodes[1:]):
            for handle in part_handles:
                state = f"driller/drill_tip > {handle} | 0-0_pregrasp"
                res, err = armPlanner.graph.getConfigErrorForNode(state, q1)
                if res:
                    state = f"tiago/gripper grasps driller/handle : driller/drill_tip grasps {handle}"
                    res, err = armPlanner.graph.getConfigErrorForNode(state, q2)
                    assert(res)
                    armPlanner.pregraspToGrasp[tuple(q1)] = q2
                    armPlanner.handleToConfigs[handle].append(q1)
                    for q_home in home_configs:
                        if q_home[:7] == q1[:7]:
                            armPlanner.homeToConfigs[tuple(q_home)].append(q1)
                            break
                    break

                
