## aim to do the same procedure as SampleSR1.cpp
## exec(open('sample.py').read())

import cnoid.Util
from cnoid_pyutil import *
from cnoid import IKSolvers
import numpy as np

fname = cnoid.Util.getShareDirectory() + '/model/SR1/SR1.body'
robot = loadRobot(fname)

reset_pose_av = np.array([ 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## rleg
                           0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045, ## rarm
                           0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, ## lleg
                           0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045, ## larm
                           0.0, 0.0, 0.0 ]);

for idx in range(len(reset_pose_av)):
    robot.joint(idx).q = reset_pose_av[idx]

robot.calcForwardKinematics()
robot.calcCenterOfMass()

flushRobotView('SR1')

##// setup tasks
constraints0 = IKSolvers.Constraints()
constraints1 = IKSolvers.Constraints()

##// task: rleg to target
constraint = IKSolvers.PositionConstraint()
constraint.A_link = robot.link('RLEG_ANKLE_R');
constraint.A_localpos = cnoidPosition(translation=np.array([0.0, 0.0, -0.04]))
#constraint.B_link() = nullptr;
constraint.B_localpos = cnoidPosition(translation=np.array([0.0, -0.2, 0.0]))
constraints1.push_back(constraint)

##// task: lleg to target
constraint = IKSolvers.PositionConstraint()
constraint.A_link = robot.link('LLEG_ANKLE_R');
constraint.A_localpos = cnoidPosition(translation=np.array([0.0, 0.0, -0.04]))
#constraint.B_link() = nullptr;
constraint.B_localpos = cnoidPosition(translation=np.array([0.0, 0.2, 0.0]))
constraints1.push_back(constraint)

##// task: COM to target
constraint = IKSolvers.COMConstraint()
constraint.A_robot = robot
constraint.B_localp = np.array([0.0, 0.0, 0.7])
constraints1.push_back(constraint)

constraints2 = IKSolvers.Constraints()

##// task: rarm to target. never reach
constraint = IKSolvers.PositionConstraint()
constraint.A_link = robot.link("RARM_WRIST_R");
constraint.A_localpos = cnoidPosition(translation=np.array([0.0, 0.0, -0.02]))
#constraint.B_link = nullptr;
constraint.B_localpos = cnoidPosition(translation=np.array([0.3, -0.2, 0.8]), rotation=cnoid.Util.angleAxis(-1.5, np.array([0,1,0])))
constraints2.push_back(constraint);

##// task: rarm to target. never reach
constraint = IKSolvers.PositionConstraint()
constraint.A_link = robot.link("LARM_WRIST_R");
constraint.A_localpos = cnoidPosition(translation=np.array([0.0, 0.0, -0.02]))
#constraint.B_link = nullptr;
constraint.B_localpos = cnoidPosition(translation=np.array([0.3, 0.2, 0.8]), rotation=cnoid.Util.angleAxis(-1.5, np.array([0,1,0])))
constraints2.push_back(constraint);

constraints3 = IKSolvers.Constraints()

#// task: joint angle to target
constraint = IKSolvers.JointAngleConstraint()
constraint.joint = robot.link('CHEST')
constraint.targetq = 0.1
constraints3.push_back(constraint);

tasks = IKSolvers.Tasks()
variables = []
variables.append(robot.rootLink)
for idx in range(robot.getNumJoints()):
    variables.append(robot.joint(idx))

constraints = [constraints0, constraints1, constraints2, constraints3 ]
for constl in constraints:
    for const in constl:
        const.debuglevel = 1

loop = IKSolvers.prioritized_solveIKLoop(variables,
                                         constraints,
                                         tasks,
                                         40,
                                         1e-6,
                                         1)

print('loop : {}'.format(loop))

flushRobotView('SR1')

cntr = 0
for constl in constraints:
    for const in constl:
        const.debuglevel = 0
        if const.checkConvergence():
            print('constraint %d (%s) : converged'%(cntr, const))
        else:
            print('constraint %d (%s) : NOT converged'%(cntr, const))
        cntr = cntr + 1
