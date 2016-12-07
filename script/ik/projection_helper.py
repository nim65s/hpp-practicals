from numpy import array

__init = False

## Compute the jacobian matrix of the manipulator
#
# \param ps: problem solver
# \param q: configuration for which to compute the jacobian
# \return a python array containing the 6*3 jacobian matrix of the manipulator

def computeJacobian(ps, q):
	global __init
	if not __init:
		#prepare jacobian computation
		ps.createPositionConstraint("pos_cons", 'wrist_3_joint', '', [0,0,0], [0,0,0], [True,True,True])
		ps.setNumericalConstraints("pos_num_cons",["pos_cons"])
		__init = True
	return array(ps.computeValueAndJacobian(q)[1])

