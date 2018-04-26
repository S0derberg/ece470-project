import sys
import vrep
import time
import math
import numpy as np
from scipy.linalg import expm, logm

# Run our Baxter Demonstration. Based off the sample code in 'test.py'
# from Professor Bretl.

obstacle_names = ["Dummy", "Dummy0"]
obstacle_diam = [0.16, 0.16]

body_names = ["Dummy_body_low"]
body_diam = [0.4]

<<<<<<< HEAD
arm_names = ["Dummy_left_joint1", "Dummy_left_joint2", "Dummy_left_joint4", "Dummy_left_joint6", "Dummy_right_joint1", "Dummy_right_joint2", "Dummy_right_joint4", "Dummy_right_joint6"]
arm_diam = [0.25, 0.3, 0.3, 0.3, 0.25, 0.3, 0.3, 0.3]
=======
arm_names = ["Dummy_left_joint1", "Dummy_left_joint2", "Dummy_left_joint4", "Dummy_left_joint6","Dummy_left_joint7", "Dummy_left_hand", "Dummy_right_joint1", "Dummy_right_joint2", "Dummy_right_joint4", "Dummy_right_joint6", "Dummy_right_joint7", "Dummy_right_hand"]
arm_diam = [0.25, 0.3, 0.3, 0.15, 0.11, 0.10, 0.25, 0.3, 0.3, 0.15, 0.11, 0.10]
>>>>>>> b2028818b660bbb41ae98a28358531c22afc9b27

self_diam = body_diam.copy()
self_diam.extend(arm_diam)

class Node:
	def __init__(self,theta,parent):
		self.theta = theta
		self.parent = parent

# Get the skew symmetric matrix of an array
def skew(arr):
	mat = np.zeros((3,3))
	mat[0][1] = -1 * arr[2]
	mat[0][2] = arr[1]
	mat[1][0] = arr[2]
	mat[1][2] = -1 * arr[0]
	mat[2][0] = -1 * arr[1]
	mat[2][1] = arr[0]
	return mat

# Get the actual values from a skew-symmetric matrix
def unskew(mat):
	return np.array([[mat[2][1]], [mat[0][2]], [mat[1][0]]])

# Convert degrees to radians
def degToRad(angle):
	return angle * math.pi / 180

# Get the bobstacleet of a screw
def bobstacleet(v):
	Bobstacleet = np.zeros((4,4))
	Bobstacleet[0:3,0:3] = skew(v[0:3])
	Bobstacleet[0:3,3] = np.transpose(v[3:])
	return Bobstacleet

# Get the adjoint of a pose matrix
def adjoint(t):
	Ade = np.zeros((6,6))
	Ade[0:3,0:3] = t[0:3,0:3]
	Ade[3:,3:] = t[0:3,0:3]
	Ade[3:,0:3] = np.dot(skew(t[0:3,3]), t[0:3,0:3])
	return Ade

# Get the screw axis from the vector 'a' (a,b,c) that points along
# the axis of rotation and a point 'q' (d,e,f) that lies on the axis.
def screw(a,b,c,d,e,f):
	s = np.zeros(6)
	s[0:3] = np.array([a,b,c])
	s[3:] = -1*np.dot(skew(np.array([a,b,c])), np.array([d,e,f]))
	return s

# Turn a bobstacleeted V into a twist
def twist(v):
	Twist = np.zeros(6)
	Twist[0:3] = np.transpose(unskew(v[0:3,0:3]))
	Twist[3:] = v[0:3,3]
	return Twist

# Compute a space jacobian
def spaceJacobian(S, theta):
	J = np.zeros((6,len(theta)))

	for i in range(len(theta)):
		if i == 0:
			J[:,i] = S[:,i]
		else:
			product = 1
			for j in range(i):
				product = np.dot(product, expm(bobstacleet(S[:,j])*theta[j]))

			J[:,i] = np.dot(adjoint(product), S[:,i])

	return J

# Convert a rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :

    x = math.atan2(-R[1,2],R[2,2])
    y = math.asin(R[0,2])
    z = math.atan2(-R[0,1],R[0,0])
 
    return np.array([x, y, z])


def testJoint(joint_handle, jointID, clientID):

	time.sleep(2)

	# Get the initial value of the joint variable
	result, theta0 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}".format(jointID+1,theta0))

	# Set the desired value of the joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0 + np.pi, vrep.simx_opmode_oneshot)

	time.sleep(2)

	# Get the value of the joint variable after moving it once
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}".format(jointID+1,theta1))

	# Set the desired value of the joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0 - np.pi, vrep.simx_opmode_oneshot)

	time.sleep(4)

	# Get the value of the joint variable after moving it again
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}\n".format(jointID+1,theta2))

	# Set the desired value of the joint variable back to the initial state
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0, vrep.simx_opmode_oneshot)

def testGripper(gripper_handle, gripperID, gripper_Pos, clientID):

	time.sleep(1)

	# Close the gripper
	# vrep.simxSetJointTargetPosition(clientID, gripper_handle, -gripper_Pos, vrep.simx_opmode_oneshot)
	# time.sleep(1)

	result, theta = vrep.simxGetJointPosition(clientID, gripper_handle, vrep.simx_opmode_blocking)
	print("Theta is {}".format(theta))

	# Open the gripper
	vrep.simxSetJointTargetPosition(clientID, gripper_handle, theta - np.pi, vrep.simx_opmode_oneshot)
	time.sleep(1)

	result, theta = vrep.simxGetJointPosition(clientID, gripper_handle, vrep.simx_opmode_blocking)
	print("Theta is {}".format(theta))

	# Hold the gripper
	vrep.simxSetJointTargetPosition(clientID, gripper_handle, 0, vrep.simx_opmode_oneshot)
	time.sleep(1)

# Move all the joints of an arm
def testArm(arm, clientID):
	armID = "Baxter_" + arm + "Arm_joint"
	print("Moving {} arm.\n".format(arm))

	for i in range(7):
		print("Joint: {}".format(i+1))
		# Get "handle" to the a joint of the robot
		result, joint_handle = vrep.simxGetObjectHandle(clientID, armID + str(i+1), vrep.simx_opmode_blocking)
		if result != vrep.simx_return_ok:
		    raise Exception("Could not get object handle for {} arm joint {}".format(arm, i+1))

		testJoint(joint_handle, i, clientID)


# Rotate the torso
def moveTorso(clientID,theta,test=False):
	print("Moving torso\n")
	# Get "handle" to the torso joint
	result, joint_handle = vrep.simxGetObjectHandle(clientID, "Baxter_rotationJoint", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get object handle for rotation joint")
	if test:
		testJoint(joint_handle, 0, clientID)
	else:
		vrep.simxSetJointTargetPosition(clientID, joint_handle, theta, vrep.simx_opmode_oneshot)

# Move the gripper
def moveLeftGripper(clientID):
	print("Test Gripper 1\n")
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "JacoHand_fingers12_motor1", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "JacoHand_fingers12_motor1", 1, clientID)

def moveRightGripper(clientID):
	print("Test Gripper 2\n")
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "JacoHand_fingers12_motor1#0", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "JacoHand_fingers12_motor1#0", 1, clientID)
	
# Move the dummy reference frame to the given pose.
def moveFrame(clientID, frameID, pose):

	result, frame_handle = vrep.simxGetObjectHandle(clientID, frameID, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get object handle for the Reference Frame object")


	position = pose[0:3,3]
	orientation = pose[0:3,0:3]

	vrep.simxSetObjectPosition(clientID, frame_handle, -1, position, vrep.simx_opmode_oneshot)
	vrep.simxSetObjectOrientation(clientID, frame_handle, frame_handle, rotationMatrixToEulerAngles(orientation), vrep.simx_opmode_oneshot)
	#vrep.simxSetObjectQuaternion(clientID, frame_handle, -1, rotationMatrixToQuaternion(orientation), vrep.simx_opmode_oneshot)

# Move an arm to a set of joint variables
def moveArm(arm, clientID, thetas):
	armID = "Baxter_" + arm + "Arm_joint"
	print("Moving {} arm.\n".format(arm))

	for i in range(7):
		# Get "handle" to the a joint of the robot
		result, joint_handle = vrep.simxGetObjectHandle(clientID, armID + str(i+1), vrep.simx_opmode_blocking)
		if result != vrep.simx_return_ok:
		    raise Exception("Could not get object handle for {} arm joint {}".format(arm, i+1))

		# Set the desired value of the joint variable
		vrep.simxSetJointTargetPosition(clientID, joint_handle, thetas[i], vrep.simx_opmode_oneshot)



# Compute the predicted pose for the tool frame given a set of joint variables.
def forwardKinematics(M, S, thetas):

	product = 1
	for s in range(len(thetas)):
		product = np.dot(product, expm(bobstacleet(S[:,s])*thetas[s]))

	T = np.dot(product, M)
	return T

# Find a set of joint variables to reach the goal pose
def inverseKinematics(goal, M, S):
	theta = np.random.rand(S.shape[1])
	# theta = np.zeros(S.shape[1])
	V_error = 5
	theta_error = 5
	tries = 0
	while V_error > 0.1 or theta_error > 0.01:
		if tries > 100:
			return theta, False

		T = forwardKinematics(M, S, theta)
		V_bobstacleet = logm(np.dot(goal, np.linalg.inv(T)))
		V = twist(V_bobstacleet)
		J = spaceJacobian(S, theta)

		# Theta = Theta + [ (JT * J + 0.00001*I)^-1 * (JT * V) ] - [ (I - J#J) * Theta ]
		theta_dot = np.dot(np.linalg.inv(np.dot(np.transpose(J), J) + 0.1*np.identity(8)), np.dot(np.transpose(J), V)) - np.dot(np.identity(8) - np.dot(np.linalg.pinv(J), J), theta)
		theta = theta + theta_dot
		V_error = np.linalg.norm(V)
		theta_error =  np.linalg.norm(theta_dot)
		print("V Error: {}, Theta Error: {}".format(V_error, theta_error))
		tries += 1

	return theta, True

# Check if a set of thetas found from inverse kinematics are possible for Baxter
def checkThetas(thetas, arm):
	#print(thetas)
	if thetas[0] < -2.967 or thetas[0] > 2.967:
		return False	
	if thetas[1] < -1.7016 or thetas[1] > 1.7016:
		return False
	if thetas[2] < -2.147 or thetas[2] > 1.047:
		return False
	if thetas[3] < -3.0541 or thetas[3] > 3.0541:
		return False
	if thetas[4] < -0.05 or thetas[4] > 2.618:
		return False
	if thetas[5] < -3.059 or thetas[5] > 3.059:
		return False
	if thetas[6] < -1.5707 or thetas[6] > 2.094:
		return False
	if thetas[7] < -3.059 or thetas[7] > 3.059:
		return False
	return True


# Move the dummy frames to a pose calculated with Forward Kinematics and then
# move the arms to the provided joint variables
def moveArmsAndFrames(clientID, MLeft, SLeft, MRight, SRight, thetas):

	setOne = thetas
	setTwo = [-1*thetas[0], thetas[1], -1*thetas[2], thetas[3], -1*thetas[4], thetas[5], thetas[6]]

	poseOne = forwardKinematics(MLeft, SLeft, setOne)
	poseTwo = forwardKinematics(MRight, SRight, setTwo)

	moveArm("left", clientID, setOne)
	moveArm("right", clientID, setTwo)



# move the arm using inverse kinematics
def moveArmAndFrame(clientID, M, S, SLeft, SRight, arm_centers, body_centers, obstacle_centers, pose, arm, frame):

	# Get handles for the frames so we can move them back to the origin when we're done with them
	result, frame_handle = vrep.simxGetObjectHandle(clientID, frame, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get object handle for the Reference Frame object")


	moveFrame(clientID, frame, pose)

	time.sleep(2)

	validThetas = False
	tries = 0
	while not validThetas:
		if tries > 10:
			break

		goal_thetas, result = inverseKinematics(pose, M, S)

		if not result:
			break

		print("++++++++++++++++++++++++++++++++++++++")
		validThetas = checkThetas(goal_thetas, arm)
		tries += 1

	if not validThetas:
		# Couldn't find a valid set of thetas to reach the goal pose
		moveTorso(clientID, 0, test=True)
	else:

		# FIND PATH (LIST OF THETAS), ASSUMING THE START THETAS ARE ALL ZERO (RADIANS) AND USING THE GOAL THETAS FROM INVERSE KINEMATICS
		path = findPath(clientID, np.array([0,0,0,0,0,0,0,0]), goal_thetas, arm_centers, body_centers, obstacle_centers, SLeft, SRight, arm)
		print(path)
		
		if not path:
			moveTorso(clientID, 0, test=True)
		else:
			for theta in path:

				moveTorso(clientID, theta[0])
				moveArm(arm, clientID, theta[1:])
				time.sleep(2)
		# moveTorso(clientID, goal_thetas[0])
		# moveArm(arm, clientID, goal_thetas[1:])

	time.sleep(3)

	vrep.simxSetObjectPosition(clientID, frame_handle, -1, [0,0,0], vrep.simx_opmode_oneshot)


def poseFromTranslationAndRotation(x, y, z, alpha, beta, gamma):
	pose = np.zeros((4,4))
	pose[0,3] = x
	pose[1,3] = y
	pose[2,3] = z
	pose[3,3] = 1

	x_rot = np.array([[1, 0, 0],
					  [0, math.cos(degToRad(alpha)), -1*math.sin(degToRad(alpha))],
					  [0, math.sin(degToRad(alpha)), math.cos(degToRad(alpha))]])

	y_rot = np.array([[math.cos(degToRad(beta)), 0, math.sin(degToRad(beta))],
					  [0, 1, 0],
					  [-1*math.sin(degToRad(beta)), 0, math.cos(degToRad(beta))]])

	z_rot = np.array([[math.cos(degToRad(gamma)), -1*math.sin(degToRad(gamma)), 0],
					  [math.sin(degToRad(gamma)), math.cos(degToRad(gamma)), 0],
					  [0, 0, 1]])

	rotation = np.dot(np.dot(x_rot, y_rot), z_rot)
	pose[0:3,0:3] = rotation
	return pose


# Update sphere centers using forward kinematics
def updateCenters(clientID, centers, SLeft, SRight, thetas, FK=True):
	new_centers = []

	if FK:
		left_thetas = thetas[:8]
		#right_thetas = [thetas[0], -1*thetas[1], thetas[2], -1*thetas[3], thetas[4], -1*thetas[5], thetas[6], thetas[7]]
		right_thetas = np.block([thetas[0], thetas[8:]])

		joints_to_add = [0,1,3,5,5,7]
		for i in range(6):
			old_position = np.block([centers[i], 1])
			new_position = forwardKinematics(old_position, SLeft[:,:joints_to_add[i]+1], left_thetas[:joints_to_add[i]+1])
			new_centers.append(new_position[0:3])

		for j in range(6):
			old_position = np.block([centers[j+6], 1])
			new_position = forwardKinematics(old_position, SRight[:,:joints_to_add[j]+1], right_thetas[:joints_to_add[j]+1])
			new_centers.append(new_position[0:3])

	else:
		new_centers = []
		for h in range(len(arm_names)):
			result, dummy_handle = vrep.simxGetObjectHandle(clientID, arm_names[h], vrep.simx_opmode_blocking)
			if result != vrep.simx_return_ok:
				raise Exception("Could not get object handle for the Dummy object")

			status, position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
			new_centers.append(np.array(position))
	return new_centers


# Check for collision
def checkCollision(arm_centers, body_centers, obstacle_centers):

	# Check for obstacle collision
	for a in range(len(arm_names)):
		center = arm_centers[a]

		for r in range(len(obstacle_names)):
			obstacle = obstacle_centers[r]

			if np.linalg.norm(center - obstacle) < arm_diam[a]/2 + obstacle_diam[r]/2:
				return True

	#Check for self-collision
	self_centers = body_centers.copy()
	self_centers.extend(arm_centers)
	total = len(arm_names) + len(body_names)
	for i in range(total):
		for j in range(i+1, total):
			if np.linalg.norm(self_centers[i] - self_centers[j]) < self_diam[i]/2 + self_diam[j]/2:
				return True

	return False


# Notify of collision with a colored dummy
def notifyCollision(clientID, collision, offset):

	result, handle = vrep.simxCreateDummy(clientID, 0.3, None, vrep.simx_opmode_blocking)

	vrep.simxSetObjectPosition(clientID, handle, -1, [1.5, -2.5+(0.35*offset), 0], vrep.simx_opmode_oneshot)

	return handle

# Clear the dummies
def clearNotifications(clientID, dummies):
	for handle in dummies:
		vrep.simxRemoveObject(clientID, handle, vrep.simx_opmode_oneshot)


# Check for collisions along a straight line from one theta to another.
def checkStraightLine(clientID, theta_a, theta_b, arm_centers, body_centers, obstacle_centers, SLeft, SRight, arm):
	dtheta = 0.01
	s = 0
	while s <= 1:
		theta = (1-s)*theta_a + s*theta_b
		if arm == "left":
			theta = np.block([theta,0,0,0,0,0,0,0])
			new_arm_centers = updateCenters(clientID, arm_centers, SLeft, SRight, theta)
		else:
			theta = np.block([theta[0],0,0,0,0,0,0,0,theta[1:]])
			new_arm_centers = updateCenters(clientID, arm_centers, SLeft, SRight, theta)
		collision = checkCollision(new_arm_centers, body_centers, obstacle_centers)
		if collision:
			return True
		s = s + dtheta
	return False


# Find a path from the start thetas to the goal thetas by using straight line segments.
def findPath(clientID, start, goal, arm_centers, body_centers, obstacle_centers, SLeft, SRight, arm):
	theta_start = Node(start,None)
	theta_goal = Node(goal,None)

	forward = [theta_start]
	backward = [theta_goal]

	n = 0
	while n <= 50:
		print(n)
		theta = np.zeros(8)
		# theta[0] = (2.967 - (-2.967)) *np.random.random_sample() + (-2.967)
		theta[0] = (1 - (-1)) *np.random.random_sample() + (-1)
		theta[1] = (1.7016 - (-1.7016)) *np.random.random_sample() + (-1.7016)
		theta[2] = (1.047 - (-2.147)) *np.random.random_sample() + (-2.147)
		theta[3] = (3.0541 - (-3.0541)) *np.random.random_sample() + (-3.0541)
		theta[4] = (2.618 - (-0.05)) *np.random.random_sample() + (-0.05)
		theta[5] = (3.059 - (-3.059)) *np.random.random_sample() + (-3.059)
		theta[6] = (2.094 - (-1.5707)) *np.random.random_sample() + (-1.5707)
		theta[7] = (3.059 - (-3.059)) *np.random.random_sample() + (-3.059)

		LeastDistanceSoFar = 9999999999
		addedToforward = False
		addedTobackward = False

		for Tforward in forward:
			if np.linalg.norm(Tforward.theta-theta) < LeastDistanceSoFar:
				ClosestNode = Tforward
				LeastDistanceSoFar = np.linalg.norm(Tforward.theta-theta)

		collision = checkStraightLine(clientID, ClosestNode.theta, theta, arm_centers, body_centers, obstacle_centers, SLeft, SRight, arm)
		if not collision:
			theta_new = Node(theta,ClosestNode)
			forward.append(theta_new)
			addedToforward = True

		LeastDistanceSoFar = 9999999999
		for Tbackward in backward:
			if np.linalg.norm(Tbackward.theta-theta) < LeastDistanceSoFar:
				ClosestNode = Tbackward
				LeastDistanceSoFar = np.linalg.norm(Tbackward.theta-theta)


		collision = checkStraightLine(clientID, theta, ClosestNode.theta, arm_centers, body_centers, obstacle_centers, SLeft, SRight, arm)
		if not collision:
			theta_new_backward = Node(theta,ClosestNode)
			backward.append(theta_new_backward)
			addedTobackward = True


		if addedToforward and addedTobackward:
			path = [theta_new.theta]
			parent = theta_new.parent
			while parent != None:
				path = [parent.theta] + path
				parent = parent.parent
			
			parent = theta_new_backward.parent
			while parent != None:
				path = path + [parent.theta]
				parent = parent.parent

			return path

		n += 1

	return False


# Connect to V-Rep and start the simulation
def main(args):

	# Close all open connections (just in case)
	vrep.simxFinish(-1)

	# Connect to V-REP (raise exception on failure)
	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	if clientID == -1:
	    raise Exception('Failed connecting to remote API server')

	# Start simulation
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

	MLeft = np.array([[-1, 0, 0, -0.1278],
				  	  [0, 0, 1, 1.3363],
				      [0, 1, 0, 1.2445],
				      [0, 0, 0, 1]])

	SLeft = np.zeros((6,8))
	SLeft[:,0] = screw(0, 0, 1, 0.0103, 0.0147, 0)
	SLeft[:,1] = screw(0, 0, 1, -0.1278, 0.2630, 0)
	SLeft[:,2] = screw(-1, 0, 0, -0.1278, 0.310, 1.3244)
	SLeft[:,3] = screw(0, 1, 0, -0.1278, 0.4140, 1.3244)
	SLeft[:,4] = screw(-1, 0, 0, -0.1278, 0.6765, 1.2554)
	SLeft[:,5] = screw(0, 1, 0, -0.1278, 0.7801, 1.2554)
	SLeft[:,6] = screw(-1, 0, 0, -0.1278, 1.0508, 1.2454)
	SLeft[:,7] = screw(0, 1, 0, -0.1278, 1.1667, 1.2454)

	MRight = np.array([[0, 0, 1, 1.332],
				       [1, 0, 0, -0.12287],
				       [0, 1, 0, 1.2445],
				       [0, 0, 0, 1]])

	SRight = np.zeros((6,8))
	SRight[:,0] = screw(0, 0, 1, 0.0103, 0.0147, 0)
	SRight[:,1] = screw(0, 0, 1, 0.2387, -0.1230, 0)
	SRight[:,2] = screw(0, 1, 0, 0.3077, -0.1230, 1.3244)
	SRight[:,3] = screw(1, 0, 0, 0.4097, -0.1230, 1.3244)
	SRight[:,4] = screw(0, 1, 0, 0.6722, -0.1230, 1.2554)
	SRight[:,5] = screw(1, 0, 0, 0.7758, -0.1230, 1.2554)
	SRight[:,6] = screw(0, 1, 0, 1.0465, -0.1230, 1.2454)
	SRight[:,7] = screw(1, 0, 0, 1.1624, -0.1230, 1.2454)


	obstacle_centers = []
	body_centers = []
	arm_centers = []

	for j in range(len(obstacle_names)):
		result, dummy_handle = vrep.simxGetObjectHandle(clientID, obstacle_names[j], vrep.simx_opmode_blocking)
		if result != vrep.simx_return_ok:
		    raise Exception("Could not get object handle for the Dummy object")

		status, position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
		obstacle_centers.append(np.array(position))

	for k in range(len(body_names)):
		result, dummy_handle = vrep.simxGetObjectHandle(clientID, body_names[k], vrep.simx_opmode_blocking)
		if result != vrep.simx_return_ok:
		    raise Exception("Could not get object handle for the Dummy object")

		status, position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
		body_centers.append(np.array(position))

	for h in range(len(arm_names)):
		result, dummy_handle = vrep.simxGetObjectHandle(clientID, arm_names[h], vrep.simx_opmode_blocking)
		if result != vrep.simx_return_ok:
		    raise Exception("Could not get object handle for the Dummy object")

		status, position = vrep.simxGetObjectPosition(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
		arm_centers.append(np.array(position))

	result, dumbell_handle = vrep.simxGetObjectHandle(clientID, "15lbDumbell0", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception("Could not get object handle for the Dumbell object")

	status, position = vrep.simxGetObjectPosition(clientID, dumbell_handle, -1, vrep.simx_opmode_blocking)
	pose = np.eye(4)
	pose[0:3,3] = np.array(position)+np.array([0,-0.15,0])
	rotation = np.array([[-1,0,0],[0,0,1],[0,1,0]])
	pose[0:3,0:3] = rotation

	result, table_handle = vrep.simxGetObjectHandle(clientID, "customizableTable", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception("Could not get object handle for the Table object")
	
	status, table_position = vrep.simxGetObjectPosition(clientID, table_handle, -1, vrep.simx_opmode_blocking)
	
	dummies = []
	for i in range(13):
		x_offset = np.array([i*0.1-0.6, 0, 0])


		for j in range(9):
			y_offset = np.array([0, j*0.1-0.4, 0])


			result, handle = vrep.simxCreateDummy(clientID, 0.1, [0,0,255], vrep.simx_opmode_blocking)
			dummies.append(handle)
			vrep.simxSetObjectPosition(clientID, handle, -1, np.array(table_position) + x_offset + y_offset, vrep.simx_opmode_oneshot)
			obstacle_centers.append(np.array(table_position) + x_offset + y_offset)
			obstacle_names.append("dummy")
			obstacle_diam.append(0.1)



	arm = "left"

	if arm == "left":
		M = MLeft
		S = SLeft
	elif arm == "right":
		M = MRight
		S = SRight
	else:
		print("Please enter left or right for the arm")
		return

	moveArmAndFrame(clientID, M, S, SLeft, SRight, arm_centers, body_centers, obstacle_centers, pose, arm, "ReferenceFrame0")

	# time.sleep(2)

	# vrep.simxSetIntegerSignal(clientID, "leftGripperClose", 1, vrep.simx_opmode_oneshot)
	# vrep.simxSetIntegerSignal(clientID, "rightGripperClose", 1, vrep.simx_opmode_oneshot)

	# time.sleep(3)

	# # Open the hands
	# # vrep.simxSetIntegerSignal(clientID, "leftGripperClose", 0, vrep.simx_opmode_oneshot)
	# # vrep.simxSetIntegerSignal(clientID, "rightGripperClose", 0, vrep.simx_opmode_oneshot)

	# # time.sleep(4)

	# #clearNotifications(clientID, dummies)

	# result, joint_handle = vrep.simxGetObjectHandle(clientID, "Baxter_rightArm_joint2", vrep.simx_opmode_blocking)
	# if result != vrep.simx_return_ok:
	#     raise Exception("Could not get object handle for {} arm joint {}".format(arm, i+1))

	# # Set the desired value of the joint variable
	# vrep.simxSetJointTargetPosition(clientID, joint_handle, -3, vrep.simx_opmode_oneshot)

	# time.sleep(3)


	# result, joint_handle = vrep.simxGetObjectHandle(clientID, "Baxter_rightArm_joint4", vrep.simx_opmode_blocking)
	# if result != vrep.simx_return_ok:
	#     raise Exception("Could not get object handle for {} arm joint {}".format(arm, i+1))

	# # Set the desired value of the joint variable
	# vrep.simxSetJointTargetPosition(clientID, joint_handle, 3, vrep.simx_opmode_oneshot)

	# time.sleep(3)

	# moveTorso(clientID, 0, test=True)

	time.sleep(3)

	# clearNotifications(clientID,dummies)
	
	# Stop simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)

	# Close the connection to V-REP
	vrep.simxFinish(clientID)


if __name__=="__main__":
	main(sys.argv)