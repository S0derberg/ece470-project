import sys
import vrep
import time
import math
import numpy as np
from scipy.linalg import expm, logm

# Run our Baxter Demonstration. Based off the sample code in 'test.py'
# from Professor Bretl.

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

# Convert degrees to radians
def degToRad(angle):
	return angle * math.pi / 180

# Get the bracket of a screw
def bracket(v):
	Bracket = np.zeros((4,4))
	Bracket[0:3,0:3] = skew(v[0:3])
	Bracket[0:3,3] = np.transpose(v[3:])
	return Bracket

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

# Convert a rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :

    x = math.atan2(-R[1,2], R[2,2])
    y = math.sin(R[0,2])
    z = math.atan2(-R[0,1],R[0,0])

    # x = math.atan2(R[1,2], R[2,2])
    # y = math.atan2(-R[0,2], math.sqrt(R[0,0]*R[0,0] + R[0,1]*R[0,1]))
    # z = math.atan2(R[0,1], R[0,0])

    return [x, y, z]
    #return [x, y, z]
    #return [z, x, -y]

    # Below works pretty well as long as the first theta wasn't too big
     
    # sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    # singular = sy < 1e-6
 
    # if  not singular :
    #     x = math.atan2(R[2,1] , R[2,2])
    #     y = math.atan2(-R[2,0], sy)
    #     z = math.atan2(R[1,0], R[0,0])
    # else :
    #     x = math.atan2(-R[1,2], R[1,1])
    #     y = math.atan2(-R[2,0], sy)
    #     z = 0
 
    return np.array([-x, y, z])

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
def moveTorso(clientID):
	print("Moving torso\n")
	# Get "handle" to the torso joint
	result, joint_handle = vrep.simxGetObjectHandle(clientID, "Baxter_rotationJoint", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get object handle for rotation joint")

	testJoint(joint_handle, 0, clientID)

# Move the gripper
def moveGripper1(clientID):
	print("Test Gripper 1\n")
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "BarrettHand_jointB_0", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "BarrettHand_jointB_0", 1, clientID)

def moveGripper2(clientID):
	print("Test Gripper 2\n")
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "JacoHand1_fingers12_motor1", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "JacoHand1_fingers12_motor1", 1, clientID)
	
def moveFrame(clientID, frameID, pose):

	result, frame_handle = vrep.simxGetObjectHandle(clientID, frameID, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get object handle for the Reference Frame object")


	position = pose[0:3,3]
	orientation = pose[0:3,0:3]

	vrep.simxSetObjectPosition(clientID, frame_handle, -1, position, vrep.simx_opmode_oneshot)
	vrep.simxSetObjectOrientation(clientID, frame_handle, -1, rotationMatrixToEulerAngles(orientation), vrep.simx_opmode_oneshot)	

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
		vrep.simxSetJointTargetPosition(clientID, joint_handle, degToRad(thetas[i]), vrep.simx_opmode_oneshot)



# Compute the predicted pose for the tool frame given a set of joint variables.
def forwardKinematics(M, S, thetas):

	product = 1
	for s in range(7):
		product = np.dot(product, expm(bracket(S[:,s])*degToRad(thetas[s])))

	T = np.dot(product, M)
	return T


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

	SLeft = np.zeros((6,7))
	SLeft[:,0] = screw(0, 0, 1, -0.1278, 0.2630, 0)
	SLeft[:,1] = screw(-1, 0, 0, -0.1278, 0.310, 1.3244)
	SLeft[:,2] = screw(0, 1, 0, -0.1278, 0.4140, 1.3244)
	SLeft[:,3] = screw(-1, 0, 0, -0.1278, 0.6765, 1.2554)
	SLeft[:,4] = screw(0, 1, 0, -0.1278, 0.7801, 1.2554)
	SLeft[:,5] = screw(-1, 0, 0, -0.1278, 1.0508, 1.2454)
	SLeft[:,6] = screw(0, 1, 0, -0.1278, 1.1667, 1.2454)

	MRight = np.array([[0, 0, 1, 1.332],
				       [1, 0, 0, -0.12287],
				       [0, 1, 0, 1.2445],
				       [0, 0, 0, 1]])

	SRight = np.zeros((6,7))
	SRight[:,0] = screw(0, 0, 1, 0.2387, -0.1230, 0)
	SRight[:,1] = screw(0, 1, 0, 0.3077, -0.1230, 1.3244)
	SRight[:,2] = screw(1, 0, 0, 0.4097, -0.1230, 1.3244)
	SRight[:,3] = screw(0, 1, 0, 0.6722, -0.1230, 1.2554)
	SRight[:,4] = screw(1, 0, 0, 0.7758, -0.1230, 1.2554)
	SRight[:,5] = screw(0, 1, 0, 1.0465, -0.1230, 1.2454)
	SRight[:,6] = screw(1, 0, 0, 1.1624, -0.1230, 1.2454)

	# To mirror the arms, negate the thetas of joints 1, 3, and 5

	setOne = [45, 10, -30, 20, -40, -30, 100]
	#setTwo = [-45, -30, -50, -10, 10, 10, -10]
	setTwo = [-45, 10, 30, 20, 40, -30, 20]
	#setThree = []

	poseOne = forwardKinematics(MLeft, SLeft, setOne)
	poseTwo = forwardKinematics(MRight, SRight, setTwo)

	moveFrame(clientID, "ReferenceFrame", poseOne)
	moveFrame(clientID, "ReferenceFrame0", poseTwo)

	time.sleep(2)

	moveArm("left", clientID, setOne)
	moveArm("right", clientID, setTwo)

	# Let all animations finish
	time.sleep(5)

	# Stop simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)

	# Close the connection to V-REP
	vrep.simxFinish(clientID)


if __name__=="__main__":
	main(sys.argv)