import sys
import vrep
import time
import numpy as np

# Run our Baxter Demonstration. Based off the sample code in 'test.py'
# from Professor Bretl.

def testJoint(joint_handle, jointID, clientID):
	# Wait two seconds
	time.sleep(1)

	# Get the initial value of the joint variable
	result, theta0 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}".format(jointID+1,theta0))

	# Set the desired value of the joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0 + np.pi, vrep.simx_opmode_oneshot)

	time.sleep(1)

	# Get the value of the joint variable after moving it once
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}".format(jointID+1,theta1))

	# Set the desired value of the joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0 - np.pi, vrep.simx_opmode_oneshot)

	# Wait two seconds
	time.sleep(1)

	# Get the value of the joint variable after moving it again
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception("Could not get joint variable {}".format(jointID+1))
	print("Current value of joint {} variable: theta = {:f}\n".format(jointID+1,theta2))

	# Set the desired value of the joint variable back to the initial state
	vrep.simxSetJointTargetPosition(clientID, joint_handle, theta0, vrep.simx_opmode_oneshot)

def testGripper(gripper_handle, gripperID, gripper_Pos, clientID):
	# Wait two seconds
	time.sleep(1)

	# Close the gripper
	vrep.simxSetJointTargetPosition(clientID, gripper_handle, -gripper_Pos, vrep.simx_opmode_oneshot)
	time.sleep(1)

	# Open the gripper
	vrep.simxSetJointTargetPosition(clientID, gripper_handle, gripper_Pos, vrep.simx_opmode_oneshot)
	time.sleep(1)

	# Hold the gripper
	vrep.simxSetJointTargetPosition(clientID, gripper_handle, 0, vrep.simx_opmode_oneshot)
	time.sleep(1)

# Move all the joints of an arm
def moveArm(arm, clientID):
	armID = "Baxter_" + arm + "Arm_joint"
	print("Moving {} arm.\n".format(arm))

	for i in range(1):
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
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "JacoHand_fingers12_motor1", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "JacoHand_fingers12_motor1", 1, clientID)


def moveGripper2(clientID):
	result, gripper_handle = vrep.simxGetObjectHandle(clientID, "JacoHand1_fingers12_motor1", vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Execption("Could not get object handle for gripper")

	testGripper(gripper_handle, "JacoHand1_fingers12_motor1", 1, clientID)
	

# Connect to V-Rep and start the simulation
def main(args):

	# Close all open connections (just in case)
	vrep.simxFinish(-1)

	# Connect to V-REP (raise exception on failure)
	clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	if clientID == -1:
	    raise Exception('Failed connecting to remote API server')


	# # Move the joints of the left arm
	moveArm("left", clientID)

	# # # Move the joints of the right arm
	moveArm("right", clientID)

	# # # Rotate the torso
	moveTorso(clientID)

	# Test gripper
	moveGripper1(clientID)
	moveGripper2(clientID)

	# Let all animations finish
	time.sleep(2)

	# Stop simulation
	vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

	# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID)

	# Close the connection to V-REP
	vrep.simxFinish(clientID)


if __name__=="__main__":
	main(sys.argv)