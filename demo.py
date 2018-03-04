import vrep
import time
import numpy as np

# Run our Baxter Demonstration. Based off the sample code in 'test.py'
# from Professor Bretl.

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

for i in range(7):
	print(i)
	print()
	# Get "handle" to the first joint of robot
	result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint' + str(i+1), vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get object handle for first joint')

	# Start simulation
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

	# Wait two seconds
	time.sleep(2)

	# Get the current value of the first joint variable
	result, theta0 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get first joint variable')
	print('current value of first joint variable: theta = {:f}'.format(theta0))

	# Set the desired value of the first joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta0 + (np.pi), vrep.simx_opmode_oneshot)

	time.sleep(2)

	# Get the current value of the first joint variable
	result, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get first joint variable')
	print('current value of first joint variable: theta = {:f}'.format(theta1))

	# Set the desired value of the first joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta1 - (2*np.pi), vrep.simx_opmode_oneshot)

	# Wait two seconds
	time.sleep(2)

	# Get the current value of the first joint variable
	result, theta2 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get first joint variable')
	print('current value of first joint variable: theta = {:f}'.format(theta2))

	# Set the desired value of the first joint variable
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta0, vrep.simx_opmode_oneshot)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)