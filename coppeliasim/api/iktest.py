import sim

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:
    print('Connected to CoppeliaSim')
else:
    print('Failed to connect to CoppeliaSim')
    exit(1)

# Get object handles
res, ur5_handle = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_blocking)
res, end_effector_handle = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_blocking)

# Set the desired target position and orientation for the end-effector
target_position = [0.5, 0.2, 0.4]  # Example position [x, y, z]
target_orientation = [0.0, 0.0, 0.0]  # Example orientation [roll, pitch, yaw]

# Perform inverse kinematics
res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(clientID, 'CoppeliaIK', sim.sim_scripttype_customizationscript, 'calculateIK', [ur5_handle, end_effector_handle], target_position + target_orientation, [], '', sim.simx_opmode_blocking)

print(res)

if res == 0:
    joint_angles = ret_floats
else:
    print('IK calculation failed')
    exit(1)

# Set joint angles
for i in range(len(joint_angles)):
    sim.simxSetJointTargetPosition(clientID, joint_angles[i], sim.simx_opmode_oneshot)

# Disconnect from CoppeliaSim
sim.simxFinish(clientID)
