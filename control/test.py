from robolink import *
from robodk import *
RDK = Robolink()
robot = RDK.Item('UR5')
targetPose = RDK.Item('Target 1')
prog = RDK.AddProgram('My Program', robot)
pickupArea_pose = targetPose.Pose()
robot.setSpeed(50)
def moveRobottoPickUp_Reach(pos):
    if pos == 'active':
        robot.MoveJ(pickupArea_pose)  # Move the robot to the specified joint configuration
    if pos == 'draw':
        for i in range(5):
            ang = i*2*pi/4
            posei = pickupArea_pose*rotz(ang)*transl(300,0,0)*rotz(-ang)
            robot.MoveL(posei)
