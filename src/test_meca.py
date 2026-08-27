import mecademicpy.robot as mdr

robot = mdr.Robot()
robot.Connect(address='192.168.0.100')

print("Connected:", robot.IsConnected())

# Activate and home the robot (required before any motion commands)
robot.ActivateRobot()
robot.Home()
robot.WaitHomed()

print("Robot is activated and homed.")

# Read current pose
pose = robot.GetPose()
print("Current pose (x, y, z, alpha, beta, gamma):", pose)

robot.Disconnect()