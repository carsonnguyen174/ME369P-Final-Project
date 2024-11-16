import pybullet as p
import pybullet_data
import speech_recognition as sr
import numpy as np 
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

plane=p.loadURDF('plane.urdf', [0,0,0], [0,0,0,1])
start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0]) #yaw, pitch, roll
car = p.loadURDF("racecar/racecar.urdf", start_pos, start_orientation)

num_joints = p.getNumJoints(car)
wheels = [2, 3]  #rear wheels for motor torque
steering = [4, 6]  #front wheels control angle 

targetVelocity = 0 
steeringAngle = 0  #radians

# direction=command[0]
# magnitude=command[1]
# match direction:
#     case "forward": 
#         targetVelocity = magnitude

#     case "backward": 
#         targetVelocity = -magnitude

#     case "left": 
#         steeringAngle = -magnitude

#     case "right": 
#         steeringAngle = magnitude

#     case "stop": 
#         targetVelocity=0
#         steeringAngle=0


for _ in range(1000):
    #forward/backward
    for wheel in wheels:
        p.setJointMotorControl2(bodyUniqueId=car,jointIndex=wheel,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=targetVelocity,force=10  # Maximum torque
        )

    #turning
    for steer in steering:
        p.setJointMotorControl2(bodyUniqueId=car,jointIndex=steer,controlMode=p.POSITION_CONTROL,targetPosition=steeringAngle)

    p.stepSimulation()
    time.sleep(1/240)
