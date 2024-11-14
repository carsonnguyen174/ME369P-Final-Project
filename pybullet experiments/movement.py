import numpy as np 
import pybullet as p
import pybullet_data
import time
import speech_recognition as sr
 
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(0)
#time.sleep(5.01)

#first asset
p.loadURDF('plane.urdf', [0,0,0], [0,0,0,1]) #x,y,z and the orientation
# second asset
targid=p.loadURDF('racecar/racecar.urdf', [0,0,0], [0,0,0,1], useFixedBase=True)
focus=targid
# for i in range(p.getNumJoints(targid)):
#     print(p.getJointInfo(targid, 1))
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     nertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     (1, b'chassis_inertia_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'chassis_inertia', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
#     numActiveThreads = 0
jointid=4
jtype=p.getJointInfo(targid, jointid)[2]
jlower=p.getJointInfo(targid, jointid)[8]
jupper=p.getJointInfo(targid, jointid)[9]

# for step in range(300):
#     focus_position, _ = p.getBasePositionAndOrientation(targid)
#     p.resetDebugVisualCamera(cameraDistance=3, cameraYaw=o, cameraPitch=-40, cameraTargetPosition=focus_position)
#     p.setpSimulation()
#     time.sleep(.01)


#changing joint angles
for step in range(500):
    joint_two_targ=np.random.uniform(jlower, jupper)
    joint_four_targ=np.random.uniform(jlower,jupper)
    #set velocity control instead of postion control
    p.setJointMotorControlArray(targid, [2,4], p.POSITION_CONTROL, targetPositions=[joint_two_targ, joint_four_targ])
    p.setpSimulation()
    # focus_position, _ = p.getBasePositionAndOrientation(targid)
    # p.resetDebugVisualCamera(cameraDistance=3, cameraYaw=o, cameraPitch=-40, cameraTargetPosition=focus_position)
    # p.setpSimulation()
  
