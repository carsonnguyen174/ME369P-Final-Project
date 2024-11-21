import pybullet as p
import pybullet_data
import speech_recognition as sr
import time
import threading
import numpy as np
from word2number import w2n as wn 

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

plane = p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
start_pos = [0, 0, .5]  
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
car = p.loadURDF("racecar/racecar.urdf",[1,0,5], start_orientation)
track=p.loadURDF("track/urdf/track.urdf", start_pos, start_orientation)
cylinder = p.loadURDF("obstacle1/urdf/obstacle1.urdf", [0, 0, 3], start_orientation)  
cube = p.loadURDF("obstacle2/urdf/obstacle2.urdf", [0, 0, 3], start_orientation)
#change last set of [x,y,z] to change obstacle placement, orgin is the flag in the track 
p.createConstraint(cylinder, -1, track, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [2, 4, 0.5]) 
p.createConstraint(cube, -1, track, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [-4, -2, 0.5])

wheels = [2,3]  # rear wheel indicies for motor torque
steering = [4, 6]  # front wheels indicies for steering angle
inactive_wheels = [ 5, 7]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

#create a correction value slider
correctionSlider = p.addUserDebugParameter("CorrectionValue", 0, 0.05, 0.01)

#for left turns decrease speed for right turns increase speed 
targetVelocity =  0 #rad/s
steeringAngle = 0 # degrees
set_force = 15 # Newtons
length = 0.00032500 #meters between the front axle and back axle
width = 0.0002 #meters between left and right wheel
left_wheel_angle = 0
right_wheel_angle = 0
correction = p.readUserDebugParameter(correctionSlider) 

p.changeDynamics(track, -1, lateralFriction=1) #change friction of the base link 
for wheel in wheels:
    p.changeDynamics(car, wheel, lateralFriction=1.5)  # Adjust friction for each wheel 
for steer in steering:
    p.changeDynamics(car, steer, lateralFriction=1.5)  # Adjust friction for each wheel

# Dictionary for possible driving keywords
keywords = {0: ['forward', 'drive', 'front', 'forwards'], 1: ['backward', 'reverse', 'backwards', 'back'], 2: ['left', 'last'], 3: ['right', 'write', 'rite'], 4: ['stop', 'park']}

def process_command():
    recog = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for commands...\
              \n'Forward x',  'Backward x', 'Left x', 'Right x', 'Stop',")
        audio = recog.listen(source)

    try:
        command = recog.recognize_google(audio).lower()
        print(f"You said: {command}")
        
        parsed_command = command.split()
        if len(parsed_command) == 0:
            print("No command detected. Try saying something")
            return None, None

        direction = parsed_command[0]

        if len(parsed_command) > 2:
            print("Please enter in a valid, singular command followed by a floating positive number")
            return None, None

        # Sorts out magnitude from the parsed command
        if len(parsed_command) > 1:
            try:
                magnitude = wn.word_to_num(parsed_command[1])
            except:
                print("Please enter in a valid command followed by a floating positive number")
                return None, None
        else:
            magnitude = 0

        return direction, magnitude

    except sr.UnknownValueError:
        print("Sorry, I didn't understand that. Please try again")
        return None, None
    except sr.RequestError as e:
        print(f"Error with the speech recognition service: {e}")
        return None, None

#set new direction/speed based on voice commands
def voice_command_thread():
    global targetVelocity, steeringAngle
    while True:
        direction, magnitude = process_command()
        # Move forward x magnitude
        if direction in keywords[0] and magnitude:
            targetVelocity = magnitude
        
        # Move forward
        elif direction in keywords[0]:
            steeringAngle = 0 # Sets steering angle back to default

        # Move backward
        elif direction in keywords[1]:
            targetVelocity = -magnitude
            steeringAngle = 0  
        
        # Turn left
        elif direction in keywords[2]:
            steeringAngle = np.deg2rad(magnitude)
            print(f"Turning left with angle: {steeringAngle}")
        
        # Turn right
        elif direction in keywords[3]:
            steeringAngle = -np.deg2rad(magnitude)
            print(f"Turning right with angle: {steeringAngle}")

        # Stop
        elif direction in keywords[4]:
            targetVelocity = 0
            steeringAngle = 0
        
#runs the current speed/direction while allowing to change it using voice commands
threading.Thread(target=voice_command_thread, daemon=True).start()

while p.isConnected():
    #update the correction value
    correction = p.readUserDebugParameter(correctionSlider)

    Position, Orientation = p.getBasePositionAndOrientation(car)
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=-90, cameraPitch=-40, cameraTargetPosition=Position)

    #gradually bring the wheels back to 0 position - allows wider turn
    if abs(steeringAngle) > np.deg2rad(correction):
        steeringAngle -= np.sign(steeringAngle)*np.deg2rad(correction)
    else:
        steeringAngle = 0
    
    #make sure the steering is within Ackermann Steering bounds (-35 -- 35 degrees)
    steeringAngle = np.clip(steeringAngle, -0.610865, 0.610865)

    #set the wheel angles for Ackermann drive
    right_wheel_angle = np.arctan((length*np.tan(steeringAngle))/(length + 0.5*width*np.tan(steeringAngle))) #right wheel
    left_wheel_angle = np.arctan((length*np.tan(steeringAngle))/(length - 0.5*width*np.tan(steeringAngle))) #left wheel

    # Sets the speed for each drive wheel
    for wheel in wheels:
        p.setJointMotorControl2(car, jointIndex=wheel, controlMode=p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=set_force)
    
    # Sets the steering for each front wheel
    p.setJointMotorControl2(car, jointIndex=steering[0], controlMode=p.POSITION_CONTROL, targetPosition=right_wheel_angle)
    p.setJointMotorControl2(car, jointIndex=steering[1], controlMode=p.POSITION_CONTROL, targetPosition=left_wheel_angle)

    p.stepSimulation()
    time.sleep(1/240)
