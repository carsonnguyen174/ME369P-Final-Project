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
start_pos = [0, 0, 0]  
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
car = p.loadURDF("racecar/racecar.urdf", start_pos, start_orientation)

wheels = [2, 3]  # rear wheels indicies for motor torque
steering = [4, 6]  # front wheels indicies for steering angle 

# Dictionary for possible driving keywords
keywords = {0: ['forward', 'drive'], 1: ['backward', 'reverse', 'backwards'], 2: ['left'], 3: ['right', 'write'], 4: ['stop', 'park']}

def process_command():
    recog = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for commands...")
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

targetVelocity = 0
steeringAngle = 0  # degrees

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
        
        # Turn left
        elif direction in keywords[2]:
            steeringAngle = np.rad2deg(magnitude)
        
        # Turn right
        elif direction in keywords[3]:
            steeringAngle = -np.rad2deg(magnitude)

        # Stop
        elif direction in keywords[4]:
            targetVelocity = 0
            steeringAngle = 0

#runs the current speed/direction while allowing to change it using voice commands
threading.Thread(target=voice_command_thread, daemon=True).start()

while p.isConnected():

    Position, Orientation = p.getBasePositionAndOrientation(car)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=Position)

    # Sets the speed for each drive wheel
    for wheel in wheels:
<<<<<<< Updated upstream
        p.setJointMotorControl2(car, jointIndex=wheel, controlMode=p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=10)

    for steer in steering:
        p.setJointMotorControl2(car, jointIndex=steer, controlMode=p.POSITION_CONTROL,targetPosition=steeringAngle)
=======
        p.setJointMotorControl2(bodyUniqueId=car, jointIndex=wheel, controlMode=p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=10)
    
    # Sets the steering for each front wheel
    for steer in steering:
        p.setJointMotorControl2(bodyUniqueId=car, jointIndex=steer, controlMode=p.POSITION_CONTROL, targetPosition=steeringAngle)
>>>>>>> Stashed changes

    p.stepSimulation()
    time.sleep(1/240)