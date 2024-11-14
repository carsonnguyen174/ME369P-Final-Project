import numpy as np 
import pybullet as p
import pybullet_data
import time
import speech_recognition as sr

# Initialize PyBullet
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Load assets
p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF('racecar/racecar.urdf', [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)

# Control parameters
speed = 0
steering_angle = 0

def command(command_str):
    parsed_command = command_str.split()
    if len(parsed_command) > 1:
        try:
            value = float(parsed_command[1])
        except ValueError:
            print("Value must be a positive float.")
            return None, parsed_command[0]
        return value, parsed_command[0]
    return None, command_str

def voice():
    possible_commands = ("forward", "backward", "left", "right", "stop")
    recog = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Say command:")
        audio = recog.listen(source)

    try:
        input_command = recog.recognize_google(audio)
        print(f"You said: {input_command}")
        
        value, action = command(input_command)
        if action not in possible_commands:
            print("Invalid command.")
            return None
        
        if action == "stop":
            return 0, "stop"
        return value, action
    except sr.UnknownValueError:
        print("Could not understand audio.")
    except sr.RequestError as e:
        print(f"Request error: {e}")
    return None

while True:
    # Get voice command
    command_output = voice()
    if command_output:
        value, action = command_output
        if action == "forward":
            speed = value
        elif action == "backward":
            speed = -value
        elif action == "left":
            steering_angle = np.deg2rad(value)
        elif action == "right":
            steering_angle = -np.deg2rad(value)
        elif action == "stop":
            speed = 0
            steering_angle = 0
        # Apply controls to the car
    p.setJointMotorControl2(targid, 2, p.VELOCITY_CONTROL, targetVelocity=speed)
    p.setJointMotorControl2(targid, 4, p.POSITION_CONTROL, targetPosition=steering_angle)

    p.stepSimulation()
    time.sleep(0.01)

