import pybullet as p
import pybullet_data
import time
import numpy as np 
import speech_recognition as sr


def voice():
    """Captures a voice command, parses it, and returns direction and magnitude."""
    print('Available Commands:')
    print(' 1. "Forward <x>"')
    print(' 2. "Backward <x>"')
    print(' 3. "Left <x>"')
    print(' 4. "Right <x>"')
    print(' 5. "Stop"')

    print("Say command:")

    recog = sr.Recognizer()
    
    with sr.Microphone() as source:
        audio = recog.listen(source)

    try:
        # Recognize the speech input
        input = recog.recognize_google(audio)
        print(f"You said: {input}")

        # Split into direction and magnitude
        parsed_command = input.split()
        if len(parsed_command) < 1:
            print("Invalid command.")
            return None

        # Extract direction (first word) and magnitude (optional second word)
        direction = parsed_command[0].lower()  # First word is the direction
        if direction == "stop":
            return direction, 0  # No magnitude needed for "stop"

        if len(parsed_command) > 1:
            try:
                magnitude = float(parsed_command[1])  # Convert second word to a number
            except ValueError:
                print("Invalid number. Please say a numeric value after the direction.")
                return None
        else:
            print("Command must include a numeric value.")
            return None

        return direction, magnitude

    except sr.UnknownValueError:
        print("Could not understand audio! Please try again.")  
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
    
    return None


# PyBullet Simulation Setup
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

plane = p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # yaw, pitch, roll
car = p.loadURDF("racecar/racecar.urdf", start_pos, start_orientation)

num_joints = p.getNumJoints(car)
wheels = [2, 3]  # Rear wheels for motor torque
steering = [4, 6]  # Front wheels control angle

targetVelocity = 0
steeringAngle = 0

# Main Simulation Loop
while True:
    # Get voice command
    command = voice()
    if not command:
        continue  # Skip invalid commands

    direction, magnitude = command  # Split command into direction and magnitude

    # Match the direction and update motion
    match direction:
        case "forward": 
            targetVelocity = magnitude
        case "backward": 
            targetVelocity = -magnitude
        case "left": 
            steeringAngle = -magnitude
        case "right" or "write": 
            steeringAngle = magnitude
        case "stop": 
            targetVelocity = 0
            steeringAngle = 0

    # Apply commands to PyBullet simulation
    for wheel in wheels:
        p.setJointMotorControl2(
            bodyUniqueId=car,
            jointIndex=wheel,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=targetVelocity,
            force=10
        )

    for steer in steering:
        p.setJointMotorControl2(
            bodyUniqueId=car,
            jointIndex=steer,
            controlMode=p.POSITION_CONTROL,
            targetPosition=steeringAngle
        )

    p.stepSimulation()
    time.sleep(1 / 240)
