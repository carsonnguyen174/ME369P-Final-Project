import pybullet as p
import pybullet_data
import speech_recognition as sr
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

plane = p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
start_pos = [0, 0, 0]  
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
car = p.loadURDF("racecar/racecar.urdf", start_pos, start_orientation)

wheels = [2, 3]  # rear wheels for motor torque
steering = [4, 6]  # front wheels for steering angle 

def process_voice_command():
    recog = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Listening for commands...")
        audio = recog.listen(source)

    try:
        command = recog.recognize_google(audio).lower()
        print(f"You said: {command}")

        # Parse command for direction and magnitude
        parsed_command = command.split()
        if len(parsed_command) == 0:
            print("No command detected.")
            return None, None

        direction = parsed_command[0]
        magnitude = float(parsed_command[1]) if len(parsed_command) > 1 else 0

        return direction, magnitude

    except sr.UnknownValueError:
        print("Sorry, I didn't understand that.")
        return None, None
    except sr.RequestError as e:
        print(f"Error with the speech recognition service: {e}")
        return None, None

while p.isConnected():
    direction, magnitude = process_voice_command()

    targetVelocity = 50
    steeringAngle = .5  # radians

    # If no valid command, continue with the previous values
    if direction is None:
        print(f"Using previous values -> Speed: {targetVelocity}, Steering: {steeringAngle}")
        direction = "no_command"

    if direction == "forward":
        targetVelocity = magnitude
    elif direction == "backward":
        targetVelocity = -magnitude
    elif direction == "left":
        steeringAngle = -magnitude
    elif direction == "right":
        steeringAngle = magnitude
    elif direction == "stop":
        targetVelocity = 0
        steeringAngle = 0


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
