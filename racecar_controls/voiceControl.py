import pybullet as p
import pybullet_data
import speech_recognition as sr
import time
import threading
import word2number as wn #some times reads as five and not 5 (and other numbers) so it won't convert and move need to implement this in the code 

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

plane = p.loadURDF('plane.urdf', [0, 0, 0], [0, 0, 0, 1])
start_position = [0, 0, 0]  
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
# chassis is about 10.3 x 5.1 x 1.5 mm
car = p.loadURDF("racecar/racecar.urdf", start_position, start_orientation)
#track = p.loadURDF("", start_position, start_orientation)

wheels = [2, 3]  # rear wheels indicies for motor torque
steering = [4, 6]  # front wheels indicies for steering angle 

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

targetVelocity = 10
steeringAngle = 0 # radians

#set new direction/speed based on voice commands 
def voice_command_thread():
    global targetVelocity, steeringAngle
    while True:
        direction, magnitude = process_command()
        if direction == "forward":
            targetVelocity = magnitude
        elif direction == "backward":
            targetVelocity = -magnitude
        elif direction == "left":
            steeringAngle = -magnitude
        elif direction == "right":
            steeringAngle = magnitude
        elif direction == "write":
            steeringAngle = magnitude
        elif direction == "stop":
            targetVelocity = 0
            steeringAngle = 0

#runs the current speed/direction while allowing to change it using voice commands
threading.Thread(target=voice_command_thread, daemon=True).start()

while p.isConnected():

    Position, Orientation = p.getBasePositionAndOrientation(car)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=Position)

    for wheel in wheels:
        p.setJointMotorControl2(car, jointIndex=wheel, controlMode=p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=10)

    for steer in steering:
        p.setJointMotorControl2(car, jointIndex=steer, controlMode=p.POSITION_CONTROL,targetPosition=steeringAngle)

    p.stepSimulation()
    time.sleep(1/240)