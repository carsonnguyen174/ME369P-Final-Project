import numpy as np 
import pybullet as p
import pybullet_data
import time
import speech_recognition as sr

p.connect(p.GUI)
racecar_id= p.loadURDF('racecar/racecar.urdf', [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
aabb_min, aabb_max = p.getAABB(racecar_id)

length = aabb_max[0] - aabb_min[0]
width = aabb_max[1] - aabb_min[1]
height = aabb_max[2] - aabb_min[2]

print(f"Dimensions (LxWxH): {length:.2f} x {width:.2f} x {height:.2f}")
