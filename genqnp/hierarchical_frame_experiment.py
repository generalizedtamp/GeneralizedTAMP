import pybullet as p
from GeneralizedTAMP.pybullet_planning.pybullet_tools.utils import create_box, connect, get_pose, set_pose, multiply, invert
import time


w = 2
h = 2

small_w = 0.2
small_h = 0.2
connect(use_gui=True)
b = create_box(w, w, h, color=[1,0,0,1.0])
smal_b = create_box(small_w, small_w, small_h, color=[0,1,0,1.0])



b_pose = get_pose(b)
smal_b_pose = get_pose(smal_b)

off_x = 0.0
off_y = 0.1
set_pose(b, ((off_x, off_y, 0), b_pose[1]))

smal_off_x = 0.4
smal_off_y = 0.9
set_pose(smal_b, ((smal_off_x, smal_off_y, h/2.0+small_h/2.0), p.getQuaternionFromEuler((0, 0, 1.0)) ))


print(get_pose(b))
print(get_pose(smal_b))

b_pose = get_pose(b)
smal_b_pose = get_pose(smal_b)

print("Relative")
relative = multiply(invert(b_pose), smal_b_pose)
print(relative)

yaw = 0
while(True):
	time.sleep(0.01)
	p.stepSimulation()


	off_x  += 0.01
	off_y += 0.01
	yaw+=0.01
	set_pose(b, ((off_x, off_y, 0),p.getQuaternionFromEuler((0, 0, yaw))))
	b_pose = get_pose(b)


	print("New absolute")
	new_absolute = multiply(b_pose, relative)
	set_pose(smal_b, new_absolute)
	print(new_absolute)
