
from RobotRaconteur.Client import *     #import RR client library
import sys, time, traceback
import numpy as np

#url of each robot
url1='rr+tcp://192.168.1.64:23232/?service=stretch'
url2='rr+tcp://192.168.1.28:23232/?service=stretch'

#Startup, connect, and pull out the arm/lift/eoa from the objref in robot obj  
robot1=RRN.ConnectService(url1)
robot2=RRN.ConnectService(url2)

# create robot 1 instance
base1 = robot1.get_base()
lift1=robot1.get_lift()
arm1=robot1.get_arm()
end_of_arm1=robot1.get_end_of_arm()

# create robot 2 instance
base2 = robot2.get_base()
lift2=robot2.get_lift()
arm2=robot2.get_arm()
end_of_arm2=robot2.get_end_of_arm()

# Connect to robot1 status RR wire
arm1_status = arm1.status_rr.Connect()
lift1_status=lift1.status_rr.Connect()
base1_status=base1.status_rr.Connect()
end_of_arm1_status=end_of_arm1.status_rr.Connect()

# Connect to robot2 status RR wire
arm2_status = arm2.status_rr.Connect()
lift2_status=lift2.status_rr.Connect()
base2_status=base2.status_rr.Connect()
end_of_arm2_status=end_of_arm2.status_rr.Connect()


lift1.move_to(0.03) #Reach all the way out
arm1.move_to(0.03)
base1.move_to(0.01)
lift2.move_to(0.03) #Reach all the way out
arm2.move_to(0.03)
base2.move_to(0.01)
robot1.push_command()
robot2.push_command()
time.sleep(3)
now=time.time()

mass = 1.2
weight = mass * 9.8

f1_d = weight / 2
f2_d = weight / 2
v_d = 0.1
K1 = 150 * np.eye(3)
K2 = 150 * np.eye(3)
F1 = -K1*( - v_d) + f_d
F2 = -K2*( - v_d) + f_d


while True:
	try:
		f1 = round(-1 * lift1_status.InValue['force'],3)
		x1 = K1 * (f1_d - f1) + v_d
		f2 = round(-1 * lift2_status.InValue['force'],3)
		x2 = K2 * (f2_d - f2) + v_d
		base1.translate_by(x1)
		base2.translate_by(x2)

	except:
		traceback.print_exc()
		break

print ('Retracting...')
lift1.move_to(0.3)
arm1.move_to(0.0)
end_of_arm1.move_to('wrist_yaw',0.)
end_of_arm1.move_to('stretch_gripper',0.)

lift2.move_to(0.3)
arm2.move_to(0.0)
end_of_arm2.move_to('wrist_yaw',0.)
end_of_arm2.move_to('stretch_gripper',0.)

robot1.push_command( )
robot2.push_command( )
