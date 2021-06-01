'''
desired force = 20N along arm direction
desired velocity = 0.01m/s along lift direction
'''

from RobotRaconteur.Client import *     #import RR client library
import sys, time, traceback
import numpy as np
from scipy.signal import filtfilt
from scipy import stats
import matplotlib.pyplot as plt
import scipy
import PID

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

# Connect to robot2 status RR wire
arm2_status = arm2.status_rr.Connect()
lift2_status=lift2.status_rr.Connect()
base2_status=base2.status_rr.Connect()

# start the robots' motors
lift1.move_to(0.377) #Reach all the way out
arm1.move_to(0.05)
base1.translate_by(0.00001)
lift2.move_to(0.4) #Reach all the way out
arm2.move_to(0.05)
base2.translate_by(0.00001)
robot1.push_command()
robot2.push_command()
time.sleep(1)
now=time.time()

def bandpassfilter(signal):
	fs = 25.0
	lowcut = 2
	#highcut = 50.0

	nyq = 0.5*fs
	low = lowcut / nyq
	#high = highcut / nyq

	order = 6
	b,a = scipy.signal.butter(order, low, btype='low', analog=False)
	y = scipy.signal.filtfilt(b,a,signal, axis=0)

	return y

# some parameters
mass = 1.2
weight = mass * 9.8

#f1_d = weight / 2
#f2_d = weight / 2
f1_d = 20
f2_d = 20
v1_d = 0
v2_d = 0
K1_fwd = 0.000368
K2_fwd = 0.00035
K1_bwd = 0.0001
K2_bwd = 0.0001
#F1 = -K1*( - v_d) + f_d
#F2 = -K2*( - v_d) + f_d
time.sleep(2)
filter_flag = 0
f1_record = []
f2_record = []
y1_0 = 0.377
y2_0 = 0.4
y_vel = 0.01

P = 0.0
I = 0.0005
D = 0.0001
pid1 = PID.PID(P, I, D)
pid2 = PID.PID(P, I, D)
pid1.SetPoint = f1_d
pid2.SetPoint = f2_d
pid1.setSampleTime(0.05)
pid2.setSampleTime(0.05)


# discard the first few noisy readings
for i in range(20):
	discard1 = lift1_status.InValue['force']
	discard2 = lift2_status.InValue['force']
	discard3 = arm1_status.InValue['force']
	discard4 = arm2_status.InValue['force']
	time.sleep(0.05) # the motor sampling frequency is 25 Hz

x1_dot = 0.0
x2_dot = 0.0
f1_reading = 0
f2_reading = 0
global_count = 0
y_vel_count = 0
while True:
	try:
		if global_count > 90:
			y_vel = 0.01
			y_vel_count += 1

		now=time.time()

		# collect 25 data points. When the num is reached, remove the oldest point and add the newest point
		if filter_flag == 0:
			count = 0
			arm1_force = []
			arm2_force = []
			while count < 25:
				arm1_force.append(arm1_status.InValue['force'])
				arm2_force.append(arm2_status.InValue['force'])
				count += 1
				time.sleep(0.06) # the motor sampling frequency is 25 Hz
		else:
			arm1_force.pop(0)
			arm2_force.pop(0)
			count_ = 0

			# force reading is very noisy when the arm is moving
			# this while loop is created to discard the first few readings
			prev_value=arm1_status.InValue['force']
			count_=0
			while True:
				if count_==3:
					break
				if prev_value == arm1_status.InValue['force']:
					continue
				else:
					count_+=1
					prev_value=arm1_status.InValue['force']


			f1_reading = arm1_status.InValue['force']
			f2_reading = arm2_status.InValue['force']
			arm1_force.append(f1_reading)
			arm2_force.append(f2_reading)

		filter_flag = 1
		arm1_force_filtered = np.array(bandpassfilter(arm1_force))
		arm2_force_filtered = np.array(bandpassfilter(arm2_force))
		arm1_force_mean = np.mean(arm1_force_filtered)
		arm2_force_mean = np.mean(arm2_force_filtered)

		f1 = round(arm1_force_mean,5)
		f2 = round(arm2_force_mean,5)
		diff_f1 = f1_d - f1
		diff_f2 = f2_d - f2 

		if abs(diff_f1) < 1:
			diff_f1 = 0

		if abs(diff_f2) < 1:
			diff_f2 = 0

		f1_record.append(f1)
		f2_record.append(f2)

		pid1.update(f1)
		pid2.update(f2)
		offset1 = pid1.output
		offset2 = pid2.output

		# various gains for forward and backward motion
		if diff_f1 > 0:
			x1_dot = K1_fwd * diff_f1 + v1_d
		else:
			x1_dot = K1_bwd * diff_f1 + v1_d

		if diff_f2 > 0:
			x2_dot = K2_fwd * diff_f2 + v2_d
		else:
			x2_dot = K2_bwd * diff_f2 + v2_d
 
		y1 = y1_0 + y_vel * y_vel_count
		y2 = y2_0 + y_vel * y_vel_count
	
		arm1.move_by(x1_dot)
		arm2.move_by(x2_dot)
		lift1.move_to(y1) # maintain the pose of the lift
		lift2.move_to(y2) # maintain the pose of the lift
		base1.translate_by(0.0) # maintain the pose of the base
		base2.translate_by(0.0) # maintain the pose of the base
		base1.rotate_by(0.0)
		base1.rotate_by(0.0)
		robot1.push_command()
		robot2.push_command()
		print(time.time()-now)
		global_count += 1
	except:
		traceback.print_exc()
		break

time.sleep(0.5)
lift1.move_to(0.3)
lift2.move_to(0.3)
arm1.move_to(0.0)
arm2.move_to(0.0)
robot1.push_command( )
robot2.push_command( )
print ('Retracting...')

n = np.linspace(0,len(f1_record),len(f1_record))
f1_record = np.array(f1_record)
f2_record = np.array(f2_record)
fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)
ax1.plot(n, f1_record,label='1027')
ax1.set_xlabel('number of samples')
ax1.set_ylabel('force (N)')
ax2.plot(n, f2_record,label='1028')
ax2.set_xlabel('number of samples')
ax2.set_ylabel('force (N)')
ax1.legend()
ax2.legend()
plt.show()