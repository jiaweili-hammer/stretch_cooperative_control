'''
analysis for demo 1's data
consider stage two only
'''

import numpy as np
import matplotlib.pyplot as plt

data = np.load('thesis_data_1.npy.npz')

f_1027 = data['arr_0']
f_1028 = data['arr_1']
v_1027 = data['arr_2']
v_1028 = data['arr_3']
n_f = np.linspace(0,len(f_1027),len(f_1027))
n_v = np.linspace(0,len(v_1028),len(v_1028))


f_1027_array = f_1027[20:]
f_1028_array = f_1028[20:]
v_1027_array = v_1027[20:]
v_1028_array = v_1028[20:]

print('std of f_1027', np.std(f_1027_array))
print('var of f_1027', np.var(f_1027_array))
print('mean of f_1027', np.mean(f_1027_array))

print('std of f_1028', np.std(f_1028_array))
print('var of f_1028', np.var(f_1028_array))
print('mean of f_1028', np.mean(f_1028_array))

print('std of v_1027', np.std(v_1027_array))
print('var of v_1027', np.var(v_1027_array))
print('mean of v_1027', np.mean(v_1027_array))

print('std of v_1028', np.std(v_1028_array))
print('var of v_1028', np.var(v_1028_array))
print('mean of v_1028', np.mean(v_1028_array))




fig1 = plt.figure()
ax1 = fig1.add_subplot(1,1,1)
ax1.plot(n_f, f_1027,label='1027')
ax1.set_xlabel('frame')
ax1.set_ylabel('force (N)')

fig2 = plt.figure()
ax2 = fig2.add_subplot(1,1,1)
ax2.plot(n_f,-1*f_1028,label='1028')
ax2.set_xlabel('frame')
ax2.set_ylabel('force (N)')

fig3 = plt.figure()
ax3 = fig3.add_subplot(1,1,1)
ax3.plot(n_v,v_1027,label='1027')
ax3.set_xlabel('frame')
ax3.set_ylabel('velocity (m/s)')

fig4 = plt.figure()
ax4 = fig4.add_subplot(1,1,1)
ax4.plot(n_v,-1*v_1028,label='1028')
ax4.set_xlabel('frame')
ax4.set_ylabel('velocity (m/s)')

ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()
ax1.set_ylim([0,40])
ax2.set_ylim([-40,0])

#plt.show()

