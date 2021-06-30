import numpy as np
import matplotlib.pyplot as plt

data = np.load('thesis_data_3.npy.npz')


f_1027 = data['f1']
f_1028 = data['f2']
v_1027 = data['x1_dot']
v_1028 = data['x2_dot']
base_1027 = data['z1']
base_1028 = data['z2']
n_f = np.linspace(0,len(f_1027),len(f_1027))
n_v = np.linspace(0,len(v_1028),len(v_1028))
n_base = np.linspace(0,len(base_1027),len(base_1027))


f_1027_array = f_1027[60:]
f_1028_array = f_1028[60:]
v_1027_array = v_1027[60:]
v_1028_array = v_1028[60:]

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
ax1.set_ylim([0,40])

fig2 = plt.figure()
ax2 = fig2.add_subplot(1,1,1)
ax2.plot(n_f,-1*f_1028,label='1028')
ax2.set_xlabel('frame')
ax2.set_ylabel('force (N)')
ax2.set_ylim([-40,0])

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

fig5 = plt.figure()
ax5 = fig5.add_subplot(1,1,1)
ax5.plot(n_base,base_1027,label='1027')
ax5.set_xlabel('frame')
ax5.set_ylabel('position (m)')

base_1028 = base_1028 - 0.2

fig6 = plt.figure()
ax6 = fig6.add_subplot(1,1,1)
ax6.plot(n_base,-1*base_1028,label='1028')
ax6.set_xlabel('frame')
ax6.set_ylabel('position (m)')

ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()
ax5.legend()
ax6.legend()

#plt.show()
