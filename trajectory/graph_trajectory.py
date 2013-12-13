#!/usr/bin/env python
from __future__ import division
from numpy import *
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from smooth import smooth

#TODO: put in spine zero offset -- maybe can account for backlash + gravity?
#TODO: put in phase shift?

f = 500 #Hz, frequency of microcontroller update

limits = array([[0,40],[0,40],[-15,15]])
colors = array([[1,0,0],[0,1,0],[0,0,1]]) #three colors for three gen coords

#first row is time values, second is positions.  All in relative coordinates, 1->T in time, 1->pi/2 in space

#based on standup and sitdown routines, a1 and a2 must start and end with 1, while phi must start and end with 0

T = .33 #s, period of stride
ramp = .06 #length of time to ramp up and down in a1 and a2
w = .5*(1-2*ramp)#.3 #length of time for one leg to stay at max
stand = 35. #amplitude of leg standing
bent = 20. #amplitude of leg bent
spine_amp = 7.
spine_zero = 0
spine_shift_frac = -.25 #fraction of period
smoothing_window = 91 #wider window for more smoothing, must be odd

spine_p = spine_zero+spine_amp
spine_m = spine_zero-spine_amp
spine_shift = int(round(spine_shift_frac*f*T))

a1 = array([
	[0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.],
	[stand, stand, bent, bent, stand, stand]
	])
a2 = array([
	[0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.],
	[bent, bent, stand, stand, bent, bent]
	])
phi = array([
	[0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.],
	[spine_m,   spine_m,      spine_p,         spine_p,            spine_m,      spine_m]
	])

'''
a1 = array([
	[0., base, base+ramp, base+ramp+width, base+2*ramp+width, 1.],
	[stand, stand, bent, bent, stand, stand]
	])

a2 = array([
	[0., 1-base-2*ramp-width, 1-base-ramp-width, 1-base-ramp, 1-base, 1.],
	[stand, stand, bent, bent, stand, stand]
	])
phi = array([
	[0, base, base+ramp, base+ramp+width, base+2*ramp+width, 1-base-2*ramp-width, 1-base-ramp-width, 1-base-ramp, 1-base,1],
	[0.,   0,         0.,            0,             0,            0,-0,-0,0 , 0]
	])
'''
a1 *= array([[T],[1.]])
a2 *= array([[T],[1.]])
phi *= array([[T],[1.]])

#plot limits
#for i in range(3):
	#plt.plot([0,T],[limits[i,0],limits[i,0]], c=colors[i],linestyle=':')
	#plt.plot([0,T],[limits[i,1],limits[i,1]], c=colors[i],linestyle=':')
#plot control points
plt.plot(a1[0],a1[1],c=colors[0],label='$a_1$',marker='o')
#plt.plot(a2[0],a2[1],c=colors[1],label='$a_2$',marker='o')
plt.plot(phi[0]-spine_shift/(f),phi[1],c=colors[2],label='$\psi$',marker='o')

t = arange(0,T,1/f)
a1_v = interp1d(a1[0],a1[1])(t)
a2_v = interp1d(a2[0],a2[1])(t)
#phi_v = interp1d(phi[0],phi[1])(t)
phi_v = roll(interp1d(phi[0],phi[1])(t) , -spine_shift,axis=0)

a1_v = smooth(a1_v,window_len=smoothing_window)
a2_v = smooth(a2_v,window_len=smoothing_window)
phi_v = smooth(phi_v,window_len=smoothing_window)

print 'Number samples: %d'%(shape(t)[0])
print "Transition: %d, %d, %d"%(a1_v[0], a2_v[0], phi_v[0])

#plot time points
plt.plot(t,a1_v,c=colors[0],marker='.')
#plt.plot(t,a2_v,c=colors[1],marker='.')
plt.plot(t,phi_v,c=colors[2],marker='.')

y_lab = 18
sep = .02*T
for i in range(len(a1[0])-1):
	plt.arrow(a1[0][i]+.5*sep,y_lab, a1[0][i+1]-a1[0][i]-.5*sep,0, color='black',
		width=.3,
		fc="k", ec="k",
		head_width=0., head_length=0.
		)
plt.text(T*(.25*w), y_lab-1, '$w/2$',size=20,rotation=-90,ha='center',va='top')
plt.text(T*(.5*w+.5*ramp), y_lab-1, '$ramp$',size=20,rotation=-90,ha='center',va='top')
plt.text(T*(w+ramp), y_lab-1, '$w$',size=20,rotation=-90,ha='center',va='top')
plt.text(T*(1.5*w+1.5*ramp), y_lab-1, '$ramp$',size=20,rotation=-90,ha='center',va='top')
plt.text(T*(1-.25*w), y_lab-1, '$w/2$',size=20,rotation=-90,ha='center',va='top')


plt.text(T*(.5*w+.5*ramp), stand, '$l_{max}$', size=20,ha='left')
plt.text(T*(.5*w+.5*ramp), bent, '$l_{min}$', size=20,ha='right')
plt.text(T*(-spine_shift_frac+.5*w+ramp-.02), spine_amp, '$spine\_amp$', size=20,ha='right')

plt.arrow(.5*sep,.25*spine_amp, -T*spine_shift_frac-.5*sep,0, color='black',
		width=.3,
		fc="k", ec="k",
		head_width=0., head_length=0.
		)
plt.text(T*(-.5*spine_shift_frac), .25*spine_amp, '$spine\_shift$',size=20,ha='center',va='bottom')



plt.xlabel('time')
plt.ylabel('position (degrees)')
plt.xlim([0,T])
plt.ylim([-10,40])
#plt.grid()
plt.legend(loc='lower right')
#plt.title('Ocelot Trajectories\n %d$^\circ$ spine phase offset'%(spine_shift_frac*360))
plt.title('Ocelot Trajectories')
plt.show()

out = open('trajectory.txt','w')
for a1i, a2i, phii in zip(a1_v,a2_v,phi_v):
	out.write('%d,%d,%d,\n'%(a1i,a2i,phii))
out.close()


'''
a1 = array([
	[0., base, base+ramp, base+ramp+width, base+2*ramp+width, 1.],
	[1., 1, 1, 1, 1, 1.]
	])

a2 = array([
	[0., 1-base-2*ramp-width, 1-base-ramp-width, 1-base-ramp, 1-base, 1.],
	[1., 1, 1, 1, 1., 1.]
	])
phi = array([
	[0, base, base+ramp, base+ramp+width, base+2*ramp+width, 1-base-2*ramp-width, 1-base-ramp-width, 1-base-ramp, 1-base,1],
	[0.,   0,         0.,            0,             0,            0,-0,-0,0 , 0]
	])
'''