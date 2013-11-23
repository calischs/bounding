#!/usr/bin/env python
from __future__ import division
from numpy import *
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d

f = 100 #Hz, frequency of microcontroller update
T = 1. #s, period of stride

limits = array([[0,pi/2],[0,pi/2],[-pi/2,pi/2]])
colors = array([[1,0,0],[0,1,0],[0,0,1]]) #three colors for three gen coords

#first row is time values, second is positions.  All in relative coordinates, 1->T in time, 1->pi/2 in space
a1 = array([
	[0., .25, .5, .75, 1.],
	[0., 1, 1, 0, 0.]
	])

a2 = array([
	[0., .25, .5, .75, 1.],
	[0., 0, 1, 1, 0.]
	])
phi = array([
	[0., 1],
	[0., 0.]
	])
a1 *= array([[T],[pi/2]])
a2 *= array([[T],[pi/2]])
phi *= array([[T],[pi/2]])

for i in range(3):
	plt.plot([0,T],[limits[i,0],limits[i,0]], c=colors[i],linestyle=':')
	plt.plot([0,T],[limits[i,1],limits[i,1]], c=colors[i],linestyle=':')
plt.plot(a1[0],a1[1],c=colors[0],label='a1',marker='o')
plt.plot(a2[0],a2[1],c=colors[1],label='a2',marker='o')
plt.plot(phi[0],phi[1],c=colors[2],label='phi',marker='o')

plt.grid()
plt.legend()
plt.show()

out = open('trajectory.txt','w')
a1_f = interp1d(a1[0],a1[1])
a2_f = interp1d(a2[0],a2[1])
phi_f = interp1d(phi[0],phi[1])

for i in range(int(T*f)):
	out.write('%.3f,%.3f,%.3f\n'%(a1_f(i/f), a2_f(i/f), phi_f(i/f)))
out.close()