import numpy as np
import scipy.optimize as spopt
import matplotlib.pyplot as plt
import pandas as pd
import ast  # safer than eval for parsing Python literals
# For LaTeX-powered plotting:
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['text.usetex'] = True
rcParams['font.size'] = 12

import scipy.signal as spsig
def strToFloatArr(str_array):
    """
    Converts an array of stringified lists into a 2D NumPy array of floats.
    
    Parameters:
        str_array (array-like): e.g.
            ['[9.42, -0.22, -0.88]', '[9.61, -0.38, -0.53]', '[9.50, -0.38, -0.38]']
    
    Returns:
        np.ndarray: 2D array of shape (n, m) with dtype float
    """
    # Parse each string into a Python list using ast.literal_eval
    list_of_lists = [ast.literal_eval(s) for s in str_array]
    
    # Convert to numpy array
    return np.array(list_of_lists, dtype=float)

#csvOut = pd.read_csv("XANTHUS_DATA.csv")
csvOut = pd.read_csv("XANTHUS2.csv")
print(csvOut.keys())
#exit(0)
timeStamp = np.array(csvOut['time'])
altDat = np.array(csvOut[' \"baro_filtered_alt\"'])
altDat_GPS = np.array(csvOut[' \"gps_alt\"'])
velDat = np.array(csvOut[' \"baro_filtered_velocity\"'])
baro_max_alt_dat = np.array(csvOut[' \"baro_max_alt\"'])
gps_max_alt_dat = np.array(csvOut[' \"gps_max_alt\"'])
accelDat = strToFloatArr(np.array(csvOut[' \"accelerometer\"']))[:,0] - 9.81
gyroDat = strToFloatArr(np.array(csvOut[' \"gyro\"']))[:,1]
servoDat = strToFloatArr(np.array(csvOut[' \"servos\"']))[:,7]
vertVel = np.array(csvOut[' \"accel_integrated_velo\"'])

yawDat = np.array(csvOut[' \"yaw_gyro_int\"'])
pitchDat = np.array(csvOut[' \"pitch_gyro_int\"'])



servoDat -= 1500
servoDat *= 60/500
servoDat += 2



rollDat = np.array(csvOut[' \"roll_gyro_int\"'])
targetRoll = 0

stateStringArr = np.array(csvOut[' \"state\"'])
ts_flight = np.array(csvOut[' \"flight_time\"'])

changeToFlight = 0
changeToApogee = 0
changeToDisreef = 0


def convertStateToInt(stateStringArr):
	global changeToFlight,changeToApogee,changeToDisreef
	new = []
	for i in range(len(stateStringArr)):
		intVal = 0
		if stateStringArr[i] == 'state.PRE_FLIGHT':
			intVal = 1
		elif stateStringArr[i] == 'state.FLIGHT':
			intVal = 2
			if changeToFlight == 0:
				changeToFlight = i
		elif stateStringArr[i] == 'state.APOGEE':
			intVal = 3
			if changeToApogee == 0:
				changeToApogee = i
		elif stateStringArr[i] == 'state.DISREEF':
			intVal = 4
			if changeToDisreef == 0:
				changeToDisreef = i

		new.append(intVal)

	return np.array(new)

stateIntArr = convertStateToInt(stateStringArr)

def integralOverInterval(timeseries,dat):
	new = []
	dt = timeseries[1]-timeseries[0]
	for i in range(len(timeseries)):
		new.append(0)
		for j in range(i):
			new[i] += dat[j]*dt

	return np.array(new)




#velDat = np.array(csvOut[' \"baro_filtered_vel\"'])
#altDat_GPS = np.array(csvOut[' \"gps_alt\"'])

mask = (timeStamp-1762635814 > 0)
mask2 = (timeStamp-1762635814 > 0)

iniShift = 1762635814+15
ts_plot = timeStamp[mask]-iniShift
ts_plot2 = timeStamp[mask2]-iniShift


integrVel = integralOverInterval(ts_plot,accelDat[mask])



funcOut = 180*np.acos(np.cos(yawDat*np.pi/180) * np.cos(pitchDat*np.pi/180))/np.pi
funcOutDeriv = spsig.savgol_filter(funcOut,2,1,1)*20 # linear interpolation, no smoothing
funcOutDeriv3 = spsig.savgol_filter(funcOut,20,2,1)*20
funcOutDeriv2 = []
dt = ts_plot2[1]-ts_plot2[0]
for i in range(len(funcOutDeriv)):
	size = int(1/dt) + 1 # half a second
	if i < size:  
		funcOutDeriv2.append(funcOutDeriv[i])
	else:
		funcOutDeriv2.append(np.mean(funcOutDeriv[i-size:i]))

funcOutDeriv2 = np.array(funcOutDeriv2)

trueApogee = ts_plot2[np.argmax(altDat[mask2][ts_plot2 < 11])]
detectedApogee = ts_plot2[np.argmax(altDat[mask2])]

#funcOut = yawDat/pitchDat

plt.subplots()
ax = plt.gca()
ax.plot(ts_plot2,altDat[mask2],label='Altitude (m)',color='red')
#ax.plot(ts_plot2,vertVel[mask2]*10,label='Vertical Velocity (m/s)',color='pink')

ax2 = ax.twinx()
ax2.plot(ts_plot2,funcOut[mask2],label='$\\gamma$ ($^\\circ$)',color='blue')
ax2.plot(ts_plot2,funcOutDeriv2[mask2],label='$20\\dot \\gamma$ ($^\\circ$/s)',color='purple')
ax2.plot(ts_plot2,funcOutDeriv3[mask2],label='$20\\dot \\gamma$ {\\tt (savgol)}',color='orange')
ax2.plot(ts_plot2,yawDat[mask2],label='Yaw $\\psi$ ($^\\circ$)',color='green',linestyle='dotted',alpha=0.5)
ax2.plot(ts_plot2,pitchDat[mask2],label='Pitch $\\theta$ ($^\\circ$)',color='teal',linestyle='dotted',alpha=0.5)
ax2.plot([-1],[-1],label='Altitude (m)',color='red')
ax.set_xlim(0,50)
ax2.set_ylim(0,180)
ax.set_ylim(0,450)
ax.set_xlabel("Time (s)")
ax2.set_ylabel("Angle From Vertical ($^\\circ$)")
ax.set_ylabel("Altitude (m)")
ax2.spines['left'].set_color('red')
ax2.spines['right'].set_color('blue')
ax2.yaxis.label.set_color('blue')
ax.yaxis.label.set_color('red')
ax2.yaxis.set_ticks(np.arange(0,181,45))
ax2.tick_params(axis='y', colors='blue')
ax.tick_params(axis='y', colors='red')
ax2.legend(loc='upper center')
ax2.hlines(45,0,50,color='k',linestyle='dotted')

pointPast45 = -1
for i in range(len(funcOut[mask2])):
	if altDat[mask2][i] > 100:
		if funcOut[mask2][i] > 45:
			pointPast45 = i
			break

localMaxAfter45 = -1
for i in range(len(funcOutDeriv2[mask2])):
	if i > pointPast45:
		if funcOutDeriv2[mask2][i] < funcOutDeriv2[mask2][i-1]:
			localMinAfter45 = i
			break

ax2.vlines(ts_plot2[pointPast45],0,180,color='k',linestyle='dotted')
ax2.vlines(ts_plot2[localMinAfter45],0,180,color='r',linestyle='dotted')
ax2.vlines(trueApogee,0,180,color='blue',linestyle='dashed')
print("Got faster by " + str(detectedApogee - ts_plot2[localMinAfter45]))
print("Got off by " + str(trueApogee - ts_plot2[localMinAfter45]))




plt.savefig('fig1.pdf')
#plt.savefig('fig0.pdf')
plt.show()




exit(0)


dt = timeStamp[1] - timeStamp[0]

fig,axs = plt.subplots(nrows=4,sharex='col')
ax1,ax2,ax3,ax4 = axs[0], axs[1],axs[2],axs[3]
ax1.plot(ts_plot,velDat[mask],label='baro velocity m/s',color='blue')
ax1.plot(ts_plot,integrVel,label='accel integral velocity m/s',color='orange')
ax1.plot(ts_plot,spsig.savgol_filter(altDat[mask],20,2,1)/dt,label='savgol deriv',color='purple',linewidth=5,alpha=0.3)
ax1.set_xlim(0,40-15)
ax1.hlines(100,*ax1.get_xlim(),color='k',label='100 m/s')
ax1.hlines(50,*ax1.get_xlim(),color='red',label='50 m/s')
ax1.hlines(0,*ax1.get_xlim(),color='k',linestyle='dotted',label='0 m/s')

ax1.vlines(timeStamp[changeToFlight]-iniShift, *ax1.get_ylim(),color='k')
ax1.vlines(timeStamp[changeToApogee]-iniShift, *ax1.get_ylim(),color='r')
ax1.vlines(timeStamp[changeToDisreef]-iniShift,*ax1.get_ylim(),color='g')
ax1.set_ylim(-200,200)

ax1.set_title('Velocity (m/s)')
ax1.legend(ncol=3)


ax2.set_title('Altitude (m)')
#ax2.plot(ts_plot,altDat[mask],label='baro altitude m',color='blue')
ax2.plot(ts_plot,baro_max_alt_dat[mask] - altDat[mask],label='baro altitude diff from max m',color='blue')
#ax2.plot(ts_plot,altDat_GPS[mask],label='gps altitude m',color='orange')
ax2.plot(ts_plot,gps_max_alt_dat[mask] - altDat_GPS[mask],label='gps altitude diff from max m',color='orange')
ax2.vlines(timeStamp[changeToFlight]-iniShift,*ax2.get_ylim(),color='k')
ax2.vlines(timeStamp[changeToApogee]-iniShift,*ax2.get_ylim(),color='r')
ax2.vlines(timeStamp[changeToDisreef]-iniShift,*ax2.get_ylim(),color='g')
ax2.hlines(20,*ax2.get_xlim(),color='k')
ax2.hlines(150,*ax2.get_xlim(),color='k')
ax2.set_ylim(-200,500)
ax2.legend()


ax3.set_title('Accel (m/s$^2$)')
ax3.plot(ts_plot,accelDat[mask],label='accel m/s$^2$',color='blue')
ax3.vlines(timeStamp[changeToFlight]-iniShift, *ax3.get_ylim(),color='k')
ax3.vlines(timeStamp[changeToApogee]-iniShift, *ax3.get_ylim(),color='r')
ax3.vlines(timeStamp[changeToDisreef]-iniShift,*ax3.get_ylim(),color='g')

ax4.set_title('State')
ax4.plot(ts_plot,stateIntArr[mask],color='b')
ax4.set_yticks([1,2,3,4],['\\tt PRE_FLIGHT','\\tt FLIGHT','\\tt APOGEE','\\tt DISREEF'])
ax4.vlines(timeStamp[changeToFlight]-iniShift, *ax4.get_ylim(),color='k')
ax4.vlines(timeStamp[changeToApogee]-iniShift, *ax4.get_ylim(),color='r')
ax4.vlines(timeStamp[changeToDisreef]-iniShift,*ax4.get_ylim(),color='g')




print("Triggered APOGEE @ {}".format((ts_flight[changeToApogee]-ts_flight[changeToFlight])*1e-3))
print("Triggered DISREEF @ {}".format((ts_flight[changeToDisreef]-ts_flight[changeToApogee])*1e-3))


#plt.show()

def localMinMaxFetcher(dat):
	minIndices = []
	maxIndices = []
	for i in range(1,len(dat)-1):
		prev = dat[i-1]
		now = dat[i]
		future = dat[i+1]

		if (now > future) & (now > prev):
			maxIndices.append(i)

		elif (now < future) & (now < prev):
			minIndices.append(i)

	return (np.array(minIndices),np.array(maxIndices))




fig, ax = plt.subplots()

procMask = (ts_plot > 0.82) & (ts_plot < 12)
xtraMask = (ts_plot > 12)

procRoll = rollDat[mask][procMask]
rollXtra = rollDat[mask][xtraMask]
tProc = ts_plot[procMask]

minTs = []
maxTs = []
minRs = []
maxRs = []

minInds,maxInds = localMinMaxFetcher(procRoll)
ax.plot(tProc,procRoll,color='b',label='roll')
ax.plot(ts_plot[xtraMask],rollXtra,color='b',alpha=0.5,linestyle='dashed')

for i in range(len(minInds)):
	index = minInds[i]
	minTs.append(tProc[index])
	minRs.append(procRoll[index])
	ax.scatter(tProc[index],procRoll[index],color='red',marker='x')

for i in range(len(maxInds)):
	index = maxInds[i]
	maxTs.append(tProc[index])
	maxRs.append(procRoll[index])
	ax.scatter(tProc[index],procRoll[index],color='blue',marker='x')

def exp(x,a,b):
	return a*np.exp(b*x)

opt0,pcov = spopt.curve_fit(exp,minTs[2:],minRs[2:],p0=[-400,-0.01])
opt1,pcov = spopt.curve_fit(exp,maxTs[3:],maxRs[3:],p0=[-400,-0.01])
plotSpace = np.linspace(np.min(minTs),np.max(minTs),1000)
plotSpace2 = np.linspace(np.min(maxTs),np.max(maxTs),1000)
ax.plot(plotSpace,exp(plotSpace,*opt0),color='r',linestyle='dotted')
ax.plot(plotSpace2,exp(plotSpace2,*opt1),color='b',linestyle='dotted')

def shiftDHO(x,y0,a,tau,omega,phi,tau2):
	return y0+a*np.sin(x*omega*np.exp(-tau2*x)+phi)*np.exp(-tau*x)


ampGuess = (np.mean(maxRs) - np.mean(minRs))/2
shiftGuess = (np.mean(maxRs) + np.mean(minRs))/2
omegaGuess = 2*np.pi/(minTs[3] - minTs[2])

fitMask = (tProc > minTs[2]) & (tProc < minTs[4])
opt2,pcov = spopt.curve_fit(shiftDHO,tProc[fitMask],procRoll[fitMask],p0=[shiftGuess,ampGuess,opt0[1],omegaGuess,np.pi/2,opt0[1]*1])


plotSpace2 = np.linspace(minTs[1],tProc[-1],1000)
ax.plot(plotSpace2,shiftDHO(plotSpace2,*opt2),color='purple',linewidth=5,alpha=0.2)
print(opt2)
ax.vlines(tProc[fitMask][0],*ax.get_ylim(),color='k',linestyle='dotted')
ax.vlines(tProc[fitMask][-1],*ax.get_ylim(),color='k',linestyle='dotted')
#bGuess = -0.1
#aGuess = minRs[2]/np.exp(bGuess*minTs[2])
#ax.plot(plotSpace,exp(plotSpace,aGuess,bGuess),color='r',linestyle='dotted')

# CHOOSE TIMESTAMP 3
chosenTim = 3
indexOfTimestamp = 0
for i in range(len(ts_plot)):
	if ts_plot[i] < chosenTim:
		indexOfTimestamp = i
	else:
		indexOfTimestamp = i
		break
alt0 = altDat[mask][i]
v0 = velDat[mask][i]
rotRate0 = gyroDat[mask][i]
rot0 = rollDat[mask][i]


print("alt0 = " + str(alt0))
print("v0 = " + str(v0))
print("rotRate0 = " + str(rotRate0))
print("rot0 = " + str(rot0))
print("tab0 = " + str(servoDat[mask][i]))

ax2 = ax.twinx()
ax2.plot(ts_plot,servoDat[mask],color='r')




ax.set_xlim(0,16)
ax.set_ylim(-75,50)
plt.show()
