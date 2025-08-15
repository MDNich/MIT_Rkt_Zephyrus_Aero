#!/opt/homebrew/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

csvOut = np.array(pd.read_csv('allOut.csv'))
convertEU_US = lambda x: x.replace(".","").replace(",",".")

angAxis = csvOut[2][2:]
angAxis = 180-np.round(np.degrees(np.array([float(convertEU_US(angAxis[i])) for i in range(len(angAxis))])))
angU = np.unique(angAxis)[::-1]

velAxis = csvOut[1][2:]
velAxis = np.round(-1*np.array(velAxis,dtype=float))
velU = np.unique(velAxis)[::-1]


normalForceDat = csvOut[7][2:]
normalForceDat = np.array([float(convertEU_US(normalForceDat[i])) for i in range(len(normalForceDat))])

torqueDat = normalForceDat * np.cos(np.radians(angAxis)) * 0.13 # meters

torquePlot = np.reshape(torqueDat,(len(angU),len(velU)))
normalForcePlot = np.reshape(normalForceDat,(len(angU),len(velU)))

torqueServoDat = normalForceDat * 0.016 # meters
torqueServoPlot = np.reshape(torqueServoDat,(len(angU),len(velU)))

# --- import data from second simulation ---

csvOut2 = np.array(pd.read_csv('partialStudy_results.csv'))

angAxis2 = csvOut2[2][2:]
angAxis2 = 180-np.round(np.degrees(np.array([float(convertEU_US(angAxis2[i])) for i in range(len(angAxis2))])),1)
angU2 = np.unique(angAxis2)[::-1]
print(angAxis2)

velAxis2 = csvOut2[1][2:]
velAxis2 = np.round(-1*np.array(velAxis2,dtype=float))
velU2 = np.unique(velAxis2)[::-1]


normalForceDat2 = csvOut2[7][2:]
normalForceDat2 = np.array([float(convertEU_US(normalForceDat2[i])) for i in range(len(normalForceDat2))])

torqueDat2 = normalForceDat2 * np.cos(np.radians(angAxis2)) * 0.13 # meters

torquePlot2 = np.reshape(torqueDat2,(len(angU2),len(velU2)))
normalForcePlot2 = np.reshape(normalForceDat2,(len(angU2),len(velU2)))

torqueServoDat2 = normalForceDat2 * 0.016 # meters
torqueServoPlot2 = np.reshape(torqueServoDat2,(len(angU2),len(velU2)))


# ---


from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['font.sans-serif'] = ['Computer Modern']
rcParams['text.usetex'] = True

fig,ax = plt.subplots()

#plt.scatter(velAxis,angAxis,c=normalForceDat,cmap='jet',marker='s',s=1000)
#plt.pcolormesh(velU,angU,torquePlot,cmap='jet',shading='nearest')
plt.pcolormesh(velU,angU,torqueServoPlot,cmap='jet',shading='nearest')
plt.scatter(velAxis2,angAxis2,c=torqueServoPlot2,cmap='jet',s=100)#,shading='nearest')
#plt.pcolormesh(velU,angU,normalForcePlot,cmap='jet',shading='nearest')
plt.xlabel("Freestream Vel. (m/s)")
plt.ylabel("Ctrl. Surface Angle (deg)")
#plt.colorbar(label='Torque on Rocket ($\\rm N\\cdot m$)')
plt.colorbar(label='Torque on Servo ($\\rm N\\cdot m$)')
#plt.colorbar(label='Normal Force on Surface (N)')
plt.show()



exit(0)
#angAxis = csvOut[]

