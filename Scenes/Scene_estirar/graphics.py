#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Estirar_hyper.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x1 = df['P1_Position_X'].values
y1 = df['P1_Position_Y'].values
z1 = df['P1_Position_Z'].values
x2 = df['P2_Position_X'].values
y2 = df['P2_Position_Y'].values
z2 = df['P2_Position_Z'].values

Time_v2= Time*20

y1_ad = y1 - y1[0]
z2_ad = z2 - z2[0]
Pressure_PSI = Pressure/6.89

mid = int(len(Time)/2)
y1_ad_up= y1_ad[:mid]
y1_ad_down = y1_ad[mid:]
z2_ad_up= z2_ad[:mid]
z2_ad_down = z2_ad[mid:]
Pressure_PSI_up = Pressure_PSI[:mid]
Pressure_PSI_down = Pressure_PSI[mid:]

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, y1_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, y1_ad_down , marker=',', color='green')
plt.legend(['Strain Z Inflated', 'Strain Z Deflated'])


plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre el desplazamiento en Z y la presión')
plt.grid(True)
plt.savefig('DesplaZ_presion_uniaxial.png', dpi=300)


plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, z2_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, z2_ad_down , marker=',', color='green')
plt.legend(['Strain X Inflated', 'Strain X Deflated'])


plt.ylabel('Axis X (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre el desplazamiento en X y la presión')
plt.grid(True)
plt.savefig('DesplaX_presion_uniaxial.png', dpi=300)


plt.show()


plt.figure(figsize=(10, 6))
plt.plot(Time, Pressure_PSI, marker=',')

plt.xlabel('Time (s)')
plt.ylabel('Pressure PSI')
plt.title('Presión a lo largo de la simulación')
plt.grid(True)
plt.savefig('Pressure_time_uniaxial.png', dpi=300)


plt.show()

# plt.figure(figsize=(10, 6))
# plt.plot(Time_v2, Pressure_PSI, marker=',')

# plt.xlabel('Time (s)')
# plt.ylabel('Pressure PSI')
# plt.title('Pressure throughout simulation')
# plt.grid(True)
# plt.savefig('Pressure_timev2_uniaxial.png', dpi=300)


# plt.show()