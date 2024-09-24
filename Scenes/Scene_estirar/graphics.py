#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Estirar.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x = df['Position_X'].values
y = df['Position_Y'].values
z = df['Position_Z'].values

y_ad = y - y[0]
Pressure_PSI = Pressure/6.89

mid = int(len(Time)/2)
y_ad_up= y_ad[:mid]
y_ad_down = y_ad[mid:]
Pressure_PSI_up = Pressure_PSI[:mid]
Pressure_PSI_down = Pressure_PSI[mid:]

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI_up, y_ad_up , marker=',', color='blue')
plt.plot(Pressure_PSI_down, y_ad_down , marker=',', color='green')
plt.legend(['Strain Z Inflated', 'Strain Z Deflated'])


plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Relación entre el desplazamiento en Z y la presión')

plt.savefig('DesplaZ_presion_uniaxial.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Time, Pressure_PSI, marker=',')

plt.xlabel('Time (s)')
plt.ylabel('Pressure PSI')
plt.title('Presión a lo largo de la simulación')

plt.savefig('Pressure_time_uniaxial.png', dpi=300)

plt.grid(True)
plt.show()