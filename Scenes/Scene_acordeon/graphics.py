#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 21 17:38:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Acordeon.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x = df['Position_X'].values
y = df['Position_Y'].values
z = df['Position_Z'].values
Angle = df['Angle'].values

y_adjusted = y - y[0]
Pressure_PSI = Pressure/6.89
Neg_Angle = -Angle

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, y_adjusted , marker='o')

plt.ylabel('Eje Z')
plt.xlabel('Pressure')
plt.title('Relación entre el eje Z y la presión')

plt.savefig('despla_Z_presion.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, Neg_Angle, marker='o')

plt.ylabel('Angle')
plt.xlabel('Pressure')
plt.title('Inclinacion del módulo')

plt.savefig('Angle_presion.png', dpi=300)

plt.grid(True)
plt.show()