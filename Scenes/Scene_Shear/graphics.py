#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Shear.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x = df['Position_X'].values
y = df['Position_Y'].values
z = df['Position_Z'].values

x_ajust= x - x[0]
y_ajust= y - y[0]
Pressure_PSI = Pressure/6.89


plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, y_ajust, marker='o')

plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Variación de la altura en relación a la presión')

plt.savefig('despla_Z_presion.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, x_ajust, marker='o')

plt.ylabel('Axis X (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Desplazamiento en X en relación a la presión')

plt.savefig('despla_Y_presion.png', dpi=300)

plt.grid(True)
plt.show()