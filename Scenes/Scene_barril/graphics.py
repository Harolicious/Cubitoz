#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_Barril.csv')

Time = df['Time'].values
Pressure = df['Pressure'].values
x1 = df['P1_Position_X'].values
y1 = df['P1_Position_Y'].values
z1 = df['P1_Position_Z'].values
x2 = df['P2_Position_X'].values
y2 = df['P2_Position_Y'].values
z2 = df['P2_Position_Z'].values

z2_ajust = z2 - z2[0] 
y1_ajust = y1 - y1[0] 
Pressure_PSI = Pressure/6.89

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, y1_ajust, marker='o')

plt.ylabel('Eje Z')
plt.xlabel('Pressure')
plt.title('Relación entre el eje Z y la presión')

plt.savefig('despla_Z_presion.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, z2_ajust, marker='o')

plt.ylabel('Eje X')
plt.xlabel('Pressure')
plt.title('Desplazamiento Biaxial')

plt.savefig('despla_X_presion.png', dpi=300)

plt.grid(True)
plt.show()