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

y_adjusted = y - y[0]
Pressure_PSI = Pressure/6.89

plt.figure(figsize=(10, 6))
plt.plot(Pressure_PSI, y_adjusted, marker='o')

plt.ylabel('Axis Z mm')
plt.xlabel('Pressure PSI')
plt.title('Desplazamiento vs Presión')

plt.savefig('DesplaZ_presion_uniaxial.png', dpi=300)

plt.grid(True)
plt.show()