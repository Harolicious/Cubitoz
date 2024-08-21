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


plt.figure(figsize=(10, 6))
plt.plot(y, Pressure, marker='o')

plt.xlabel('Eje Z')
plt.ylabel('Pressure')
plt.title('Relación entre el eje Z y la presión')

plt.savefig('despla_Z_presion.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(x, Pressure, marker='o')

plt.xlabel('Eje Y')
plt.ylabel('Pressure')
plt.title('Relación entre el eje Y y la presión')

plt.savefig('despla_Y_presion.png', dpi=300)

plt.grid(True)
plt.show()