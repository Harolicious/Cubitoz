#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 20 18:08:37 2024

@author: lab
"""

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('end_effector_data_ESS.csv')

Time = df['Time'].values
Pressure1 = df['Pressure_1'].values
Pressure2 = df['Pressure_2'].values
Pressure3 = df['Pressure_3'].values
x = df['Position_X'].values
y = df['Position_Y'].values  # y -> z
z = df['Position_Z'].values  # z -> y

x_ajust= x - x[0]
y_ajust= y - y[0]
z_ajust= -(z - z[0])
Pressure1_PSI = Pressure1/6.89
Pressure2_PSI = Pressure2/6.89
Pressure3_PSI = Pressure3/6.89

Pressure_all = Pressure1_PSI+Pressure2_PSI+Pressure3_PSI


plt.figure(figsize=(10, 6))
plt.plot(Pressure1_PSI, y_ajust, marker='o')
plt.plot(Pressure2_PSI, y_ajust, marker='o')
plt.plot(Pressure3_PSI, y_ajust, marker='o')
plt.legend(['Uniaxial module 1','Shear module 2', 'Shear module 3'])

plt.ylabel('Axis Z (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Variación de la altura en relación a la presión')

plt.savefig('despla_Z_ESS.png', dpi=300)

plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(Pressure1_PSI, x_ajust, marker='o')
plt.plot(Pressure2_PSI, x_ajust, marker='o')
plt.plot(Pressure3_PSI, x_ajust, marker='o')


plt.ylabel('Axis X (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Desplazamiento en X en relación a la presión')

plt.savefig('despla_X_ESS.png', dpi=300)

plt.grid(True)
plt.show()


plt.figure(figsize=(10, 6))
plt.plot(Pressure1_PSI, z_ajust, marker='o')
plt.plot(Pressure2_PSI, z_ajust, marker='o')
plt.plot(Pressure3_PSI, z_ajust, marker='o')


plt.ylabel('Axis Y (mm)')
plt.xlabel('Pressure (PSI)')
plt.title('Desplazamiento en Y en relación a la presión')

plt.savefig('despla_Y_ESS.png', dpi=300)

plt.grid(True)
plt.show()

######################################33



plt.figure(figsize=(10, 6))
plt.plot(z_ajust, x_ajust, marker='o')

plt.xlabel('Axis Y (mm)')
plt.ylabel('Axis X (mm)')
plt.title('Movimiento en X e Y')

plt.savefig('despla_Y_X_ESS.png', dpi=300)

plt.grid(True)
plt.show()


###


plt.figure(figsize=(10, 6))
plt.plot(x_ajust, y_ajust, marker='o')

plt.xlabel('Axis X (mm)')
plt.ylabel('Axis Z (mm)')
plt.title('Movimiento en X y Z')

plt.savefig('despla_X_Z_ESS.png', dpi=300)

plt.grid(True)
plt.show()


###


plt.figure(figsize=(10, 6))
plt.plot(z_ajust, y_ajust, marker='o')

plt.xlabel('Axis Y (mm)')
plt.ylabel('Axis Z (mm)')
plt.title('Movimiento en Y y Z')

plt.savefig('despla_Y_Z_ESS.png', dpi=300)

plt.grid(True)
plt.show()