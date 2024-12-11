# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 16:16:01 2023

@author: tu_sc
"""
# ---------- GenerateMeshes ----------
LadoCubo = 20 #mm
AlturaCilindro = 17
RadioCilindro = 9

# ---------- SPC ---------- 
Density = 30
Density2 = 10
LevelHeight = AlturaCilindro-2 #13.5
Repeat = 14
Deg = 80

Diff = (AlturaCilindro - LevelHeight)/2
H = (LadoCubo-AlturaCilindro)/2 + Diff