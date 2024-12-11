# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 16:16:01 2023

@author: tu_sc
"""

# ---------- GenerateMeshes ----------
LadoCubo = 23 #mm
AlturaCilindro = 18
RadioCilindro = 10

# ---------- SPC ---------- 

Density = 20
LevelHeight = AlturaCilindro-2 #13.5
Repeat = 3

Diff = (AlturaCilindro - LevelHeight)/2
H = (LadoCubo-AlturaCilindro)/2 + Diff
