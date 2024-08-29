#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023


@author: lab_Harold
"""

import Constants
import numpy as np
import gmsh
from defineMeshSizes import defineMeshSizes

gmsh.initialize()

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro


BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

Cutout = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])

###plano embebido  en  mallado

p1 = gmsh.model.occ.addPoint(-LadoCubo/2 + 0.01, 1, LadoCubo/2 - 0.01)
p2 = gmsh.model.occ.addPoint(LadoCubo/2 - 0.01, 1, LadoCubo/2 - 0.01)
p3 = gmsh.model.occ.addPoint(LadoCubo/2 - 0.01, 1, -LadoCubo/2 + 0.01)
p4 = gmsh.model.occ.addPoint(-LadoCubo/2 + 0.01, 1, -LadoCubo/2 + 0.01)

l1 = gmsh.model.occ.addLine(p1, p2)
l2 = gmsh.model.occ.addLine(p2, p3)
l3 = gmsh.model.occ.addLine(p3, p4)
l4 = gmsh.model.occ.addLine(p4, p1)
loop1 = gmsh.model.occ.addCurveLoop([l1, l2, l3, l4])
plane1 = gmsh.model.occ.addPlaneSurface([loop1])

p5 = gmsh.model.occ.addPoint(-LadoCubo/2 + 0.01, LadoCubo-1, LadoCubo/2 - 0.01)
p6 = gmsh.model.occ.addPoint(LadoCubo/2 - 0.01, LadoCubo-1, LadoCubo/2 - 0.01)
p7 = gmsh.model.occ.addPoint(LadoCubo/2 - 0.01, LadoCubo-1, -LadoCubo/2 + 0.01)
p8 = gmsh.model.occ.addPoint(-LadoCubo/2 + 0.01, LadoCubo-1, -LadoCubo/2 + 0.01)

l5 = gmsh.model.occ.addLine(p5, p6)
l6 = gmsh.model.occ.addLine(p6, p7)
l7 = gmsh.model.occ.addLine(p7, p8)
l8 = gmsh.model.occ.addLine(p8, p5)
loop2 = gmsh.model.occ.addCurveLoop([l5, l6, l7, l8])
plane2 = gmsh.model.occ.addPlaneSurface([loop2])

gmsh.model.occ.synchronize()

gmsh.model.mesh.embed(2, [plane1,plane2], 3, BoxTag)

defineMeshSizes(1.6)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar.vtk")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()

defineMeshSizes(1.5)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar_Cavity.stl")
gmsh.fltk.run()

# Visual model

Box1Tag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox1 = (3, Box1Tag)


gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()

gmsh.write("Cubito_Estirar_visu.stl")

gmsh.clear()
gmsh.finalize()