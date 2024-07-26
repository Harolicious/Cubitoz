# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 15:37:12 2023

@author: tu_sc
"""


import Constants
import gmsh
import numpy as np

gmsh.initialize()

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro
BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2, LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
gmsh.model.occ.rotate([DimTagCylinder], 0, (LadoCubo)/2, 0, 0, 0, 1, -np.pi/4)

CutOut = gmsh.model.occ.cut([DimTagBox],[DimTagCylinder])

gmsh.model.occ.synchronize()

gmsh.model.mesh.generate(3)
gmsh.model.mesh.refine()
gmsh.write("CubitoShear.vtk")
gmsh.fltk.run()

gmsh.clear()


CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)
gmsh.model.occ.rotate([DimTagCylinder], 0, (LadoCubo)/2, 0, 0, 0, 1, -np.pi/4)
gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()
gmsh.write("CubitoShear_Cavity.stl")
gmsh.fltk.run()
