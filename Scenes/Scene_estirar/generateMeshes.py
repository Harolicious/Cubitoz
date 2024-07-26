#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023


@author: tu_sc
"""

import Constants
import numpy as np
import gmsh

gmsh.initialize()

def defineMeshSizes(lc=0.5):   
    #-------------------
    # MeshSizes 
    #-------------------

    gmsh.model.mesh.field.add("Box", 1)
    gmsh.model.mesh.field.setNumber(1, "VIn", lc)
    gmsh.model.mesh.field.setNumber(1, "VOut", lc)
    gmsh.model.mesh.field.setNumber(1, "XMin", -100)
    gmsh.model.mesh.field.setNumber(1, "XMax", 100)
    gmsh.model.mesh.field.setNumber(1, "YMin", -100)
    gmsh.model.mesh.field.setNumber(1, "YMax", 100)
    gmsh.model.mesh.field.setNumber(1, "ZMin", -100)
    gmsh.model.mesh.field.setNumber(1, "ZMax", 100)    
    gmsh.model.mesh.field.setNumber(1, "Thickness", 0.3)
     
    gmsh.model.mesh.field.setAsBackgroundMesh(1)
    
    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro

BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2,LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0,0, AlturaCilindro, 0 , RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

Cutout = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])

gmsh.model.occ.synchronize()

defineMeshSizes(1.6)
gmsh.model.mesh.generate(3)
# gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar.vtk")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro, angle= 2*np.pi)
DimTagCylinder = (3, CylinderTag)

gmsh.model.occ.synchronize()

defineMeshSizes(1)
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
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

