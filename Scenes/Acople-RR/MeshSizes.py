#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 27 17:58:44 2024

@author: harold
"""

import gmsh

def MeshSizes(lc=0.5):   
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
    