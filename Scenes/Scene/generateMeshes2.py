# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 15:37:12 2023

@author: tu_sc
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 14:02:25 2023

@author: stefan
"""

import Constants
import gmsh

gmsh.initialize()

LadoCubo = Constants.LadoCubo
AlturaCilindro = Constants.AlturaCilindro
RadioCilindro = Constants.RadioCilindro
BoxTag = gmsh.model.occ.addBox(-LadoCubo/2,0,-LadoCubo/2, LadoCubo, LadoCubo, LadoCubo)
DimTagBox = (3, BoxTag)
CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro)
DimTagCylinder = (3, CylinderTag)

CutOut = gmsh.model.occ.cut([DimTagBox], [DimTagCylinder])

gmsh.model.occ.synchronize()

gmsh.model.mesh.generate(3)
gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar.vtk")
gmsh.fltk.run()

gmsh.clear()

CylinderTag = gmsh.model.occ.addCylinder(0, (LadoCubo-AlturaCilindro)/2,0, 0, AlturaCilindro,0, RadioCilindro)
gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)
gmsh.model.mesh.refine()
# gmsh.model.mesh.refine()
gmsh.write("CubitoEstirar_Cavity.stl")
gmsh.fltk.run()
