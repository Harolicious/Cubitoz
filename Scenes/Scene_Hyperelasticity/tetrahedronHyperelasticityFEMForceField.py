#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct  1 18:53:10 2024

@author: lab
"""

def createScene(root_node):

   root = root_node.addChild('root', dt="0.005", gravity="0 -9 0")

   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm")
   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection")
   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact")
   root.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective")
   root.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select")
   root.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative")
   root.addObject('RequiredPlugin', name="Sofa.Component.Mass")
   root.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
   root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic")
   root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.HyperElastic")
   root.addObject('RequiredPlugin', name="Sofa.Component.StateContainer")
   root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Dynamic")
   root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Grid")
   root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Mapping")
   root.addObject('RequiredPlugin', name="Sofa.Component.Visual")
   root.addObject('VisualStyle', displayFlags="showForceFields showBehaviorModels")
   root.addObject('CollisionPipeline', verbose="0")
   root.addObject('BruteForceBroadPhase', )
   root.addObject('BVHNarrowPhase', )
   root.addObject('CollisionResponse', response="PenalityContactForceField")
   root.addObject('MinProximityIntersection', name="Proximity", alarmDistance="0.8", contactDistance="0.5")
   root.addObject('DefaultAnimationLoop', )

   corrotational = root.addChild('Corrotational')

   corrotational.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false")
   corrotational.addObject('CGLinearSolver', iterations="25", name="linear solver", tolerance="1.0e-9", threshold="1.0e-9")
   corrotational.addObject('RegularGridTopology', name="hexaGrid", min="0 0 0", max="1 1 2.7", n="3 3 8", p0="0 0 0")
   corrotational.addObject('MechanicalObject', name="mechObj")
   corrotational.addObject('MeshMatrixMass', totalMass="1.0")
   corrotational.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="10000", poissonRatio="0.45", method="large")
   corrotational.addObject('BoxROI', drawBoxes="0", box="0 0 0 1 1 0.05", name="box")
   corrotational.addObject('FixedConstraint', indices="@box.indices")
   corrotational.addObject('Visual3DText', text="Corrotational", position="1 0 -0.5", scale="0.2")


   MooneyRivlin = root.addChild('MooneyRivlin')

   MooneyRivlin.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false")
   MooneyRivlin.addObject('CGLinearSolver', iterations="25", name="linear solver", tolerance="1.0e-9", threshold="1.0e-9")
   MooneyRivlin.addObject('RegularGridTopology', name="hexaGrid", min="0 0 0", max="1 1 2.7", n="3 3 8", p0="8 0 0")
   MooneyRivlin.addObject('MechanicalObject', name="mechObj")
   MooneyRivlin.addObject('MeshMatrixMass', totalMass="1.0")

   tetras = MooneyRivlin.addChild('tetras')

   tetras.addObject('TetrahedronSetTopologyContainer', name="Container")
   tetras.addObject('TetrahedronSetTopologyModifier', name="Modifier")
   tetras.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3", name="GeomAlgo")
   tetras.addObject('Hexa2TetraTopologicalMapping', name="default28", input="@../", output="@Container", printLog="0")
   tetras.addObject('TetrahedronHyperelasticityFEMForceField', name="FEM", ParameterSet="5000 7000 10", materialName="MooneyRivlin")

   MooneyRivlin.addObject('BoxROI', drawBoxes="1", box="8 0 0 9 1 0.05", name="box")
   MooneyRivlin.addObject('FixedConstraint', indices="@box.indices")
   MooneyRivlin.addObject('Visual3DText', text="MooneyRivlin", position="9 0 -0.5", scale="0.2")