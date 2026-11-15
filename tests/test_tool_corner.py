# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2026 Foad S. Farimani
# SPDX-FileNotice: Part of the DFM addon.

"""Tool corner radius (CNC milling): inside corners along the tool axis against the end mill's radius.

A 100 x 60 x 20 block with a 40 x 30 x 10 pocket from the top: sharp corners, then rounded R2 and R4; a
through hole, rounded outside edges; and the same pocket seen along X instead of Z.
"""

import unittest

import dfm_test_support  # noqa: F401  (repository and FreeCAD stand-ins on sys.path first)

from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.GeomAbs import GeomAbs_Line
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

from freecad.DFM.core.analyzers.tool_corner_analyzer import ToolCornerAnalyzer
from freecad.DFM.core.checks.tool_corner_check import ToolCornerRadiusCheck
from freecad.DFM.core.models import ProcessRequirement, Severity
from freecad.DFM.core.processes.process import ProcessArchetype, RuleLimit
from freecad.DFM.core.registries.process_registry import ProcessRegistry
from freecad.DFM.core.rules import Rulebook
from freecad.DFM.core.utils.geometry import EdgeIndex, FaceIndex

Z, X = gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)


def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def cut(a, b):
    op = BRepAlgoAPI_Cut(a, b)
    op.Build()
    return op.Shape()


def round_edges_along(shape, axis, radius):
    """shape with every straight edge along axis rounded."""
    fillet = BRepFilletAPI_MakeFillet(shape)
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        curve = BRepAdaptor_Curve(edge)
        if curve.GetType() == GeomAbs_Line and curve.Line().Direction().IsParallel(axis, 1e-6):
            fillet.Add(radius, edge)
        exp.Next()
    fillet.Build()
    return fillet.Shape()


def pocketed(corner_radius=0.0):
    tool = box(30, 15, 10, 70, 45, 25)
    if corner_radius:
        tool = round_edges_along(tool, Z, corner_radius)
    return cut(box(0, 0, 0, 100, 60, 20), tool)


def corners(shape, axis=Z):
    analyzer = ToolCornerAnalyzer()
    return analyzer.execute(shape, FaceIndex(shape), EdgeIndex(shape), **{ProcessRequirement.TOOL_AXIS.name: axis})


def check(data, target="3.0", limit="0.5"):
    return ToolCornerRadiusCheck().run_check(data, RuleLimit(target=target, limit=limit),
                                             Rulebook.MIN_TOOL_CORNER_RADIUS)


class TestToolCorner(unittest.TestCase):
    def test_sharp_pocket_corners_are_radius_zero_errors(self):
        found = check(corners(pocketed()))
        self.assertEqual(len(found), 4)
        self.assertTrue(all(r.severity == Severity.ERROR and r.value == 0.0 for r in found))
        self.assertTrue(all(r.failing_geometry[0][0] == "Edge" for r in found))

    def test_rounded_corners_are_measured_once_each(self):
        data = corners(pocketed(2.0))
        found = check(data)
        self.assertEqual(len(found), 4)               # each fillet borders two edges: reported once
        self.assertTrue(all(r.severity == Severity.WARNING for r in found))
        self.assertTrue(all(abs(r.value - 2.0) < 1e-6 for r in found))
        self.assertTrue(all(r.failing_geometry[0][0] == "Face" for r in found))
        self.assertEqual(len(check(data, target="3.0", limit="2.5")), 4)
        self.assertTrue(all(r.severity == Severity.ERROR for r in check(data, target="3.0", limit="2.5")))

    def test_corners_at_least_the_target_pass(self):
        self.assertEqual(check(corners(pocketed(4.0))), [])

    def test_a_hole_and_rounded_outside_edges_are_not_corners(self):
        hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(15, 15, -1), Z), 2.0, 30).Shape()
        part = round_edges_along(cut(box(0, 0, 0, 100, 60, 20), hole), Z, 1.0)
        self.assertEqual(check(corners(part)), [])

    def test_the_tool_axis_decides_which_corners_count(self):
        # along X, the pocket's vertical corners are not cut by the tool's side; the two floor edges along X are
        found = check(corners(pocketed(), axis=X))
        self.assertEqual(len(found), 2)
        self.assertTrue(all(r.value == 0.0 for r in found))

    def test_the_cnc_milling_process(self):
        process = ProcessRegistry.get_instance().get_process_by_name("CNC Milling (3-axis)")
        self.assertIsNotNone(process)
        self.assertEqual(process.archetype, ProcessArchetype.MACHINING)
        self.assertEqual(process.active_rules, [Rulebook.MIN_TOOL_CORNER_RADIUS, Rulebook.MIN_WALL_THICKNESS])
        self.assertEqual(ToolCornerAnalyzer().requirements, {ProcessRequirement.TOOL_AXIS})
        self.assertIn("Aluminium 6061", process.materials)


if __name__ == "__main__":
    unittest.main()
