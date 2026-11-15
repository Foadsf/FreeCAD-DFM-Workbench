# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2026 Foad S. Farimani
# SPDX-FileNotice: Part of the DFM addon.

"""Inside corners along a milling tool's axis, and the radius they leave room for.

An end mill cuts a wall parallel to its axis and leaves its own radius in every
inside corner between two such walls. So, per straight edge parallel to the
tool axis:

* a sharp inside corner (two faces meeting at an angle, concave) is a corner
  of radius 0: no end mill can cut it;
* the edge of a concave cylinder round the tool axis that is less than a full
  circle (a corner fillet, a slot end) is a corner of that cylinder's radius;
* a full cylinder is a hole, drilled or bored, not a corner: the seam of a
  whole cylinder, and the edges between the halves of a split one, are left
  out.

Each value is (radius in mm, "Edge" or "Face", the 1-based index of the sharp
edge or of the fillet face); a fillet is seen from both its edges, and the check
reports it once.
"""

import math
from typing import Any, Optional

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Line
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS, TopoDS_Edge, TopoDS_Face, TopoDS_Shape
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_IndexedMapOfShape
from OCP.gp import gp_Dir, gp_Vec

from ...core.base.base_analyzer import BaseAnalyzer
from ...core.models import ProcessRequirement
from ...core.registries.analyzers_registry import register_analyzer
from ...core.utils.geometry import EdgeIndex, FaceIndex
from .sharp_corners import SharpCornersAnalyzer

#: An edge within this angle of the tool axis runs along it (radians).
AXIS_TOL = 1e-3
#: Two faces whose normals differ by less than this meet tangentially, not in a corner (degrees).
CORNER_MIN_ANGLE = 1.0


@register_analyzer("TOOL_CORNER_ANALYZER")
class ToolCornerAnalyzer(BaseAnalyzer):
    relative_cost = 0.3  # ~ms per edge (see BaseAnalyzer.relative_cost)
    item_kind = "Edge"

    @property
    def name(self) -> str:
        return "Tool Corner Analyzer"

    @property
    def analysis_type(self) -> str:
        return "TOOL_CORNER_ANALYZER"

    @property
    def requirements(self) -> set[ProcessRequirement]:
        return {ProcessRequirement.TOOL_AXIS}

    def resolve_prefs(self, prefs: dict) -> None:
        pass

    def iteration_count(self, shape: TopoDS_Shape) -> int:
        edge_map = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, TopAbs_EDGE, edge_map)
        return edge_map.Extent()

    def prepare(self, shape: TopoDS_Shape, face_index: FaceIndex, edge_index: EdgeIndex, **kwargs: Any) -> None:
        super().prepare(shape, face_index, edge_index, **kwargs)
        self.axis = kwargs.get(ProcessRequirement.TOOL_AXIS.name) or gp_Dir(0, 0, 1)
        self.face_index = face_index
        self.edge_index = edge_index
        self.edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, self.edge_face_map)
        self._corners = SharpCornersAnalyzer()       # its concavity and normal helpers

    def analyze(self, edge: TopoDS_Edge) -> Optional[tuple[float, str, int]]:
        if BRep_Tool.Degenerated_s(edge) or not self.edge_face_map.Contains(edge):
            return None
        curve = BRepAdaptor_Curve(edge)
        if curve.GetType() != GeomAbs_Line:
            return None
        if curve.Line().Direction().Angle(self.axis) > AXIS_TOL and \
                math.pi - curve.Line().Direction().Angle(self.axis) > AXIS_TOL:
            return None
        faces = self.edge_face_map.FindFromKey(edge)
        if faces.Size() != 2:
            return None
        f1, f2 = TopoDS.Face_s(faces.First()), TopoDS.Face_s(faces.Last())
        if f1.IsSame(f2):                                   # the seam of a whole cylinder: a hole
            return None
        c1, c2 = self._cylinder(f1), self._cylinder(f2)
        if c1 and c2 and self._same_cylinder(c1, c2):       # between the halves of a split hole
            return None
        mid = (curve.FirstParameter() + curve.LastParameter()) / 2.0
        for face, cyl in ((f1, c1), (f2, c2)):
            if cyl and self._is_corner_fillet(face, cyl, edge, curve.Value(mid), mid):
                return cyl[2], "Face", self.face_index.index_of(face)
        n1 = self._corners._get_normal_at_edge(f1, edge, mid)
        n2 = self._corners._get_normal_at_edge(f2, edge, mid)
        if n1 is None or n2 is None or math.degrees(n1.Angle(n2)) < CORNER_MIN_ANGLE:
            return None
        if self._corners.is_concave_edge(edge, f1, f2):
            return 0.0, "Edge", self.edge_index.index_of(edge)
        return None

    def _cylinder(self, face: TopoDS_Face):
        """(axis direction, a point on the axis, radius) of a cylindrical face round the tool axis, or None."""
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() != GeomAbs_Cylinder:
            return None
        cyl = surf.Cylinder()
        d = cyl.Axis().Direction()
        if d.Angle(self.axis) > AXIS_TOL and math.pi - d.Angle(self.axis) > AXIS_TOL:
            return None
        return d, cyl.Axis().Location(), cyl.Radius()

    @staticmethod
    def _same_cylinder(a, b) -> bool:
        (da, pa, ra), (db, pb, rb) = a, b
        if abs(ra - rb) > 1e-6:
            return False
        offset = gp_Vec(pa, pb)
        along = offset.Dot(gp_Vec(da))
        return math.sqrt(max(0.0, offset.SquareMagnitude() - along * along)) < 1e-6

    def _is_corner_fillet(self, face: TopoDS_Face, cyl, edge: TopoDS_Edge, point, param: float) -> bool:
        """A concave cylinder (the material outside it) that is less than a full circle."""
        d, loc, _r = cyl
        normal = self._corners._get_normal_at_edge(face, edge, param)
        if normal is None:
            return False
        to_point = gp_Vec(loc, point)
        foot = gp_Vec(d).Multiplied(to_point.Dot(gp_Vec(d)))
        inward = foot.Subtracted(to_point)                  # from the point towards the axis
        if inward.Magnitude() < 1e-9 or gp_Vec(normal).Dot(inward) <= 0:
            return False                                    # convex: a rounded outside edge
        umin, umax, _vmin, _vmax = BRepTools.UVBounds_s(face)
        return umax - umin < 2 * math.pi - 1e-6
