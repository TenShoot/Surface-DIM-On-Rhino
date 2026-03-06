# -*- coding: utf-8 -*-
import math
import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino


def midpoint(pt_a, pt_b):
    return Rhino.Geometry.Point3d(
        (pt_a.X + pt_b.X) * 0.5,
        (pt_a.Y + pt_b.Y) * 0.5,
        (pt_a.Z + pt_b.Z) * 0.5,
    )


def get_face_plane(face):
    tol = sc.doc.ModelAbsoluteTolerance
    ok, plane = face.TryGetPlane(tol)
    if ok:
        return plane

    udom = face.Domain(0)
    vdom = face.Domain(1)
    u = (udom.T0 + udom.T1) * 0.5
    v = (vdom.T0 + vdom.T1) * 0.5
    ok, frame = face.FrameAt(u, v)
    if ok:
        return frame

    return None


def project_and_unitize(vec, normal):
    v = Rhino.Geometry.Vector3d(vec)
    dot = Rhino.Geometry.Vector3d.Multiply(v, normal)
    v = v - (normal * dot)
    if not v.Unitize():
        return None
    return v


def tangent_from_corner(edge, corner_pt, normal, tol):
    if not edge:
        return None

    start_pt = edge.PointAtStart
    end_pt = edge.PointAtEnd

    if corner_pt.DistanceTo(start_pt) <= tol:
        t = edge.Domain.T0
        vec = edge.TangentAt(t)
        if not vec.Unitize():
            vec = end_pt - start_pt
    elif corner_pt.DistanceTo(end_pt) <= tol:
        t = edge.Domain.T1
        vec = -edge.TangentAt(t)
        if not vec.Unitize():
            vec = start_pt - end_pt
    else:
        return None

    return project_and_unitize(vec, normal)


def get_preferred_corner_plane(face, base_plane):
    tol = sc.doc.ModelAbsoluteTolerance
    loop = face.OuterLoop
    if not loop:
        return None

    normal = Rhino.Geometry.Vector3d(base_plane.ZAxis)
    if not normal.Unitize():
        return None

    trims = [trim for trim in loop.Trims if trim and trim.Edge]
    if len(trims) < 2:
        return None

    candidates = []

    for i, curr_trim in enumerate(trims):
        prev_trim = trims[i - 1]
        prev_edge = prev_trim.Edge
        curr_edge = curr_trim.Edge
        if not prev_edge or not curr_edge:
            continue

        corner = curr_trim.PointAtStart
        vec_prev = tangent_from_corner(prev_edge, corner, normal, tol)
        vec_curr = tangent_from_corner(curr_edge, corner, normal, tol)
        if not vec_prev or not vec_curr:
            continue

        angle_rad = Rhino.Geometry.Vector3d.VectorAngle(vec_prev, vec_curr)
        if angle_rad < 0:
            continue

        angle_deg = math.degrees(angle_rad)
        if 89.0 <= angle_deg <= 91.0:
            prev_len = prev_edge.GetLength()
            curr_len = curr_edge.GetLength()
            if prev_len >= curr_len:
                x_axis = vec_prev
                x_len = prev_len
            else:
                x_axis = vec_curr
                x_len = curr_len

            candidates.append({
                "deviation": abs(angle_deg - 90.0),
                "x_length": x_len,
                "origin": corner,
                "x_axis": x_axis,
            })

    if not candidates:
        return None

    candidates.sort(key=lambda c: (c["deviation"], -c["x_length"]))
    best = candidates[0]

    x_axis = Rhino.Geometry.Vector3d(best["x_axis"])
    if not x_axis.Unitize():
        return None

    y_axis = Rhino.Geometry.Vector3d.CrossProduct(normal, x_axis)
    if not y_axis.Unitize():
        return None

    return Rhino.Geometry.Plane(best["origin"], x_axis, y_axis)


def get_longest_edge_plane(face, base_plane):
    tol = sc.doc.ModelAbsoluteTolerance
    brep = face.Brep
    if not brep or brep.Edges.Count == 0:
        return None

    normal = Rhino.Geometry.Vector3d(base_plane.ZAxis)
    if not normal.Unitize():
        return None

    longest_edge = None
    longest_length = -1.0
    for edge in brep.Edges:
        edge_len = edge.GetLength()
        if edge_len > longest_length:
            longest_length = edge_len
            longest_edge = edge

    if not longest_edge or longest_length <= tol:
        return None

    origin = longest_edge.PointAtStart
    x_axis = tangent_from_corner(longest_edge, origin, normal, tol)
    if not x_axis:
        raw = longest_edge.PointAtEnd - longest_edge.PointAtStart
        x_axis = project_and_unitize(raw, normal)
        if not x_axis:
            return None

    y_axis = Rhino.Geometry.Vector3d.CrossProduct(normal, x_axis)
    if not y_axis.Unitize():
        return None

    return Rhino.Geometry.Plane(origin, x_axis, y_axis)


def get_local_cplane(obj_id):
    brep = rs.coercebrep(obj_id)
    if not brep or brep.Faces.Count == 0:
        return None

    face = brep.Faces[0]
    base_plane = get_face_plane(face)
    if not base_plane:
        return None

    plane = get_preferred_corner_plane(face, base_plane)
    if plane:
        return plane

    return get_longest_edge_plane(face, base_plane)


def add_inside_dimensions(obj_id, plane):
    bbox = rs.BoundingBox(obj_id, plane)
    if not bbox or len(bbox) != 8:
        return False

    tol = sc.doc.ModelAbsoluteTolerance

    pt1_x = bbox[0]
    pt2_x = bbox[1]
    pt1_y = bbox[0]
    pt2_y = bbox[3]

    x_len = pt1_x.DistanceTo(pt2_x)
    y_len = pt1_y.DistanceTo(pt2_y)

    if x_len <= tol or y_len <= tol:
        return False

    x_offset = y_len * 0.25
    y_offset = x_len * 0.25

    mid_x = midpoint(pt1_x, pt2_x)
    mid_y = midpoint(pt1_y, pt2_y)

    dim_point_x = mid_x + (plane.YAxis * x_offset)
    dim_point_y = mid_y + (plane.XAxis * y_offset)

    dim1 = rs.AddAlignedDimension(pt1_x, pt2_x, dim_point_x)
    dim2 = rs.AddAlignedDimension(pt1_y, pt2_y, dim_point_y)

    return bool(dim1 or dim2)


def YuzeyBoyutlandirYerelCPlane():
    yuzeyler = rs.GetObjects(
        "Boyutlandırmak istediğiniz yüzeyleri seçin",
        rs.filter.surface,
        preselect=True,
    )
    if not yuzeyler:
        return

    view = rs.CurrentView()
    old_cplane = rs.ViewCPlane(view)
    basarili = 0
    atlanan = []

    rs.EnableRedraw(False)
    try:
        for yuzey in yuzeyler:
            local_plane = get_local_cplane(yuzey)
            if not local_plane:
                atlanan.append(str(yuzey))
                continue

            rs.ViewCPlane(view, local_plane)

            if add_inside_dimensions(yuzey, local_plane):
                basarili += 1
            else:
                atlanan.append(str(yuzey))

    finally:
        rs.ViewCPlane(view, old_cplane)
        rs.EnableRedraw(True)

    if basarili:
        print("{} yüzey boyutlandırıldı.".format(basarili))
    if atlanan:
        print("{} yüzey atlandı. Genelde sebep: düzlemsel olmayan, tekilliği olan veya problemli sınır yapısı.".format(len(atlanan)))


if __name__ == "__main__":
    YuzeyBoyutlandirYerelCPlane()
