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

    corners = [trim.PointAtStart for trim in trims]
    corner_count = len(corners)
    if corner_count < 3:
        return None

    candidates = []

    for i in range(corner_count):
        prev_corner = corners[i - 1]
        corner = corners[i]
        next_corner = corners[(i + 1) % corner_count]

        incoming_raw = prev_corner - corner
        outgoing_raw = next_corner - corner

        incoming_len = incoming_raw.Length
        outgoing_len = outgoing_raw.Length
        if incoming_len <= tol or outgoing_len <= tol:
            continue

        vec_prev = project_and_unitize(incoming_raw, normal)
        vec_curr = project_and_unitize(outgoing_raw, normal)
        if not vec_prev or not vec_curr:
            continue

        angle_rad = Rhino.Geometry.Vector3d.VectorAngle(vec_prev, vec_curr)
        if angle_rad < 0:
            continue

        angle_deg = math.degrees(angle_rad)
        if 89.0 <= angle_deg <= 91.0:
            if incoming_len >= outgoing_len:
                x_axis = vec_prev
                x_len = incoming_len
            else:
                x_axis = vec_curr
                x_len = outgoing_len

            candidates.append({
                "deviation": abs(angle_deg - 90.0),
                "x_length": x_len,
                "origin": corner,
                "x_axis": x_axis,
            })

    if not candidates:
        return None

    candidates.sort(key=lambda c: (-c["x_length"], c["deviation"]))
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
    loop = face.OuterLoop
    if not loop:
        return None

    normal = Rhino.Geometry.Vector3d(base_plane.ZAxis)
    if not normal.Unitize():
        return None

    longest_edge = None
    longest_length = -1.0
    for trim in loop.Trims:
        edge = trim.Edge if trim else None
        if not edge:
            continue

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


def get_local_cplane(face):
    if not face:
        return None

    base_plane = get_face_plane(face)
    if not base_plane:
        return None

    plane = get_preferred_corner_plane(face, base_plane)
    if plane:
        return plane

    return get_longest_edge_plane(face, base_plane)


def get_face_bbox_points(face, plane):
    tol = sc.doc.ModelAbsoluteTolerance
    loop = face.OuterLoop
    if not loop:
        return None

    sample_points = []
    for trim in loop.Trims:
        edge = trim.Edge if trim else None
        if not edge:
            continue

        sample_points.append(edge.PointAtStart)
        sample_points.append(edge.PointAtEnd)

        ok_mid, mid_t = edge.NormalizedLengthParameter(0.5)
        if ok_mid:
            sample_points.append(edge.PointAt(mid_t))

    if not sample_points:
        return None

    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")

    for pt in sample_points:
        ok, x, y = plane.ClosestParameter(pt)
        if not ok:
            continue

        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y

    if min_x == float("inf") or min_y == float("inf"):
        return None

    if (max_x - min_x) <= tol or (max_y - min_y) <= tol:
        return None

    pt00 = plane.PointAt(min_x, min_y)
    pt10 = plane.PointAt(max_x, min_y)
    pt01 = plane.PointAt(min_x, max_y)

    return pt00, pt10, pt01


def add_inside_dimensions(face, plane):
    bbox_pts = get_face_bbox_points(face, plane)
    if not bbox_pts:
        return False

    tol = sc.doc.ModelAbsoluteTolerance

    pt1_x, pt2_x, pt2_y = bbox_pts
    pt1_y = pt1_x

    x_len = pt1_x.DistanceTo(pt2_x)
    y_len = pt1_y.DistanceTo(pt2_y)

    if x_len <= tol or y_len <= tol:
        return False

    # Ölçü çizgileri yüzey sınırından, kenar uzunluğunun 1/3'ü kadar içeride olsun.
    x_offset = y_len / 3.0
    y_offset = x_len / 3.0

    mid_x = midpoint(pt1_x, pt2_x)
    mid_y = midpoint(pt1_y, pt2_y)

    dim_point_x = mid_x + (plane.YAxis * x_offset)
    dim_point_y = mid_y + (plane.XAxis * y_offset)

    dim1 = rs.AddAlignedDimension(pt1_x, pt2_x, dim_point_x)
    dim2 = rs.AddAlignedDimension(pt1_y, pt2_y, dim_point_y)

    return bool(dim1 or dim2)


def ensure_automatic_dim_layer():
    layer_name = "Automatic DIM"
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def get_selected_faces():
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Boyutlandırmak istediğiniz yüzeyleri seçin (polysurface için yüzeye tıklayın)")
    go.GeometryFilter = Rhino.DocObjects.ObjectType.Surface | Rhino.DocObjects.ObjectType.PolysrfFilter
    go.SubObjectSelect = True
    go.EnablePreSelect(True, True)
    go.GetMultiple(1, 0)

    if go.CommandResult() != Rhino.Commands.Result.Success:
        return []

    selected = []
    for i in range(go.ObjectCount):
        obj_ref = go.Object(i)
        if not obj_ref:
            continue

        face = obj_ref.Face()
        if face:
            selected.append(face)
            continue

        brep = obj_ref.Brep()
        if brep and brep.Faces.Count == 1:
            selected.append(brep.Faces[0])

    return selected


def YuzeyBoyutlandirYerelCPlane():
    secilen_yuzeyler = get_selected_faces()
    if not secilen_yuzeyler:
        return

    view = rs.CurrentView()
    old_cplane = rs.ViewCPlane(view)
    old_layer = rs.CurrentLayer()
    dim_layer = ensure_automatic_dim_layer()
    basarili = 0
    atlanan = []

    rs.EnableRedraw(False)
    try:
        rs.CurrentLayer(dim_layer)
        for face in secilen_yuzeyler:
            local_plane = get_local_cplane(face)
            if not local_plane:
                atlanan.append("seçili yüzey")
                continue

            rs.ViewCPlane(view, local_plane)

            if add_inside_dimensions(face, local_plane):
                basarili += 1
            else:
                atlanan.append("seçili yüzey")

    finally:
        rs.ViewCPlane(view, old_cplane)
        rs.CurrentLayer(old_layer)
        rs.EnableRedraw(True)

    if basarili:
        print("{} yüzey boyutlandırıldı.".format(basarili))
    if atlanan:
        print("{} yüzey atlandı. Genelde sebep: düzlemsel olmayan, tekilliği olan veya problemli sınır yapısı.".format(len(atlanan)))


if __name__ == "__main__":
    YuzeyBoyutlandirYerelCPlane()
