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

    # Modeldeki küçük sapmaları daha toleranslı yakalamak için 90° bandını geniş tutuyoruz.
    right_angle_min = 88.0
    right_angle_max = 92.0

    corners = [trim.PointAtStart for trim in trims]
    corner_count = len(corners)
    if corner_count < 3:
        return None

    candidates = []

    for i in range(corner_count):
        corner = corners[i]
        prev_corner = corners[i - 1]
        next_corner = corners[(i + 1) % corner_count]

        incoming_raw = prev_corner - corner
        outgoing_raw = next_corner - corner

        incoming_len = incoming_raw.Length
        outgoing_len = outgoing_raw.Length
        if incoming_len <= tol or outgoing_len <= tol:
            continue

        vec_in = project_and_unitize(incoming_raw, normal)
        vec_out = project_and_unitize(outgoing_raw, normal)
        if not vec_in or not vec_out:
            continue

        angle_rad = Rhino.Geometry.Vector3d.VectorAngle(vec_in, vec_out)
        if angle_rad < 0:
            continue

        angle_deg = math.degrees(angle_rad)
        if not (right_angle_min <= angle_deg <= right_angle_max):
            continue

        deviation = abs(angle_deg - 90.0)

        # Öncelik: 90° köşelere bağlı kenarlar arasındaki en uzun kenar X ekseni olsun.
        candidates.append({
            "deviation": deviation,
            "x_length": incoming_len,
            "origin": corner,
            "x_axis": vec_in,
            "corner_index": i,
        })
        candidates.append({
            "deviation": deviation,
            "x_length": outgoing_len,
            "origin": corner,
            "x_axis": vec_out,
            "corner_index": i,
        })

    if not candidates:
        return None

    # 90° köşelerde global en uzun kenarı seç; eşitlikte 90°ye daha yakın ve düşük indeksli köşe.
    candidates.sort(key=lambda c: (-c["x_length"], c["deviation"], c["corner_index"]))
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
        return None, False

    base_plane = get_face_plane(face)
    if not base_plane:
        return None, False

    plane = get_preferred_corner_plane(face, base_plane)
    if plane:
        return plane, True

    return get_longest_edge_plane(face, base_plane), False


def align_plane_x_to_longer_bbox(obj_id, plane):
    extent_pts = get_plane_aligned_extent_points(obj_id, plane)
    if not extent_pts:
        return plane

    tol = sc.doc.ModelAbsoluteTolerance
    pt1_x, pt2_x, pt2_y = extent_pts
    x_len = pt1_x.DistanceTo(pt2_x)
    y_len = pt1_x.DistanceTo(pt2_y)

    # X ekseni her zaman daha uzun boyu temsil etsin.
    if y_len > x_len + tol:
        x_axis = Rhino.Geometry.Vector3d(plane.YAxis)
        y_axis = Rhino.Geometry.Vector3d(-plane.XAxis)
        return Rhino.Geometry.Plane(plane.Origin, x_axis, y_axis)

    return plane


def get_plane_aligned_extent_points(obj_id, plane):
    brep = rs.coercebrep(obj_id)
    if not brep:
        return None

    to_world_xy = Rhino.Geometry.Transform.PlaneToPlane(
        plane,
        Rhino.Geometry.Plane.WorldXY,
    )
    from_world_xy = Rhino.Geometry.Transform.PlaneToPlane(
        Rhino.Geometry.Plane.WorldXY,
        plane,
    )

    sample_pts_xy = []
    for edge in brep.Edges:
        start_pt = Rhino.Geometry.Point3d(edge.PointAtStart)
        end_pt = Rhino.Geometry.Point3d(edge.PointAtEnd)
        start_pt.Transform(to_world_xy)
        end_pt.Transform(to_world_xy)
        sample_pts_xy.append(start_pt)
        sample_pts_xy.append(end_pt)

        ok_mid, mid_t = edge.NormalizedLengthParameter(0.5)
        if ok_mid:
            mid_pt = Rhino.Geometry.Point3d(edge.PointAt(mid_t))
            mid_pt.Transform(to_world_xy)
            sample_pts_xy.append(mid_pt)

    if not sample_pts_xy:
        return None

    min_x = min(pt.X for pt in sample_pts_xy)
    max_x = max(pt.X for pt in sample_pts_xy)
    min_y = min(pt.Y for pt in sample_pts_xy)
    max_y = max(pt.Y for pt in sample_pts_xy)

    pt1_x = Rhino.Geometry.Point3d(min_x, min_y, 0.0)
    pt2_x = Rhino.Geometry.Point3d(max_x, min_y, 0.0)
    pt2_y = Rhino.Geometry.Point3d(min_x, max_y, 0.0)

    pt1_x.Transform(from_world_xy)
    pt2_x.Transform(from_world_xy)
    pt2_y.Transform(from_world_xy)

    return pt1_x, pt2_x, pt2_y


def add_inside_dimensions(obj_id, plane):
    extent_pts = get_plane_aligned_extent_points(obj_id, plane)
    if not extent_pts:
        return False

    tol = sc.doc.ModelAbsoluteTolerance

    pt1_x, pt2_x, pt2_y = extent_pts
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


def create_temp_face_object(face):
    if not face:
        return None

    dup_brep = face.DuplicateFace(False)
    if not dup_brep:
        return None

    temp_id = sc.doc.Objects.AddBrep(dup_brep)
    if not temp_id:
        return None

    return temp_id


def ensure_automatic_dim_layer():
    layer_name = "Automatic DIM"
    if not rs.IsLayer(layer_name):
        rs.AddLayer(layer_name)
    return layer_name


def get_selected_faces():
    go = Rhino.Input.Custom.GetObject()
    go.SetCommandPrompt("Boyutlandırmak istediğiniz yüzeyleri seçin (polysurface için yüzeye tıklayın)")
    # Sadece yüz (face) seçimlerini kabul et: polysurface gövdesi toplu seçilmesin.
    go.GeometryFilter = Rhino.DocObjects.ObjectType.Surface
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
            temp_id = create_temp_face_object(face)
            if not temp_id:
                atlanan.append("seçili yüzey")
                continue

            try:
                temp_brep = rs.coercebrep(temp_id)
                temp_face = temp_brep.Faces[0] if temp_brep and temp_brep.Faces.Count else None
                local_plane, used_right_angle = get_local_cplane(temp_face)
                if not local_plane:
                    atlanan.append("seçili yüzey")
                    continue

                # 90° köşe ile plane bulunduysa onu bozma; sadece fallback durumda hizala.
                if not used_right_angle:
                    local_plane = align_plane_x_to_longer_bbox(temp_id, local_plane)

                rs.ViewCPlane(view, local_plane)

                if add_inside_dimensions(temp_id, local_plane):
                    basarili += 1
                else:
                    atlanan.append("seçili yüzey")
            finally:
                rs.DeleteObject(temp_id)

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
