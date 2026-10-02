"""Escena de simulacion del espejo omnidireccional del robot QUPA.

Construye desde cero: habitacion 4 x 7 x 2.5 m, arena blanca 2 x 2.4 x 0.3 m en
una esquina, robot QUPA (mallas del URDF qupa_description, sin sensores IR/LEDs),
espejo hiperboloide parametrico y la camara 640x480 que lo observa.

Ejecutar sin interfaz:
    blender -b -P scripts/build_scene.py -- [--render] [--no-save]
o desde la consola de Python de Blender:
    exec(open("scripts/build_scene.py").read(), {"__file__": "scripts/build_scene.py"})

Todas las medidas del espejo/camara estan en milimetros en CONFIG.
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy
import mathutils

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import mirror_profiles  # noqa: E402  (matematica de perfiles compartida, solo numpy)
MESHES = ROOT / "assets" / "qupa_meshes"

CONFIG = {
    # --- Espejo (mm) ---
    # perfil: "hyperbolic_svp" -> hiperboloide de punto de vista unico cuyo foco
    #          exterior coincide con el centro optico de la camara (se resuelven a, b
    #          a partir de radio, altura y distancia camara-vertice).
    #         "hyperbolic"     -> hiperboloide con a, b explicitos.
    #         "spherical"      -> casquete esferico de radio sphere_radius.
    "mirror_profile": "hyperbolic",   # espejo actual (A)
    "mirror_radius": 12.0,        # radio de la superficie reflectiva (diametro 24)
    "mirror_height": 12.0,        # altura (sagita) de la superficie reflectiva
    "mirror_a": 5.7419,           # solo para "hyperbolic": estimado del espejo actual
    "mirror_b": 4.1045,           # (SVP con 24 x 12 mm y foco a 12.8 mm; coincide con la foto real)
    "sphere_radius": 12.0,        # solo para "spherical"
    # "custom": perfil a medida integrando la ley de reflexion para un mapeo
    # radio en imagen (px) -> elevacion (deg) prescrito:
    #   0 .. center_frac*u_top : -90 -> knee   (robot propio comprimido al centro)
    #   center_frac*u_top .. u_top : knee -> top, lineal (resolucion uniforme)
    # y se continua el mismo mapeo hasta mirror_radius (fuera del cuadro).
    "custom_knee_deg": -30.0,     # silueta del propio robot vista desde el espejo (D=8.3, domo a ~10 mm)
    "custom_top_deg": 5.0,        # por encima del horizonte (robots < 0 deg a cualquier distancia)
    "custom_center_frac": 0.15,   # fraccion del radio util para el propio robot
    "custom_u_top_px": 212.0,     # radio en px del limite superior (centro real en y=262 -> <= 218)
    "flange_radius": 12.7,        # ceja: diametro 25.4
    "flange_thickness": 1.0,
    "mirror_rings": 240,          # resolucion radial del perfil
    "mirror_segments": 256,       # resolucion angular

    # --- Camara ---
    "cam_to_mirror": 8.3,         # distancia lente -> vertice del espejo (mm), medida en laboratorio
    "cam_z": 145.23,              # frente del lente sobre base_link (mm): placa apoyada en la
                                  # columna de soporte (tope 131 mm) + 14.23 mm placa -> lente
    "cam_offset_xy": (-3.5, 0.0), # eje del lente respecto al centro del robot (mm): la placa se
                                  # centra en la columna, el lente esta 3.5 mm descentrado en la
                                  # placa y el conector mira al frente (+X, "espol")
    "cam_vfov_deg": 68.7,         # FOV vertical (f = 351 px, ~2 mm con pixel 5.6 um): ajuste a la foto real con D=8.3
    # Camara: Raspberry Pi Camera v1 (OV5647) con lente de enfoque manual; 640x480 usa
    # el sensor completo con escalado 4x (pixel efectivo 5.6 um). Falta confirmar la focal.
    "cam_center_px": (319.5, 239.5),  # centro del espejo en la imagen (centrado; la foto real da (290, 262))
    "cam_sensor_mm": (3.6, 2.7),  # sensor 4:3 (supuesto)
    "resolution": (640, 480),

    # --- Robot / escena (m) ---
    "robot_xy": (1.0, 1.2),       # centro de la arena
    "robot_yaw_deg": 0.0,
    # Robots adicionales para apreciar la distorsion: (x, y, yaw_deg, color RGB del domo).
    # Distintas distancias y direcciones respecto al robot central.
    "other_robots": [
        (1.30, 1.20, 180, (0.80, 0.08, 0.06)),   # rojo,     0.30 m al frente
        (1.00, 1.65, -90, (0.05, 0.20, 0.80)),   # azul,     0.45 m a la izquierda
        (0.45, 0.70, 45, (0.10, 0.60, 0.12)),    # verde,    0.74 m atras-derecha
        (1.55, 0.55, 130, (0.90, 0.70, 0.05)),   # amarillo, 0.85 m adelante-derecha
        (0.30, 1.95, -30, (0.50, 0.10, 0.65)),   # morado,   1.03 m atras-izquierda
        (1.70, 2.05, -140, (0.95, 0.35, 0.02)),  # naranja,  1.10 m adelante-izquierda
    ],
    "samples": 128,
}


# ----------------------------------------------------------------------------
# Utilidades
# ----------------------------------------------------------------------------
def get_collection(name, parent=None):
    col = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    parent = parent or bpy.context.scene.collection
    if col.name not in parent.children:
        parent.children.link(col)
    return col


def move_to(obj, col):
    for c in obj.users_collection:
        c.objects.unlink(obj)
    col.objects.link(obj)


def principled(mat):
    return next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")


def make_material(name, rgb, roughness=0.5, metallic=0.0):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    try:
        mat.use_nodes = True
    except AttributeError:
        pass
    b = principled(mat)
    b.inputs["Base Color"].default_value = (*rgb, 1.0)
    b.inputs["Roughness"].default_value = roughness
    b.inputs["Metallic"].default_value = metallic
    mat.diffuse_color = (*rgb, 1.0)
    return mat


def add_box(name, size, loc, mat, col):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    bpy.ops.object.transform_apply(scale=True)
    obj.data.materials.append(mat)
    move_to(obj, col)
    return obj


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for col in list(bpy.data.collections):
        bpy.data.collections.remove(col)
    _MESH_CACHE.clear()
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.cameras, bpy.data.lights):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


# ----------------------------------------------------------------------------
# Habitacion y arena
# ----------------------------------------------------------------------------
def build_room():
    col = get_collection("Habitacion")
    L, W, H, t = 4.0, 7.0, 2.5, 0.1
    wall = make_material("Pared_Beige", (0.62, 0.50, 0.34), 0.8)

    floor_mat = bpy.data.materials.new("Piso_Baldosa_Cafe")
    try:
        floor_mat.use_nodes = True
    except AttributeError:
        pass
    nt = floor_mat.node_tree
    b = principled(floor_mat)
    b.inputs["Roughness"].default_value = 0.35
    tc = nt.nodes.new("ShaderNodeTexCoord")
    brick = nt.nodes.new("ShaderNodeTexBrick")
    brick.offset = 0.0
    brick.squash = 1.0
    brick.inputs["Scale"].default_value = 1.0
    brick.inputs["Brick Width"].default_value = 0.5
    brick.inputs["Row Height"].default_value = 0.5
    brick.inputs["Mortar Size"].default_value = 0.006
    brick.inputs["Color1"].default_value = (0.14, 0.065, 0.025, 1)
    brick.inputs["Color2"].default_value = (0.17, 0.08, 0.03, 1)
    brick.inputs["Mortar"].default_value = (0.35, 0.30, 0.25, 1)
    nt.links.new(tc.outputs["Object"], brick.inputs["Vector"])
    nt.links.new(brick.outputs["Color"], b.inputs["Base Color"])
    floor_mat.diffuse_color = (0.33, 0.19, 0.09, 1)

    # Piso con origen en la esquina (0,0) para que las baldosas queden alineadas
    floor = add_box("Piso", (L, W, 0.05), (0, 0, 0), floor_mat, col)
    floor.data.transform(mathutils.Matrix.Translation((L / 2, W / 2, -0.025)))
    add_box("Pared_Sur", (L + 2 * t, t, H), (L / 2, -t / 2, H / 2), wall, col)
    add_box("Pared_Norte", (L + 2 * t, t, H), (L / 2, W + t / 2, H / 2), wall, col)
    add_box("Pared_Oeste", (t, W, H), (-t / 2, W / 2, H / 2), wall, col)
    add_box("Pared_Este", (t, W, H), (L + t / 2, W / 2, H / 2), wall, col)
    # Techo: el espejo ve hacia arriba, sin techo reflejaria el fondo vacio.
    # Oculto en el viewport para poder ver la habitacion desde arriba.
    ceiling = add_box("Techo", (L + 2 * t, W + 2 * t, t), (L / 2, W / 2, H + t / 2),
                      make_material("Techo_Blanco", (0.85, 0.84, 0.80), 0.9), col)
    ceiling.hide_viewport = True

    bpy.ops.object.light_add(type="AREA", location=(L / 2, W / 2, H - 0.05))
    light = bpy.context.active_object
    light.name = "Luz_Techo"
    light.data.shape = "RECTANGLE"
    light.data.size, light.data.size_y = 1.5, 3.0
    light.data.energy = 350
    move_to(light, col)

    bpy.ops.object.camera_add(location=(3.9, 6.9, 2.4))
    view = bpy.context.active_object
    view.name = "Camara_Vista"
    view.data.lens = 16
    d = mathutils.Vector((1.2, 1.5, 0.0)) - view.location
    view.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    move_to(view, col)


def build_arena():
    col = get_collection("Arena")
    mat = make_material("Arena_Blanco", (0.95, 0.95, 0.95), 0.5)
    aL, aW, aH, wt, bt = 2.0, 2.4, 0.3, 0.02, 0.02
    add_box("Arena_Base", (aL, aW, bt), (aL / 2, aW / 2, bt / 2), mat, col)
    add_box("Arena_Borde_S", (aL, wt, aH), (aL / 2, wt / 2, aH / 2), mat, col)
    add_box("Arena_Borde_N", (aL, wt, aH), (aL / 2, aW - wt / 2, aH / 2), mat, col)
    add_box("Arena_Borde_O", (wt, aW - 2 * wt, aH), (wt / 2, aW / 2, aH / 2), mat, col)
    add_box("Arena_Borde_E", (wt, aW - 2 * wt, aH), (aL - wt / 2, aW / 2, aH / 2), mat, col)
    return bt  # altura del piso de la arena


# ----------------------------------------------------------------------------
# Robot QUPA (transformaciones de qupa_description/urdf, sin sensores IR)
# ----------------------------------------------------------------------------
URDF_VISUALS = [
    # (malla, xyz del joint, rpy del joint, escala, offset visual z)
    ("qupa.obj", (0, 0, 0), (0, 0, 0), 1.0, 0.0),
    ("wheel.obj", (0, 0.0415, 0.016), (1.5708, 0, 3.1416), 0.001, 0.001),    # R_wheel
    ("wheel.obj", (0, -0.0415, 0.016), (1.5708, 0, 0), 0.001, 0.001),        # L_wheel
    ("caster.obj", (0.0305, -0.0265, 0.005), (0, 0, 0), 0.8, 0.0),
    ("caster.obj", (0.0305, 0.0265, 0.005), (0, 0, 0), 0.8, 0.0),
    ("caster.obj", (-0.0325, 0, 0.005), (0, 0, 0), 0.8, 0.0),
]
LINK_NAMES = ["base_link", "R_wheel", "L_wheel", "caster_R", "caster_L", "caster_B"]


_MESH_CACHE = {}


def _mesh_parts(mesh):
    """Importa cada OBJ una sola vez; las copias del robot comparten la malla."""
    if mesh not in _MESH_CACHE:
        bpy.ops.wm.obj_import(filepath=str(MESHES / mesh), forward_axis="Y", up_axis="Z")
        parts = list(bpy.context.selected_objects)
        _MESH_CACHE[mesh] = [o.data for o in parts]
        if mesh == "qupa.obj":
            _split_dome(parts[0].data)
        for o in parts:
            bpy.data.objects.remove(o, do_unlink=True)
    return _MESH_CACHE[mesh]


def _split_dome(me):
    """Suelda la malla del cuerpo y asigna material_index=1 a la pieza suelta del
    domo (la unica que empieza por encima de 80 mm) para poder colorearla aparte."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    seen = set()
    for f in bm.faces:
        if f in seen:
            continue
        stack, comp = [f], []
        seen.add(f)
        while stack:
            g = stack.pop()
            comp.append(g)
            for e in g.edges:
                for h in e.link_faces:
                    if h not in seen:
                        seen.add(h)
                        stack.append(h)
        idx = 1 if min(v.co.z for g in comp for v in g.verts) > 0.080 else 0
        for g in comp:
            g.material_index = idx
    bm.to_mesh(me)
    bm.free()
    while len(me.materials) < 2:
        me.materials.append(me.materials[0] if me.materials else None)


def build_robot(floor_z, name="QUPA", xy=None, yaw_deg=None, dome_rgb=(0.85, 0.85, 0.85)):
    col = get_collection(name)
    base = bpy.data.objects.new(f"{name}_base_link", None)
    base.empty_display_type = "ARROWS"
    base.empty_display_size = 0.05
    col.objects.link(base)
    x, y = xy if xy is not None else CONFIG["robot_xy"]
    yaw = yaw_deg if yaw_deg is not None else CONFIG["robot_yaw_deg"]
    base.location = (x, y, floor_z)
    base.rotation_euler = (0, 0, math.radians(yaw))

    body_mat = make_material("QUPA_Cuerpo", (0.85, 0.85, 0.85), 0.45)
    dome_mat = make_material(f"{name}_Domo", dome_rgb, 0.45)
    dark_mat = make_material("QUPA_Oscuro", (0.05, 0.05, 0.05), 0.6)
    for (mesh, xyz, rpy, scale, vz), link in zip(URDF_VISUALS, LINK_NAMES):
        datas = _mesh_parts(mesh)
        joint = mathutils.Matrix.Translation(xyz) @ mathutils.Euler(rpy, "XYZ").to_matrix().to_4x4()
        visual = mathutils.Matrix.Translation((0, 0, vz)) @ mathutils.Matrix.Scale(scale, 4)
        for i, data in enumerate(datas):
            obj_name = link if len(datas) == 1 else f"{link}_{i}"
            if name != "QUPA":
                obj_name = f"{name}_{obj_name}" + ("_mesh" if link == "base_link" else "")
            obj = bpy.data.objects.new(obj_name, data)
            col.objects.link(obj)
            obj.parent = base
            obj.matrix_parent_inverse.identity()
            obj.matrix_basis = joint @ visual
            # material por objeto: cada robot con su color sin duplicar la malla
            if not obj.material_slots:
                data.materials.append(dark_mat)
            mats = [body_mat, dome_mat] if mesh == "qupa.obj" else [dark_mat] * len(obj.material_slots)
            for slot, m in zip(obj.material_slots, mats):
                slot.link = "OBJECT"
                slot.material = m
    return base


def attach_mirror_copy(base, name, sources, ref_base):
    """Copia (malla compartida) del espejo y modulo de camara sobre otro robot,
    con la misma posicion relativa que tienen en el robot de referencia."""
    bpy.context.view_layer.update()
    col = base.users_collection[0]
    inv = ref_base.matrix_world.inverted()
    for src in sources:
        obj = bpy.data.objects.new(f"{name}_{src.name}", src.data)
        col.objects.link(obj)
        obj.parent = base
        obj.matrix_basis = inv @ src.matrix_world


# ----------------------------------------------------------------------------
# Espejo parametrico
# ----------------------------------------------------------------------------
def solve_svp_hyperbola(R, H, D):
    """Hiperboloide z(r) = a*sqrt(1 + r^2/b^2) - a (vertice en z=0) con z(R)=H y
    foco exterior a distancia D del vertice (a + c = D, c^2 = a^2 + b^2)."""
    A = R * R + 4 * H * D
    B = 2 * D * H * H - 2 * H * D * D
    C = -(H * H) * (D * D)
    a = (-B + math.sqrt(B * B - 4 * A * C)) / (2 * A)
    b = math.sqrt(D * D - 2 * a * D)
    return a, b


def mirror_profile(cfg=None):
    """Devuelve (z(r), dz/dr(r), descripcion) en mm, vertice en z=0, z hacia el espejo."""
    cfg = cfg or CONFIG
    kind = cfg["mirror_profile"]
    if kind in ("hyperbolic_svp", "hyperbolic"):
        if kind == "hyperbolic_svp":
            a, b = solve_svp_hyperbola(cfg["mirror_radius"], cfg["mirror_height"], cfg["cam_to_mirror"])
        else:
            a, b = cfg["mirror_a"], cfg["mirror_b"]
        c = math.sqrt(a * a + b * b)
        z = lambda r: a * math.sqrt(1 + (r / b) ** 2) - a
        dz = lambda r: a * r / (b * b * math.sqrt(1 + (r / b) ** 2))
        info = dict(profile=kind, a=a, b=b, c=c, e=c / a,
                    focus_inside_from_vertex=c - a, focus_outside_from_vertex=a + c)
    elif kind == "custom":
        z, dz, info = custom_profile(cfg)
    elif kind == "spherical":
        Rs = cfg["sphere_radius"]
        z = lambda r: Rs - math.sqrt(max(Rs * Rs - r * r, 0.0))
        dz = lambda r: r / math.sqrt(max(Rs * Rs - r * r, 1e-9))
        info = dict(profile=kind, sphere_radius=Rs)
    else:
        raise ValueError(f"perfil desconocido: {kind}")
    info["sag_at_rim"] = z(cfg["mirror_radius"])
    return z, dz, info


def custom_profile(cfg):
    """Perfil a medida (ver scripts/mirror_profiles.py, compartido con mirror_designer.py)."""
    import numpy as np
    f_px = mirror_profiles.f_px_from_vfov(cfg["cam_vfov_deg"], cfg["resolution"][1])
    r, zz, slope = mirror_profiles.custom(cfg["cam_to_mirror"], f_px, cfg["custom_knee_deg"],
                                          cfg["custom_top_deg"], cfg["custom_center_frac"],
                                          cfg["custom_u_top_px"], cfg["mirror_radius"])
    u, phi, _ = mirror_profiles.trace(r, zz, slope, cfg["cam_to_mirror"], f_px)
    z_of = lambda x: float(np.interp(x, r, zz))
    dz_of = lambda x: float(np.interp(x, r, slope))
    R = cfg["mirror_radius"]
    info = dict(profile="custom", knee_deg=cfg["custom_knee_deg"], top_deg=cfg["custom_top_deg"],
                center_frac=cfg["custom_center_frac"], u_top_px=cfg["custom_u_top_px"],
                u_rim_px=float(np.interp(R, r, u)), phi_rim_deg=float(np.degrees(np.interp(R, r, phi))))
    return z_of, dz_of, info


def build_mirror(parent, cam_z_m, name="Espejo", overrides=None):
    """Superficie reflectiva (convexa hacia la camara) + ceja + tapa trasera.
    `overrides` reemplaza claves de CONFIG (p. ej. otro perfil) solo para este espejo."""
    cfg = {**CONFIG, **(overrides or {})}
    col = get_collection(name)
    z_of, dz_of, info = mirror_profile(cfg)
    R, Rf, tf = cfg["mirror_radius"], cfg["flange_radius"], cfg["flange_thickness"]
    NR, NS = cfg["mirror_rings"], cfg["mirror_segments"]
    Hs = z_of(R)
    mm = 0.001

    # --- superficie reflectiva, orientada hacia -Z (hacia la camara) ---
    verts, faces, normals = [(0.0, 0.0, 0.0)], [], [(0.0, 0.0, -1.0)]
    for i in range(1, NR + 1):
        r = R * i / NR
        z, s = z_of(r), dz_of(r)
        n = mathutils.Vector((s, 0.0, -1.0)).normalized()
        for j in range(NS):
            t = 2 * math.pi * j / NS
            ct, st = math.cos(t), math.sin(t)
            verts.append((r * ct * mm, r * st * mm, z * mm))
            normals.append((n.x * ct, n.x * st, n.z))
    ring = lambda i, j: 1 + (i - 1) * NS + (j % NS)
    for j in range(NS):
        faces.append((0, ring(1, j + 1), ring(1, j)))
    for i in range(1, NR):
        for j in range(NS):
            faces.append((ring(i, j), ring(i, j + 1), ring(i + 1, j + 1), ring(i + 1, j)))
    me = bpy.data.meshes.new(f"{name}_Superficie")
    me.from_pydata(verts, [], faces)
    me.update()
    me.shade_smooth()
    me.normals_split_custom_set_from_vertices(normals)  # normales analiticas exactas
    surf = bpy.data.objects.new(f"{name}_Superficie", me)
    col.objects.link(surf)

    # --- ceja + cuerpo trasero (revolucion de un perfil cerrado) ---
    prof = [(R, Hs), (Rf, Hs), (Rf, Hs + tf), (0.0, Hs + tf)]
    v2, f2 = [], []
    for (r, z) in prof:
        for j in range(NS):
            t = 2 * math.pi * j / NS
            v2.append((r * math.cos(t) * mm, r * math.sin(t) * mm, z * mm))
    for k in range(len(prof) - 1):
        for j in range(NS):
            a0, a1 = k * NS + j, k * NS + (j + 1) % NS
            f2.append((a0, a1, a1 + NS, a0 + NS))
    me2 = bpy.data.meshes.new(f"{name}_Ceja")
    me2.from_pydata(v2, [], f2)
    me2.update()
    bpy.context.view_layer.update()
    flange = bpy.data.objects.new(f"{name}_Ceja", me2)
    col.objects.link(flange)

    mirror_mat = make_material("Espejo_Aluminio", (0.91, 0.92, 0.92), 0.0, 1.0)
    back_mat = make_material("Espejo_Ceja_Mate", (0.75, 0.75, 0.76), 0.35, 1.0)
    surf.data.materials.append(mirror_mat)
    flange.data.materials.append(back_mat)

    root = bpy.data.objects.new(name, None)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.01
    col.objects.link(root)
    root.parent = parent
    ox, oy = cfg.get("cam_offset_xy", (0.0, 0.0))
    root.location = (ox * mm, oy * mm, cam_z_m + cfg["cam_to_mirror"] * mm)   # alineado con el lente
    surf.parent = root
    flange.parent = root
    for k, v in info.items():
        root[k] = v
    return root, info


# ----------------------------------------------------------------------------
# Camara del espejo
# ----------------------------------------------------------------------------
CAMERA_OBJ = ROOT / "assets" / "camera" / "ov5647_wide_angle_rpi_camera.obj"
# El STEP (OV5647 gran angular) tiene el eje optico en +X y el frente del lente en x = -5 mm;
# se gira a +Z y se deja el frente del lente en el origen del objeto.
CAMERA_BAKE = (mathutils.Matrix.Scale(0.001, 4) @ mathutils.Matrix.Translation((0, 0, 5.0))
               @ mathutils.Matrix.Rotation(math.radians(-90), 4, "Y"))
CAMERA_PCB_BACK = 0.01423  # m, del frente del lente a la cara trasera de la placa
# conector: centro original (0.51, -50.70, 8.21) mm, largo en x -> se gira a lo ancho (y) y se
# apoya en la cara trasera de la placa (x = -19.23 mm) junto al borde z = 15.5 mm
CONNECTOR_FIX = (mathutils.Matrix.Translation((-20.23, 0.0, 12.65))
                 @ mathutils.Matrix.Rotation(math.radians(90), 4, "Z")
                 @ mathutils.Matrix.Translation((-0.51, 50.70, -8.21)))


def camera_mesh():
    """Malla del modulo de camara (una sola vez), en metros y con colores por zona."""
    me = bpy.data.meshes.get("OV5647_Gran_Angular_Mesh")
    if me is not None:
        return me
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=str(CAMERA_OBJ), forward_axis="Y", up_axis="Z",
                          use_split_objects=True, use_split_groups=True)
    parts = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    bm = bmesh.new()
    for o in parts:
        ys = [v.co.y for v in o.data.vertices]
        tmp = o.data.copy()
        if ys and max(ys) < -20.0:
            # conector F52R-1A7H1-11015: el STEP lo trae suelto a ~50 mm; se lleva a la cara
            # trasera de la placa, en el borde largo (+z del modelo), a lo ancho de la placa
            tmp.transform(CAMERA_BAKE @ CONNECTOR_FIX @ o.matrix_world)
        else:
            tmp.transform(CAMERA_BAKE @ o.matrix_world)
        bm.from_mesh(tmp)
        bpy.data.meshes.remove(tmp)
    for o in parts:
        d = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if d.users == 0:
            bpy.data.meshes.remove(d)
    me = bpy.data.meshes.new("OV5647_Gran_Angular_Mesh")
    bm.to_mesh(me)
    bm.free()
    for m in (make_material("Camara_PCB_Verde", (0.02, 0.15, 0.06), 0.5),
              make_material("Camara_Negro", (0.02, 0.02, 0.02), 0.4),
              make_material("Camara_Componentes", (0.12, 0.12, 0.13), 0.35)):
        me.materials.append(m)
    for p in me.polygons:               # placa / cuerpo del lente / componentes y tornillos
        zs = [me.vertices[i].co.z for i in p.vertices]
        rs = [math.hypot(me.vertices[i].co.x, me.vertices[i].co.y) for i in p.vertices]
        if min(zs) < -0.01430:          # por detras de la placa: tornillos y conector
            p.material_index = 2
        elif max(zs) <= -0.01315:       # placa (1 mm)
            p.material_index = 0
        else:
            p.material_index = 1 if max(rs) <= 0.0085 else 2
    return me


def camera_model_object(name, parent, col):
    ob = bpy.data.objects.new(name, camera_mesh())
    col.objects.link(ob)
    ob.parent = parent
    return ob


def build_mirror_camera(parent):
    cfg = CONFIG
    col = get_collection("Camara_Espejo")
    cam_z = cfg["cam_z"] * 0.001
    data = bpy.data.cameras.new("Camara_Espejo")
    sw, sh = cfg["cam_sensor_mm"]
    data.sensor_fit = "VERTICAL"
    data.sensor_width, data.sensor_height = sw, sh
    data.lens = (sh / 2) / math.tan(math.radians(cfg["cam_vfov_deg"]) / 2)
    data.clip_start = 0.001
    data.clip_end = 20.0
    W, H = cfg["resolution"]
    cx, cy = cfg["cam_center_px"]
    data.shift_x = ((W - 1) / 2 - cx) / H
    data.shift_y = (cy - (H - 1) / 2) / H
    cam = bpy.data.objects.new("Camara_Espejo", data)
    col.objects.link(cam)
    cam.parent = parent
    ox, oy = (v * 0.001 for v in cfg["cam_offset_xy"])
    cam.location = (ox, oy, cam_z)
    # mira hacia +Z; el frente del robot (+X) queda arriba en la imagen
    cam.rotation_euler = (math.pi, 0, math.pi / 2)

    # modulo real: OV5647 gran angular (STEP convertido con scripts/step_to_mesh.py);
    # el frente del lente queda a la altura del centro optico
    # girado 180 deg: el borde del conector (+z del modelo -> -X) queda hacia el frente (+X)
    body = camera_model_object("Camara_OV5647", parent, col)
    body.location = (ox, oy, cam_z)
    body.rotation_euler = (0, 0, math.pi)
    return cam


# ----------------------------------------------------------------------------
def setup_render(cam):
    scene = bpy.context.scene
    try:
        scene.render.engine = "CYCLES"   # trazado de rayos real para el espejo
    except TypeError as e:
        print("No se pudo activar Cycles:", e)
    scene.cycles.samples = CONFIG["samples"]
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = CONFIG["resolution"]
    scene.render.resolution_percentage = 100
    scene.render.filepath = str(ROOT / "renders" / "espejo_")
    scene.unit_settings.system = "METRIC"
    scene.camera = cam
    world = scene.world or bpy.data.worlds.new("World")
    scene.world = world
    try:
        world.use_nodes = True
    except AttributeError:
        pass
    bg = next((n for n in world.node_tree.nodes if n.type == "BACKGROUND"), None)
    if bg:
        bg.inputs["Color"].default_value = (0.05, 0.05, 0.05, 1)


# Espejos alternativos en la misma posicion que el principal (se alternan con toggle)
MIRROR_VARIANTS = [
    ("Espejo_B_Esferico", {"mirror_profile": "spherical", "sphere_radius": 12.0}),
    ("Espejo_C_Medida", {"mirror_profile": "custom"}),
]

TOGGLE_CODE = '''import bpy
# Alterna (ciclo) entre los espejos: cada ejecucion (Alt+P) muestra el siguiente.
names = [c.name for c in bpy.data.collections if c.name.startswith("Espejo")]
names.sort(key=lambda n: (n != "Espejo", n))
lc = bpy.context.view_layer.layer_collection.children
cur = next((i for i, n in enumerate(names) if not bpy.data.collections[n].hide_render), -1)
nxt = (cur + 1) % len(names)
for i, n in enumerate(names):
    bpy.data.collections[n].hide_render = i != nxt
    lc[n].hide_viewport = i != nxt
print("Espejo activo:", names[nxt])
'''


def add_mirror_variants(base, mirror):
    for name, overrides in MIRROR_VARIANTS:
        root, info = build_mirror(base, 0.0, name=name, overrides=overrides)
        root.matrix_basis = mirror.matrix_basis.copy()
        bpy.data.collections[name].hide_render = True
        bpy.context.view_layer.layer_collection.children[name].hide_viewport = True
        print(name, {k: round(v, 3) if isinstance(v, float) else v for k, v in info.items()})
    t = bpy.data.texts.get("toggle_espejo.py") or bpy.data.texts.new("toggle_espejo.py")
    t.clear()
    t.write(TOGGLE_CODE)


def build():
    clear_scene()
    build_room()
    floor_z = build_arena()
    base = build_robot(floor_z)
    cam = build_mirror_camera(base)
    mirror, info = build_mirror(base, CONFIG["cam_z"] * 0.001)
    shared = [o for o in mirror.children]      # la camara solo va en el robot de prueba
    for k, (x, y, yaw, rgb) in enumerate(CONFIG["other_robots"], start=1):
        other = build_robot(floor_z, f"QUPA_{k}", (x, y), yaw, rgb)
        attach_mirror_copy(other, f"QUPA_{k}", shared, base)
    add_mirror_variants(base, mirror)
    setup_render(cam)
    if not bpy.app.background:
        import blender_workspace
        blender_workspace.setup_mirror_workspace()
    print("Espejo:", {k: round(v, 4) if isinstance(v, float) else v for k, v in info.items()})
    return info


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    build()
    if "--no-save" not in argv and bpy.app.background:
        bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "blend" / "arena_qupa.blend"))
    if "--render" in argv:
        bpy.context.scene.render.filepath = str(ROOT / "renders" / "espejo_actual.png")
        bpy.ops.render.render(write_still=True)
