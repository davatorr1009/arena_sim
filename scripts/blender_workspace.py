"""Espacio de trabajo "Espejo" en Blender.

Izquierda: la camara del espejo fija con render en vivo (Cycles).
Derecha: la arena en vista previa de materiales, para ver y mover los robots sin
tocar la camara (moverse en esa vista no altera la camara del espejo).

Uso desde Blender (consola o script):
    import sys; sys.path.insert(0, "<arena_sim>/scripts")
    import blender_workspace; blender_workspace.setup_mirror_workspace()
"""
import bpy
import mathutils

WS_NAME = "Espejo"
ARENA_CENTER = (1.0, 1.2, 0.05)


# ----------------------------------------------------------------------------
def _view3d_area(screen):
    return max((a for a in screen.areas if a.type == "VIEW_3D"), key=lambda a: a.width * a.height)


def setup_mirror_workspace(cam_name="Camara_Espejo"):
    """Crea/activa el workspace "Espejo" y divide su vista 3D en dos.

    Se hace en fases con bpy.app.timers: cambiar de workspace no es inmediato (la
    pantalla nueva se muestra en el siguiente ciclo de eventos) y dividir un area que
    aun no se ha dibujado puede cerrar Blender. Cada fase espera a que la anterior
    se haya aplicado en la interfaz."""
    wm = bpy.context.window_manager
    win = wm.windows[0]
    ws = bpy.data.workspaces.get(WS_NAME)
    if ws is None:
        src = bpy.data.workspaces.get("Layout") or win.workspace
        with bpy.context.temp_override(window=win, workspace=src):
            bpy.ops.workspace.duplicate()
        ws = win.workspace
        ws.name = WS_NAME
    win.workspace = ws
    args = (cam_name,)
    bpy.app.timers.register(lambda: _phase_split(win, args), first_interval=0.5)
    return ws


def _phase_split(win, args, tries=[0]):
    screen = win.screen
    if win.workspace.name != WS_NAME or any(a.width <= 1 for a in screen.areas):
        tries[0] += 1
        return 0.3 if tries[0] < 20 else None       # esperar a que se dibuje
    views = [a for a in screen.areas if a.type == "VIEW_3D"]
    if len(views) < 2:
        area = _view3d_area(screen)
        region = next(r for r in area.regions if r.type == "WINDOW")
        with bpy.context.temp_override(window=win, screen=screen, area=area, region=region):
            bpy.ops.screen.area_split(direction="VERTICAL", factor=0.5)
    bpy.app.timers.register(lambda: _phase_configure(win, args), first_interval=0.5)
    return None


def _phase_configure(win, args, tries=[0]):
    (cam_name,) = args
    screen = win.screen
    views = [a for a in screen.areas if a.type == "VIEW_3D"]
    if len(views) < 2 or any(a.width <= 1 for a in views):
        tries[0] += 1
        return 0.3 if tries[0] < 20 else None
    left, right = sorted(views, key=lambda a: a.x)[:2]
    cam = bpy.data.objects[cam_name]
    # izquierda: camara del espejo, render en vivo
    sp = left.spaces.active
    sp.camera = cam
    sp.use_local_camera = True
    sp.lock_camera = False          # navegar no mueve la camara
    sp.region_3d.view_perspective = "CAMERA"
    sp.region_3d.view_camera_zoom = 0
    sp.region_3d.view_camera_offset = (0.0, 0.0)
    sp.shading.type = "RENDERED"
    sp.overlay.show_overlays = False
    sp.show_gizmo = False
    # derecha: la arena en vista previa de materiales, libre para navegar
    sp = right.spaces.active
    r3 = sp.region_3d
    r3.view_perspective = "PERSP"
    r3.view_location = ARENA_CENTER
    r3.view_rotation = mathutils.Euler((0.9, 0.0, 2.45)).to_quaternion()
    r3.view_distance = 2.6
    sp.lens = 35
    sp.clip_start = 0.005
    sp.shading.type = "MATERIAL"
    sp.overlay.show_floor = False
    sp.overlay.show_axis_x = sp.overlay.show_axis_y = False
    sp.overlay.show_relationship_lines = False
    cam.data.display_size = 0.03
    sc = bpy.context.scene
    try:
        sc.cycles.preview_samples = 32
        sc.cycles.use_preview_denoising = True
    except AttributeError:
        pass
    print("[workspace Espejo] listo: izquierda = camara (render), derecha = arena")
    return None
