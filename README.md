# arena_sim

Simulación en Blender del espejo omnidireccional del robot QUPA. Sirve para renderizar la imagen que ve la cámara (640x480) a través del espejo y comparar distintas curvaturas antes de mandar a fabricar.

## Escena

- Habitación de 4 x 7 x 2,5 m: paredes beige, piso de baldosas café de 0,5 m y techo.
- Arena blanca de 2 x 2,4 x 0,3 m en la esquina (0,0) de la habitación.
- Robot QUPA en el centro de la arena. Usa las mallas y transformaciones de `qupa_description/urdf`: base, ruedas y ruedas locas, sin sensores IR/LEDs.
- 6 robots QUPA adicionales, cada uno con el domo de un color, a entre 0,3 y 1,1 m del robot central, para apreciar la distorsión (`other_robots` en `CONFIG`).
- Cámara OV5647 gran angular (`Camara_OV5647`, desde `assets/camera/ov5647_wide_angle_rpi_camera.step`), solo en el robot de prueba:
  - la placa está centrada sobre la columna de soporte (tope a 131 mm) y el conector mira al frente («espol», +X);
  - el lente está 3,5 mm descentrado en la placa, así que su eje queda en x = −3,5 mm;
  - el frente del lente y `Camara_Espejo` están a 145,2 mm, y el espejo, alineado con el lente, a 8,3 mm más (153,5 mm).
- Espejo paramétrico: superficie reflectiva de Ø24 mm x 12 mm, con una ceja de Ø25,4 mm x 1 mm. Usa normales analíticas y material de aluminio pulido.
- El render se hace con Cycles, porque las reflexiones del espejo necesitan trazado de rayos real.

## Uso

```bash
# construir la escena, guardar blend/arena_qupa.blend y renderizar renders/espejo_actual.png
blender -b -P scripts/build_scene.py -- --render
```

Para cambiar el espejo o la cámara se edita `CONFIG` en `scripts/build_scene.py`. Todas las medidas están en mm.

| Parámetro | Descripción |
|---|---|
| `mirror_profile` | `hyperbolic_svp`, `hyperbolic` (con `mirror_a`, `mirror_b`) o `spherical` |
| `mirror_radius`, `mirror_height` | Radio y altura de la superficie reflectiva |
| `cam_to_mirror` | Distancia del lente al vértice del espejo (8,3 mm, medida) |
| `cam_z` | Altura del frente del lente sobre `base_link` (145,23 mm) |
| `cam_offset_xy` | Eje del lente respecto al centro del robot (−3,5, 0) mm |
| `cam_center_px` | Centro del espejo en la imagen; centrado por defecto, la foto real da (290, 262) |
| `cam_vfov_deg`, `cam_sensor_mm` | Óptica de la cámara |
| `other_robots` | Robots adicionales: `(x, y, yaw_deg, color RGB del domo)` |

Con `hyperbolic_svp`, `a` y `b` se calculan para que el foco exterior del hiperboloide coincida con la cámara (punto de vista único). El objeto `Espejo` guarda `a`, `b`, `c`, `e` y los focos como propiedades personalizadas.

## Supuestos a validar

1. **Perfil del espejo:** se conocen el diámetro y la altura, pero no la ecuación. Se supone un hiperboloide de punto de vista único con la cámara en el foco exterior. El resultado es a = 5,742 mm, b = 4,105 mm, e = 1,229, y el foco interior queda 1,32 mm detrás del vértice. Si tienes los parámetros reales, usa `mirror_profile="hyperbolic"`.
2. **Óptica de la cámara:** focal de unos 351 px (FOV vertical de 68,7°, equivalente a un lente de unos 2 mm en la OV5647 a 640x480), ajustada con la foto real. Falta confirmarla con un cuadro sin máscara (`snap.py`).
3. **Altura de montaje:** la placa se supone apoyada en la columna de soporte. Así, el frente del lente queda a 145,2 mm y el espejo a 153,5 mm.
4. `assets/qupa_meshes/mirror.stl` (el espejo del URDF) es una **semiesfera** de R = 12 mm con ceja de Ø28 mm, no un hiperboloide. Se conserva solo como referencia.

## Espejos alternativos

En la misma posición que `Espejo` (A, hiperboloide actual) hay dos variantes ocultas: `Espejo_B_Esferico` (semiesfera R = 12) y `Espejo_C_Medida` (perfil a medida). Para pasar de una a otra se ejecuta el texto `toggle_espejo.py` en Blender (Alt+P), que va rotando entre las tres.

El perfil `custom` integra la ley de reflexión para un mapeo prescrito entre el radio en la imagen y la elevación:
- el 15 % central de la imagen muestra de −90° a −50°, que corresponde al propio robot;
- de ahí hasta 212 px se reparte de forma lineal la banda útil, de −50° a +5°.

La tabla r–z para fabricar está en `analysis/perfil_C_medida.csv`, y la comparación de píxeles por perfil en `analysis/perfiles.py`.

## Espacio de trabajo «Espejo» en Blender

`build_scene.py` (con interfaz) crea la pestaña «Espejo», que tiene dos vistas:
- **izquierda:** la cámara del espejo fija, con render en vivo de Cycles;
- **derecha:** la arena en vista previa de materiales.

En la vista derecha se puede navegar y mover los robots sin tocar la cámara. Para crear solo la pestaña: `import blender_workspace; blender_workspace.setup_mirror_workspace()`.

## Visualizador 2D de perfiles (sin Blender)

```bash
python3 scripts/mirror_designer.py                   # ventana interactiva
python3 scripts/mirror_designer.py --perfil esfera   # empezar con otro perfil
python3 scripts/mirror_designer.py --png vista.png   # guardar la figura sin abrir ventana
```

La ventana muestra tres paneles:
- el corte del espejo, con el robot y el abanico de rayos (en rojo, los que chocan con el propio robot);
- el mapeo entre el radio en la imagen y la elevación, comparado con el espejo actual;
- una imagen sintética de 640x480, con el piso marcado con anillos cada 10 cm y sectores cada 30°, las paredes y robots de prueba.

Todos los parámetros se ajustan con deslizadores. El botón «Exportar CSV» guarda el perfil (r, z, pendiente, px y elevación) en `analysis/`. La matemática está en `scripts/mirror_profiles.py`, que comparten este visualizador y `build_scene.py`.

## Exportar el espejo C a CAD

```bash
freecad.cmd scripts/mirror_to_cad.py                          # STEP + FCStd + puntos + ecuacion en cad/
KNEE=-35 UTOP=220 NOMBRE=espejo_C2 freecad.cmd scripts/mirror_to_cad.py   # otra variante
```

Genera cuatro archivos en `cad/`:
- `<nombre>.step` y `<nombre>.FCStd`: sólido de revolución con la superficie y la ceja;
- `<nombre>_puntos.csv`: el perfil r–z cada 0,05 mm;
- `<nombre>_ecuacion.txt`: dos polinomios, uno a cada lado del codo, con un error menor a 0,01 µm, listos para usar como «Curva de ecuación» en Inventor.

Con `python3` en lugar de `freecad.cmd` solo genera los puntos y la ecuación.

## Estructura

```
assets/qupa_meshes/  mallas del robot convertidas a OBJ (+ mirror.stl original)
scripts/             build_scene.py, blender_workspace.py, mirror_designer.py, mirror_profiles.py,
                     dae_to_obj.py, step_to_mesh.py
assets/camera/       modelos STEP de la cámara y su conversión a OBJ
blend/               archivo .blend generado
renders/             imágenes renderizadas
```

Las mallas vienen de `qupa_gazebo/qupa_description/meshes`. Se convierten con `python3 scripts/dae_to_obj.py --all <carpeta>`, porque Blender 5 ya no importa COLLADA.
