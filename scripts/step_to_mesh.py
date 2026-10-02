"""Convierte un modelo STEP (.step/.stp) a malla (.obj, .stl o .glb) con FreeCAD.

Blender no importa STEP directamente; este script se ejecuta con el FreeCAD instalado
(snap) y deja un archivo que Blender si importa. Cada solido del STEP queda como un
objeto separado en el OBJ. Las unidades se conservan en milimetros (en Blender importar
el OBJ con escala 0.001 o usar scripts/import_step_blender.py).

Uso (desde la carpeta arena_sim):
    STEP_IN=modelo.step MESH_OUT=modelo.obj freecad.cmd scripts/step_to_mesh.py
Opcional: DEFLECTION=0.05  (tolerancia lineal en mm; menor = mas triangulos)

Nota: FreeCAD como snap no ve /tmp; usar rutas dentro de $HOME.
"""
import os
import sys

import FreeCAD  # noqa: F401  (disponible dentro de freecad.cmd)
import Import
import Mesh
import MeshPart

src = os.path.abspath(os.path.expanduser(os.environ["STEP_IN"]))
dst = os.path.abspath(os.path.expanduser(os.environ.get("MESH_OUT", os.path.splitext(src)[0] + ".obj")))
deflection = float(os.environ.get("DEFLECTION", "0.05"))

doc = FreeCAD.newDocument("step_import")
Import.insert(src, doc.Name)
doc.recompute()

meshes = []
for obj in doc.Objects:
    if obj.TypeId != "Part::Feature":      # contenedores (App::Part, grupos) se saltan
        continue
    shape = obj.Shape
    if shape.isNull() or not shape.Faces:
        continue
    m = MeshPart.meshFromShape(Shape=shape.copy(), LinearDeflection=deflection,
                               AngularDeflection=0.3, Relative=False)
    if m.CountFacets == 0:
        continue
    mo = doc.addObject("Mesh::Feature", f"{obj.Label}_mesh")
    mo.Mesh = m
    mo.Label = obj.Label
    meshes.append(mo)

if not meshes:
    sys.exit(f"No se encontraron solidos en {src}")
Mesh.export(meshes, dst)
tri = sum(m.Mesh.CountFacets for m in meshes)
print(f"{os.path.basename(src)} -> {dst}: {len(meshes)} piezas, {tri} triangulos (mm)")
