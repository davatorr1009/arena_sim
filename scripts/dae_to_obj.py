#!/usr/bin/env python3
"""Convierte mallas COLLADA (.dae) simples a OBJ/MTL.

Blender >= 5.0 ya no incluye el importador COLLADA, asi que las mallas del
paquete qupa_description se convierten a OBJ una sola vez con este script.
Soporta <triangles>/<polylist> con transformacion <matrix> por nodo y color
difuso por material. Solo requiere numpy.

Uso:
    python3 dae_to_obj.py entrada.dae salida.obj
    python3 dae_to_obj.py --all ../assets/qupa_meshes
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _ns(root):
    return root.tag.split('}')[0] + '}' if root.tag.startswith('{') else ''


def _floats(el):
    return np.array(el.text.split(), dtype=float) if el is not None and el.text else np.zeros(0)


def _ints(el):
    return np.array(el.text.split(), dtype=int) if el is not None and el.text else np.zeros(0, int)


def material_colors(root, ns):
    """material id -> (r, g, b) usando el color difuso del effect."""
    effects = {}
    for eff in root.iter(ns + 'effect'):
        col = eff.find('.//' + ns + 'diffuse/' + ns + 'color')
        if col is not None:
            effects[eff.get('id')] = tuple(_floats(col)[:3])
    mats = {}
    for mat in root.iter(ns + 'material'):
        inst = mat.find(ns + 'instance_effect')
        url = inst.get('url', '#')[1:] if inst is not None else ''
        mats[mat.get('id')] = effects.get(url, (0.6, 0.6, 0.6))
    return mats


def geometry_triangles(geom, ns):
    """Devuelve lista de (material_symbol, triangulos Nx3x3) en coordenadas locales."""
    mesh = geom.find(ns + 'mesh')
    sources = {s.get('id'): s for s in mesh.findall(ns + 'source')}
    verts = mesh.find(ns + 'vertices')
    pos_src = verts.find(ns + "input[@semantic='POSITION']").get('source')[1:]
    src = sources[pos_src]
    acc = src.find('.//' + ns + 'accessor')
    stride = int(acc.get('stride', 3))
    pos = _floats(src.find(ns + 'float_array')).reshape(-1, stride)[:, :3]

    out = []
    for prim in list(mesh):
        tag = prim.tag.replace(ns, '')
        if tag not in ('triangles', 'polylist'):
            continue
        inputs = prim.findall(ns + 'input')
        n_off = max(int(i.get('offset', 0)) for i in inputs) + 1
        v_off = int(next(i for i in inputs if i.get('semantic') == 'VERTEX').get('offset', 0))
        p = _ints(prim.find(ns + 'p')).reshape(-1, n_off)[:, v_off]
        if tag == 'triangles':
            idx = p.reshape(-1, 3)
        else:
            counts = _ints(prim.find(ns + 'vcount'))
            tris, k = [], 0
            for c in counts:
                poly = p[k:k + c]
                tris += [(poly[0], poly[j], poly[j + 1]) for j in range(1, c - 1)]
                k += c
            idx = np.array(tris, dtype=int).reshape(-1, 3)
        out.append((prim.get('material'), pos[idx]))
    return out


def convert(dae_path, obj_path):
    dae_path, obj_path = Path(dae_path), Path(obj_path)
    root = ET.parse(dae_path).getroot()
    ns = _ns(root)
    unit = root.find(ns + 'asset/' + ns + 'unit')
    scale = float(unit.get('meter', 1.0)) if unit is not None else 1.0
    colors = material_colors(root, ns)
    geoms = {g.get('id'): g for g in root.iter(ns + 'geometry')}

    groups = []  # (nombre, material_id, tris)

    def walk(node, parent_m):
        m = parent_m
        mat_el = node.find(ns + 'matrix')
        if mat_el is not None:
            m = parent_m @ _floats(mat_el).reshape(4, 4)
        for ig in node.findall(ns + 'instance_geometry'):
            binds = {b.get('symbol'): b.get('target', '#')[1:]
                     for b in ig.iter(ns + 'instance_material')}
            for sym, tris in geometry_triangles(geoms[ig.get('url')[1:]], ns):
                flat = tris.reshape(-1, 3)
                h = np.c_[flat, np.ones(len(flat))] @ m.T
                groups.append((node.get('name') or node.get('id'),
                               binds.get(sym, sym), h[:, :3].reshape(-1, 3, 3) * scale))
        for child in node.findall(ns + 'node'):
            walk(child, m)

    for vs in root.iter(ns + 'visual_scene'):
        for node in vs.findall(ns + 'node'):
            walk(node, np.eye(4))

    mtl_path = obj_path.with_suffix('.mtl')
    used = sorted({g[1] or 'default' for g in groups})
    with open(mtl_path, 'w') as f:
        for name in used:
            r, g, b = colors.get(name, (0.6, 0.6, 0.6))
            f.write(f'newmtl {name}\nKd {r:.4f} {g:.4f} {b:.4f}\n\n')
    with open(obj_path, 'w') as f:
        f.write(f'# convertido desde {dae_path.name}\nmtllib {mtl_path.name}\n')
        base = 1
        for name, mat, tris in groups:
            f.write(f'o {name}\nusemtl {mat or "default"}\n')
            for v in tris.reshape(-1, 3):
                f.write(f'v {v[0]:.7f} {v[1]:.7f} {v[2]:.7f}\n')
            for t in range(len(tris)):
                a = base + 3 * t
                f.write(f'f {a} {a + 1} {a + 2}\n')
            base += 3 * len(tris)
    n = sum(len(g[2]) for g in groups)
    print(f'{dae_path.name} -> {obj_path.name}: {n} triangulos')


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--all':
        for p in sorted(Path(sys.argv[2]).glob('*.dae')):
            convert(p, p.with_suffix('.obj'))
    elif len(sys.argv) == 3:
        convert(sys.argv[1], sys.argv[2])
    else:
        print(__doc__)
        sys.exit(1)
