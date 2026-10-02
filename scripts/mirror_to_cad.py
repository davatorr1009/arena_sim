"""Exporta el espejo "a medida" (perfil C) para CAD: STEP/FCStd, puntos y ecuacion.

El perfil C no es una conica: sale de integrar la ley de reflexion para un mapeo
imagen -> elevacion (ver mirror_profiles.custom). Para llevarlo a CAD se entrega en
tres formas equivalentes:
  1. <nombre>_puntos.csv   r,z cada 0.05 mm  -> spline por puntos (Inventor: Importar
                           puntos desde Excel en un boceto 2D; FreeCAD: B-spline).
  2. <nombre>_ecuacion.txt dos polinomios (tramo 1: vertice -> codo, par; tramo 2: codo ->
                           borde, en (r - r_codo)), error < 0.01 um -> dos "Curvas de
                           ecuacion" explicitas en Inventor. Un solo polinomio no sirve: en el
                           codo del diseno cambia la curvatura (error ~90 um).
  3. <nombre>.step / .FCStd  solido de revolucion (superficie + ceja), solo si se ejecuta
                           dentro de FreeCAD.

Uso:
    python3 scripts/mirror_to_cad.py                       # CSV + ecuacion
    freecad.cmd scripts/mirror_to_cad.py                   # ademas STEP y FCStd
Parametros (variables de entorno, valores por defecto = CONFIG de build_scene.py):
    D=8.3 VFOV=68.7 KNEE=-30 TOP=5 CENTER=0.15 UTOP=212 R=12 RF=12.7 TF=1.0
    NOMBRE=espejo_C OUT=cad
Ejemplo:
    KNEE=-35 UTOP=220 NOMBRE=espejo_C2 freecad.cmd scripts/mirror_to_cad.py
"""
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(os.path.abspath(__file__ if "__file__" in globals() else sys.argv[-1])).parent
sys.path.insert(0, str(HERE))
import mirror_profiles as mp  # noqa: E402

ROOT = HERE.parent


def params():
    e = os.environ.get
    return dict(D=float(e("D", 8.3)), vfov=float(e("VFOV", 68.7)), knee=float(e("KNEE", -30)),
                top=float(e("TOP", 5)), center=float(e("CENTER", 0.15)), u_top=float(e("UTOP", 212)),
                R=float(e("R", 12.0)), Rf=float(e("RF", 12.7)), tf=float(e("TF", 1.0)),
                nombre=e("NOMBRE", "espejo_C"), out=e("OUT", "cad"))


def profile(p):
    f_px = mp.f_px_from_vfov(p["vfov"])
    r, z, s = mp.custom(p["D"], f_px, p["knee"], p["top"], p["center"], p["u_top"], p["R"])
    rr = np.arange(0.0, p["R"] + 1e-9, 0.05)
    return rr, np.interp(rr, r, z), np.interp(rr, r, s)


def knee_radius(p):
    """Radio del espejo donde termina la zona del propio robot (codo del mapeo)."""
    f_px = mp.f_px_from_vfov(p["vfov"])
    r, z, s = mp.custom(p["D"], f_px, p["knee"], p["top"], p["center"], p["u_top"], p["R"])
    u, _, _ = mp.trace(r, z, s, p["D"], f_px)
    return float(np.interp(p["center"] * p["u_top"], u, r))


def fit_piecewise(r, z, R, rk):
    """Tramo 1 (0..rk): z = sum a_{2i} r^{2i}.  Tramo 2 (rk..R): z = sum b_k (r - rk)^k."""
    m1, m2 = r <= rk, r >= rk
    x1 = r[m1] / rk
    A1 = np.stack([x1 ** (2 * i) for i in range(1, 4)], 1)
    c1, *_ = np.linalg.lstsq(A1, z[m1], rcond=None)
    e1 = np.abs(A1 @ c1 - z[m1]).max()
    L = R - rk
    for deg in range(5, 13):
        x2 = (r[m2] - rk) / L
        A2 = np.stack([x2 ** k for k in range(deg + 1)], 1)
        c2, *_ = np.linalg.lstsq(A2, z[m2], rcond=None)
        e2 = np.abs(A2 @ c2 - z[m2]).max()
        if e2 < 1e-5:
            break
    t1 = [(2 * i, c1[i - 1] / rk ** (2 * i)) for i in range(1, 4)]
    t2 = [(k, c2[k] / L ** k) for k in range(deg + 1)]
    return t1, t2, max(e1, e2)


def write_outputs(p, rr, zz, ss, t1, t2, rk, err):
    out = ROOT / p["out"]
    out.mkdir(exist_ok=True)
    hdr = (f"# Espejo {p['nombre']}: D={p['D']} mm, FOV vert={p['vfov']} deg, codo={p['knee']} deg, "
           f"tope={p['top']} deg, centro={p['center']}, u_tope={p['u_top']} px, R={p['R']} mm\n"
           f"# r desde el eje, z desde el vertice hacia la ceja (mm). Ceja: R={p['Rf']} mm, espesor {p['tf']} mm\n")
    csv = out / f"{p['nombre']}_puntos.csv"
    with open(csv, "w") as f:
        f.write(hdr + "r_mm,z_mm,pendiente_dz_dr\n")
        for r, z, s in zip(rr, zz, ss):
            f.write(f"{r:.4f},{z:.5f},{s:.6f}\n")
    eq = out / f"{p['nombre']}_ecuacion.txt"
    p1 = " + ".join(f"({a:.12e})*r^{k}" for k, a in t1)
    p2 = " + ".join(f"({a:.12e})*(r-{rk:.6f})^{k}" for k, a in t2)
    i1 = " + ".join(f"({a:.12e})*t^{k}" for k, a in t1)
    i2 = " + ".join(f"({a:.12e})*(t-{rk:.6f})^{k}" for k, a in t2)
    with open(eq, "w") as f:
        f.write(hdr)
        f.write(f"\nTramo 1, 0 <= r <= {rk:.6f} mm (propio robot, vertice):\n  z(r) = {p1}\n")
        f.write(f"\nTramo 2, {rk:.6f} <= r <= {p['R']} mm (banda util):\n  z(r) = {p2}\n")
        f.write(f"\nError maximo del ajuste frente al perfil integrado: {err * 1000:.4f} um\n")
        f.write(f"\nInventor: dos Curvas de ecuacion explicitas en un boceto 2D (plano XZ), unidas en el codo:\n"
                f"  curva 1: x(t) = t, y(t) = {i1}, t de 0 a {rk:.6f}\n"
                f"  curva 2: x(t) = t, y(t) = {i2}, t de {rk:.6f} a {p['R']}\n"
                f"  y luego cerrar el perfil con la ceja y revolucionar alrededor del eje Y del boceto.\n")
        f.write(f"\nAltura en el borde: z({p['R']}) = {zz[-1]:.4f} mm; con la ceja el espejo mide "
                f"{zz[-1] + p['tf']:.4f} mm de alto y {2 * p['Rf']:.2f} mm de diametro.\n")
    print(f"puntos:   {csv}\necuacion: {eq}  (dos tramos, codo en r = {rk:.3f} mm, error max {err * 1000:.4f} um)")
    return out


def build_freecad_solid(p, rr, zz, out):
    import FreeCAD
    import Part
    V = FreeCAD.Vector
    H, R, Rf, tf = zz[-1], p["R"], p["Rf"], p["tf"]
    pts = [V(float(r), 0, float(z)) for r, z in zip(rr, zz)]
    curve = Part.BSplineCurve()
    curve.interpolate(pts)
    edges = [curve.toShape(),
             Part.LineSegment(V(R, 0, H), V(Rf, 0, H)).toShape(),
             Part.LineSegment(V(Rf, 0, H), V(Rf, 0, H + tf)).toShape(),
             Part.LineSegment(V(Rf, 0, H + tf), V(0, 0, H + tf)).toShape(),
             Part.LineSegment(V(0, 0, H + tf), V(0, 0, 0)).toShape()]
    face = Part.Face(Part.Wire(edges))
    solid = face.revolve(V(0, 0, 0), V(0, 0, 1), 360)
    edge = curve.toShape()
    dev = max(edge.distToShape(Part.Vertex(V(float(r), 0, float(z))))[0] for r, z in zip(rr[::5], zz[::5]))
    doc = FreeCAD.newDocument(p["nombre"])
    obj = doc.addObject("Part::Feature", p["nombre"])
    obj.Shape = solid
    doc.recompute()
    step = out / f"{p['nombre']}.step"
    fcstd = out / f"{p['nombre']}.FCStd"
    solid.exportStep(str(step))
    doc.saveAs(str(fcstd))
    bb = solid.BoundBox
    print(f"STEP:     {step}\nFCStd:    {fcstd}\n"
          f"solido: valido={solid.isValid()}, volumen={solid.Volume:.2f} mm3, "
          f"caja {bb.XLength:.2f} x {bb.YLength:.2f} x {bb.ZLength:.2f} mm, "
          f"desviacion del B-spline {dev * 1000:.4f} um")


def main():
    p = params()
    rr, zz, ss = profile(p)
    rk = knee_radius(p)
    t1, t2, err = fit_piecewise(rr, zz, p["R"], rk)
    out = write_outputs(p, rr, zz, ss, t1, t2, rk, err)
    try:
        import FreeCAD  # noqa: F401
    except ImportError:
        print("(sin FreeCAD: para el STEP ejecutar con  freecad.cmd scripts/mirror_to_cad.py)")
        return
    build_freecad_solid(p, rr, zz, out)


main()
