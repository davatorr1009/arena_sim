"""Perfiles de espejo omnidireccional y trazado de rayos 2D (solo numpy).

Convencion (todo en mm salvo que se indique):
  - camara pinhole en el origen mirando +z, vertice del espejo en z = D
  - perfil z(r) medido desde el vertice (z=0 en r=0) hacia la ceja
  - elevacion phi del rayo reflejado: 0 = horizonte, negativa = hacia el piso
  - u = radio en la imagen (px) = f_px * tan(theta), theta = angulo del rayo con el eje

Lo usan scripts/mirror_designer.py (visualizador 2D) y scripts/build_scene.py (Blender).
"""
import math

import numpy as np


def f_px_from_vfov(vfov_deg, height_px=480):
    return (height_px / 2) / math.tan(math.radians(vfov_deg) / 2)


def svp_hyperbola(R, H, D):
    """a, b del hiperboloide de punto de vista unico con z(R)=H y foco exterior a D del vertice."""
    A = R * R + 4 * H * D
    B = 2 * D * H * H - 2 * H * D * D
    C = -(H * H) * (D * D)
    a = (-B + math.sqrt(B * B - 4 * A * C)) / (2 * A)
    return a, math.sqrt(D * D - 2 * a * D)


def hyperbolic(a, b, R, n=2000):
    r = np.linspace(0.0, R, n)
    q = np.sqrt(1 + (r / b) ** 2)
    return r, a * q - a, a * r / (b * b * q)


def spherical(Rs, R, n=2000):
    R = min(R, Rs * 0.9999)
    r = np.linspace(0.0, R, n)
    w = np.sqrt(Rs * Rs - r * r)
    return r, Rs - w, r / w


def custom(D, f_px, knee_deg, top_deg, center_frac, u_top, R, n=20000):
    """Perfil a medida para el mapeo radio imagen -> elevacion:
         0 .. center_frac*u_top : -90 -> knee (robot propio comprimido al centro)
         center_frac*u_top .. u_top : knee -> top, lineal (resolucion uniforme)
       continuado hasta el radio R. Integra la ley de reflexion en polares:
         d(ln rho)/d(theta) = cos(theta + phi) / (1 - sin(theta + phi))"""
    u1 = center_frac * u_top
    th = np.linspace(0.0, math.atan(3 * u_top / f_px), n)
    u = f_px * np.tan(th)
    phi = np.radians(np.where(u < u1, -90 + (knee_deg + 90) * u / u1,
                              knee_deg + (top_deg - knee_deg) * (u - u1) / (u_top - u1)))
    g = np.cos(th + phi) / (1 - np.sin(th + phi))
    rho = D * np.exp(np.concatenate([[0.0], np.cumsum(0.5 * (g[1:] + g[:-1]) * np.diff(th))]))
    r, z = rho * np.sin(th), rho * np.cos(th) - D
    s = (np.cos(phi) - np.sin(th)) / (np.cos(th) - np.sin(phi))
    k = min(int(np.searchsorted(r, R)) + 2, len(r))
    return r[:k], z[:k], s[:k]


def trace(r, z, s, D, f_px):
    """Rayo camara -> punto del perfil -> reflejado. Devuelve u (px), phi (rad) y la
    direccion reflejada (o_r, o_z) para cada muestra del perfil."""
    zz = D + z
    th = np.arctan2(r, zz)
    d = np.stack([np.sin(th), np.cos(th)], 1)
    nrm = np.stack([s, -np.ones_like(s)], 1)
    nrm /= np.linalg.norm(nrm, axis=1)[:, None]
    o = d - 2 * (d * nrm).sum(1)[:, None] * nrm
    return f_px * np.tan(th), np.arctan2(o[:, 1], o[:, 0]), o


def make_profile(kind, p):
    """kind: 'hyperbolic' | 'hyperbolic_svp' | 'spherical' | 'custom'; p: dict de parametros."""
    if kind == "hyperbolic":
        return hyperbolic(p["a"], p["b"], p["R"])
    if kind == "hyperbolic_svp":
        a, b = svp_hyperbola(p["R"], p["H"], p["D"])
        return hyperbolic(a, b, p["R"])
    if kind == "spherical":
        return spherical(p["Rs"], p["R"])
    if kind == "custom":
        return custom(p["D"], p["f_px"], p["knee"], p["top"], p["center_frac"], p["u_top"], p["R"])
    raise ValueError(kind)
