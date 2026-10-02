"""Comparacion de perfiles de espejo: pixeles dedicados a cada banda de elevacion.

Camara (pinhole) en el origen mirando +z; vertice del espejo a D mm.
Elevacion phi del rayo reflejado: 0 = horizonte, negativa = hacia el piso.
"""
import numpy as np

D, R, F_PX = 20.0, 12.0, 483.0      # distancia camara-vertice, radio espejo, focal (px)
BAND = (-45.0, 10.0)                  # banda util: piso cercano .. tope de paredes

def mapping_from_profile(z_of, dz_of, n=4000):
    """Para un perfil z(r) (vertice en 0) devuelve arrays (u_px, phi_deg)."""
    r = np.linspace(1e-4, R, n); z = D + z_of(r)
    th = np.arctan2(r, z); d = np.stack([np.sin(th), np.cos(th)], 1)
    nrm = np.stack([dz_of(r), -np.ones_like(r)], 1); nrm /= np.linalg.norm(nrm, axis=1)[:, None]
    o = d - 2 * (d * nrm).sum(1)[:, None] * nrm
    return F_PX * np.tan(th), np.degrees(np.arctan2(o[:, 1], o[:, 0])), r, z - D

def designed_profile(u1_frac=0.25, phi_knee=-45.0, phi_rim=10.0, n=20000):
    """Perfil a medida: integra la ley de reflexion para un mapeo u -> phi prescrito.
    Tramo 1 (0..u1): phi -90 -> phi_knee (robot propio comprimido en el centro).
    Tramo 2 (u1..borde): phi_knee -> phi_rim lineal en pixeles (resolucion uniforme)."""
    def solve(u_rim):
        u1 = u1_frac * u_rim
        th_max = np.arctan(u_rim / F_PX); th = np.linspace(0, th_max, n)
        u = F_PX * np.tan(th)
        phi = np.where(u < u1, -90 + (phi_knee + 90) * u / u1,
                       phi_knee + (phi_rim - phi_knee) * (u - u1) / (u_rim - u1))
        g = np.cos(th + np.radians(phi)) / (1 - np.sin(th + np.radians(phi)))
        lnrho = np.log(D) + np.concatenate([[0], np.cumsum(0.5 * (g[1:] + g[:-1]) * np.diff(th))])
        rho = np.exp(lnrho)
        return rho * np.sin(th), rho * np.cos(th) - D, u, phi
    lo, hi = 50.0, 239.0              # buscar u_rim tal que el borde caiga en r = R
    for _ in range(60):
        mid = (lo + hi) / 2
        r, z, u, phi = solve(mid)
        if r[-1] < R: lo = mid
        else: hi = mid
    return solve(hi)

def budget(u, phi):
    m = (phi >= BAND[0]) & (phi <= BAND[1])
    return u.max(), (u[m].max() - u[m].min()) if m.any() else 0.0, np.interp(0, phi, u)

if __name__ == "__main__":
    a, b = 5.7419099893379455, 4.104522417157522
    hyp = (lambda r: a*np.sqrt(1+(r/b)**2)-a, lambda r: a*r/(b*b*np.sqrt(1+(r/b)**2)))
    sph = (lambda r: 12-np.sqrt(144-r*r), lambda r: r/np.sqrt(np.maximum(144-r*r,1e-9)))
    print(f"banda util {BAND[0]}..{BAND[1]} deg | D={D} mm, f={F_PX} px")
    print(f"{'perfil':34s} {'borde px':>8s} {'px banda':>8s} {'% radio':>7s} {'horiz px':>8s} {'altura mm':>9s}")
    for name, (zf, dzf) in (("A hiperboloide (a=5.74,b=4.10)", hyp), ("B esfera R=12", sph)):
        u, phi, r, z = mapping_from_profile(zf, dzf)
        rim, bp, hz = budget(u, phi)
        print(f"{name:34s} {rim:8.0f} {bp:8.0f} {100*bp/rim:6.0f}% {hz:8.0f} {z[-1]:9.2f}")
    for u1 in (0.15, 0.25, 0.35):
        r, z, u, phi = designed_profile(u1)
        rim, bp, hz = budget(u, phi)
        print(f"{'C a medida (robot en '+str(int(u1*100))+'% central)':34s} {rim:8.0f} {bp:8.0f} {100*bp/rim:6.0f}% {hz:8.0f} {z[-1]:9.2f}")
