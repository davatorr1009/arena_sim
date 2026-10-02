#!/usr/bin/env python3
"""Visualizador 2D interactivo de perfiles de espejo para QUPA (sin renderizar).

Paneles:
  1. Corte del espejo con la camara, el robot y el abanico de rayos reflejados
     (rojo = el rayo choca con el propio robot).
  2. Mapeo radio en la imagen (px) -> elevacion (deg), con el espejo actual como referencia.
  3. Imagen 640x480 sintetica: piso con anillos cada 10 cm y sectores cada 30 deg,
     paredes de la arena (cafe), robots de prueba y el propio robot (gris).

Uso:
    python3 scripts/mirror_designer.py                  # ventana interactiva
    python3 scripts/mirror_designer.py --perfil esfera  # arrancar con otro perfil
    python3 scripts/mirror_designer.py --png salida.png # guardar sin abrir ventana

Requiere numpy y matplotlib.
"""
import argparse
import datetime
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mirror_profiles as mp  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent

# ----------------------------------------------------------------------------
# Parametros fijos de la escena (mm salvo que se indique)
# ----------------------------------------------------------------------------
CAM_Z = 140.0                 # centro optico sobre el piso (base_link)
IMG_W, IMG_H = 640, 480
IMG_CENTER = (319.5, 239.5)   # centro del espejo en la imagen (la foto real da (290, 262))
REF_A = dict(a=5.7419, b=4.1045)   # espejo actual (referencia)
FLANGE_R, FLANGE_T = 12.7, 1.0
WALL_H = 300.0                # altura de las paredes de la arena
TEST_ROBOTS = [               # (distancia m, azimut deg, color RGB) robots de prueba
    (0.30, 0, (0.85, 0.15, 0.10)),
    (0.60, 120, (0.15, 0.35, 0.85)),
    (1.00, 240, (0.15, 0.65, 0.20)),
]
ROBOT_R, ROBOT_H = 61.5, 138.0

PERFILES = {"hiperboloide": "hyperbolic", "hiperboloide SVP": "hyperbolic_svp",
            "esfera": "spherical", "a medida": "custom"}

DEFAULTS = dict(D=8.3, R=12.0, vfov=68.7, a=5.7419, b=4.1045, H=12.0, Rs=12.0,
                knee=-30.0, top=5.0, center_frac=0.15, u_top=212.0, wall=1.0)

SLIDERS = [  # (clave, etiqueta, min, max, paso, perfiles donde aplica o None = todos)
    ("D", "D camara-vertice [mm]", 4.0, 40.0, 0.1, None),
    ("R", "radio espejo R [mm]", 6.0, 15.0, 0.1, None),
    ("vfov", "FOV vertical [deg]", 30.0, 90.0, 0.1, None),
    ("wall", "distancia a pared [m]", 0.2, 2.5, 0.05, None),
    ("a", "hiperb. a [mm]", 0.5, 20.0, 0.01, ("hyperbolic",)),
    ("b", "hiperb. b [mm]", 0.5, 20.0, 0.01, ("hyperbolic",)),
    ("H", "SVP altura H [mm]", 2.0, 20.0, 0.1, ("hyperbolic_svp",)),
    ("Rs", "esfera radio Rs [mm]", 6.0, 40.0, 0.1, ("spherical",)),
    ("knee", "a medida: codo [deg]", -80.0, -20.0, 0.5, ("custom",)),
    ("top", "a medida: tope [deg]", -10.0, 40.0, 0.5, ("custom",)),
    ("center_frac", "a medida: centro robot", 0.05, 0.5, 0.01, ("custom",)),
    ("u_top", "a medida: u tope [px]", 100.0, 260.0, 1.0, ("custom",)),
]


# ----------------------------------------------------------------------------
# Geometria
# ----------------------------------------------------------------------------
def load_robot_silhouette():
    """r maximo del cuerpo del robot por cada mm de altura (desde qupa.obj)."""
    path = ROOT / "assets" / "qupa_meshes" / "qupa.obj"
    zs = np.arange(0, 141)
    if not path.exists():
        return zs, np.where(zs < 90, ROBOT_R, np.sqrt(np.maximum(0, 60**2 - (zs - 78) ** 2)))
    v = np.array([ln.split()[1:4] for ln in open(path) if ln.startswith("v ")], float) * 1000
    r, z = np.hypot(v[:, 0], v[:, 1]), v[:, 2]
    rmax = np.zeros(len(zs))
    idx = np.clip(z.astype(int), 0, len(zs) - 1)
    np.maximum.at(rmax, idx, r)
    # rellenar milimetros sin vertices con el maximo de los vecinos (envolvente)
    pad = np.pad(rmax, 3, mode="edge")
    rmax = np.max(np.stack([pad[i:i + len(rmax)] for i in range(7)]), 0)
    return zs, rmax


SIL_Z, SIL_R = load_robot_silhouette()


def params_dict(vals):
    p = dict(vals)
    p["f_px"] = mp.f_px_from_vfov(p["vfov"], IMG_H)
    return p


def analyse(kind, vals):
    p = params_dict(vals)
    r, z, s = mp.make_profile(kind, p)
    u, phi, o = mp.trace(r, z, s, p["D"], p["f_px"])
    # origen de cada rayo reflejado en coordenadas del robot (r, z sobre el piso)
    pr, pz = r, CAM_Z + p["D"] + z
    # choque con el propio robot: muestrear el rayo reflejado
    t = np.linspace(0, 200, 400)[None, :]
    rr = np.abs(pr[:, None] + o[:, 0:1] * t)
    zz = pz[:, None] + o[:, 1:2] * t
    inside = (zz >= 0) & (zz < SIL_Z[-1])
    sil = SIL_R[np.clip(zz.astype(int), 0, len(SIL_R) - 1)]
    hit = inside & (rr < sil)
    occl = hit.any(1)
    hit_t = np.where(occl, t[0, np.argmax(hit, 1)], np.nan)
    return dict(p=p, r=r, z=z, s=s, u=u, phi=phi, o=o, pr=pr, pz=pz, occl=occl, hit_t=hit_t)


def stats(a):
    u, phi, occl = a["u"], np.degrees(a["phi"]), a["occl"]
    frame = min(IMG_CENTER[0], IMG_W - IMG_CENTER[0], IMG_CENTER[1], IMG_H - IMG_CENTER[1])
    u_rim = u[-1]
    self_px = u[occl].max() if occl.any() else 0.0
    vis = (~occl) & (phi <= 10) & (u <= frame)
    useful = u[vis].max() - u[vis].min() if vis.any() else 0.0
    below = (~occl) & (phi < 0) & (u <= frame)
    below_px = u[below].max() - u[below].min() if below.any() else 0.0
    return dict(u_rim=u_rim, frame=frame, self_px=self_px, useful=useful, below=below_px,
                H=a["z"][-1], phi_rim=phi[-1], horizon=np.interp(0, phi, u) if phi.min() < 0 < phi.max() else np.nan)


def synth_image(a, wall_m):
    """Imagen sintetica de lo que ve la camara a traves del espejo."""
    cx, cy = IMG_CENTER
    Y, X = np.mgrid[0:IMG_H, 0:IMG_W].astype(float)
    dx, dy = X - cx, Y - cy
    u = np.hypot(dx, dy)
    az = np.degrees(np.arctan2(dx, -dy)) % 360          # 0 = arriba en la imagen (frente)
    img = np.zeros((IMG_H, IMG_W, 3))
    img[:] = (0.08, 0.08, 0.08)                          # fuera del espejo
    ua = a["u"]
    on = u <= ua[-1]
    ui = u[on]
    phi = np.interp(ui, ua, a["phi"])
    pr = np.interp(ui, ua, a["pr"])
    pz = np.interp(ui, ua, a["pz"])
    occl = np.interp(ui, ua, a["occl"].astype(float)) > 0.5
    azi = az[on]
    tanp = np.tan(phi)
    col = np.zeros((len(ui), 3))
    col[:] = (0.80, 0.76, 0.68)                          # por encima de la pared
    W = wall_m * 1000
    # pared: altura a la que el rayo llega a la distancia W
    hw = pz + (W - pr) * tanp
    wall = (hw >= 0) & (hw <= WALL_H)
    stripe = (np.floor(hw / 50) % 2) == 0
    col[wall] = np.where(stripe[wall, None], (0.42, 0.26, 0.13), (0.36, 0.22, 0.11))
    # piso: distancia horizontal donde el rayo toca z=0
    with np.errstate(divide="ignore", invalid="ignore"):
        dfloor = pr + pz / np.where(tanp < 0, -tanp, np.nan)
    floor = np.isfinite(dfloor) & (dfloor < W)
    ring = (np.floor(dfloor / 100) % 2) == 0
    sect = (np.floor(azi / 30) % 2) == 0
    shade = np.where(ring ^ sect, 0.93, 0.80)
    col[floor] = np.stack([shade, shade, shade * 0.97], 1)[floor]
    # robots de prueba (cilindros)
    for d_m, az0, c in TEST_ROBOTS:
        d = d_m * 1000
        dang = np.radians((azi - az0 + 180) % 360 - 180)
        disc = ROBOT_R ** 2 - (d * np.sin(dang)) ** 2
        ok = (disc > 0) & (np.cos(dang) > 0)
        sq = np.sqrt(np.maximum(disc, 0))
        s1, s2 = d * np.cos(dang) - sq, d * np.cos(dang) + sq      # entrada / salida del cilindro
        h1, h2 = pz + (s1 - pr) * tanp, pz + (s2 - pr) * tanp
        side = ok & (h1 >= 0) & (h1 <= ROBOT_H)
        lid = ok & (h1 > ROBOT_H) & (h2 <= ROBOT_H)                 # tapa vista desde arriba
        before_wall = s1 < W
        col[side & before_wall] = c
        col[lid & before_wall] = 0.6 * np.array(c) + 0.4
    col[occl] = (0.55, 0.55, 0.58)                       # propio robot
    img[on] = col
    return img


# ----------------------------------------------------------------------------
# Dibujo
# ----------------------------------------------------------------------------
def draw(fig, axes, kind, vals, ref):
    ax_p, ax_m, ax_i = axes
    for ax in axes:
        ax.clear()
    a = analyse(kind, vals)
    st = stats(a)
    p = a["p"]
    D = p["D"]

    # --- corte ---
    ax_p.fill_betweenx(SIL_Z, -SIL_R, SIL_R, color="0.85", zorder=0, label="robot (qupa.obj)")
    zm = CAM_Z + D + a["z"]
    ax_p.plot(a["r"], zm, "k", lw=2)
    ax_p.plot(-a["r"], zm, "k", lw=2, label="espejo")
    top_z = CAM_Z + D + a["z"][-1]
    ax_p.add_patch(__import__("matplotlib").patches.Rectangle(
        (-FLANGE_R, top_z), 2 * FLANGE_R, FLANGE_T, color="0.4"))
    if ref is not None:
        ax_p.plot(ref["r"], CAM_Z + D + ref["z"], "--", color="tab:orange", lw=1)
        ax_p.plot(-ref["r"], CAM_Z + D + ref["z"], "--", color="tab:orange", lw=1, label="espejo actual (A)")
    ax_p.plot(0, CAM_Z, "o", color="tab:green", ms=6, label="camara")
    import matplotlib.cm as cm
    cmap = cm.get_cmap("viridis")
    idx = np.unique(np.searchsorted(a["u"], np.linspace(1, min(a["u"][-1], st["frame"]), 22)))
    idx = idx[idx < len(a["u"])]
    for i in idx:
        x0, z0 = a["pr"][i], a["pz"][i]
        ax_p.plot([0, x0], [CAM_Z, z0], color="0.6", lw=0.5)
        L = a["hit_t"][i] if a["occl"][i] else 120
        c = "red" if a["occl"][i] else cmap((np.degrees(a["phi"][i]) + 90) / 130)
        ax_p.plot([x0, x0 + a["o"][i, 0] * L], [z0, z0 + a["o"][i, 1] * L], color=c, lw=1)
    ax_p.set_xlim(-75, 75)
    ax_p.set_ylim(80, top_z + 20)
    ax_p.set_aspect("equal")
    ax_p.set_xlabel("r [mm]")
    ax_p.set_ylabel("z sobre el piso [mm]")
    ax_p.set_title(f"Corte del espejo: altura {st['H']:.2f} mm, D = {D:.1f} mm")
    ax_p.legend(loc="lower left", fontsize=7)
    ax_p.grid(alpha=0.3)

    # --- mapeo ---
    ph = np.degrees(a["phi"])
    ax_m.plot(a["u"], np.where(a["occl"], np.nan, ph), "k", lw=2, label="perfil")
    ax_m.plot(a["u"], np.where(a["occl"], ph, np.nan), color="red", lw=2, label="ve al propio robot")
    if ref is not None:
        ax_m.plot(ref["u"], np.degrees(ref["phi"]), "--", color="tab:orange", label="espejo actual (A)")
    ax_m.axhspan(-90, 0, color="tab:green", alpha=0.06)
    ax_m.axhline(0, color="0.5", lw=0.8)
    ax_m.axvline(st["frame"], color="tab:purple", ls=":", label=f"limite del cuadro ({st['frame']:.0f} px)")
    ax_m.set_xlim(0, 260)
    ax_m.set_ylim(-90, 60)
    ax_m.set_xlabel("radio en la imagen [px]")
    ax_m.set_ylabel("elevacion [deg]")
    ax_m.set_title("Mapeo imagen -> elevacion")
    ax_m.legend(loc="upper left", fontsize=7)
    ax_m.grid(alpha=0.3)
    txt = (f"borde espejo: {st['u_rim']:.0f} px   horizonte: {st['horizon']:.0f} px\n"
           f"propio robot: 0-{st['self_px']:.0f} px   bajo horizonte util: {st['below']:.0f} px\n"
           f"util (sin robot, <=10 deg, en cuadro): {st['useful']:.0f} px   "
           f"elev. borde: {st['phi_rim']:.1f} deg")
    ax_m.text(0.99, 0.02, txt, transform=ax_m.transAxes, ha="right", va="bottom", fontsize=7,
              bbox=dict(fc="white", alpha=0.85, lw=0))

    # --- imagen sintetica ---
    ax_i.imshow(synth_image(a, vals["wall"]), origin="upper")
    ax_i.set_title(f"Vista sintetica 640x480\npared a {vals['wall']:.2f} m, robots a 0.3 / 0.6 / 1.0 m", fontsize=10)
    ax_i.set_xticks([])
    ax_i.set_yticks([])
    fig.canvas.draw_idle()
    return a


def reference(vals):
    return analyse("hyperbolic", {**vals, **REF_A})


def export_csv(kind, vals):
    a = analyse(kind, vals)
    out = ROOT / "analysis"
    out.mkdir(exist_ok=True)
    fn = out / f"perfil_{kind}_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
    rs = np.arange(0, a["r"][-1] + 1e-9, 0.1)
    with open(fn, "w") as f:
        f.write("# " + ", ".join(f"{k}={v:.4g}" for k, v in sorted(vals.items())) + f", perfil={kind}\n")
        f.write("r_mm,z_mm,pendiente_dz_dr,u_px,elevacion_deg\n")
        for r in rs:
            f.write(f"{r:.3f},{np.interp(r, a['r'], a['z']):.4f},{np.interp(r, a['r'], a['s']):.5f},"
                    f"{np.interp(r, a['r'], a['u']):.2f},{np.degrees(np.interp(r, a['r'], a['phi'])):.3f}\n")
    print("exportado:", fn)
    return fn


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--perfil", choices=list(PERFILES), default="a medida")
    ap.add_argument("--png", help="guardar la figura en este archivo y salir (sin ventana)")
    for k in DEFAULTS:
        ap.add_argument(f"--{k}", type=float, default=DEFAULTS[k])
    args = ap.parse_args()

    import matplotlib
    if args.png:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, Slider

    vals = {k: getattr(args, k) for k in DEFAULTS}
    state = {"kind": PERFILES[args.perfil]}

    fig = plt.figure(figsize=(16, 9.5))
    ax_p = fig.add_axes([0.04, 0.36, 0.34, 0.60])
    ax_m = fig.add_axes([0.43, 0.62, 0.25, 0.34])
    ax_i = fig.add_axes([0.70, 0.52, 0.29, 0.44])
    axes = (ax_p, ax_m, ax_i)

    if args.png:
        draw(fig, axes, state["kind"], vals, reference(vals))
        fig.savefig(args.png, dpi=90, bbox_inches="tight")
        print("guardado:", args.png)
        return

    rax = fig.add_axes([0.43, 0.36, 0.12, 0.2])
    radio = RadioButtons(rax, list(PERFILES), active=list(PERFILES).index(args.perfil))
    rax.set_title("Perfil", fontsize=9)

    sliders = {}
    for i, (k, label, lo, hi, step, _) in enumerate(SLIDERS):
        col, row = divmod(i, 6)
        sax = fig.add_axes([0.14 + col * 0.49, 0.27 - row * 0.042, 0.27, 0.025])
        sliders[k] = Slider(sax, label, lo, hi, valinit=vals[k], valstep=step)

    def refresh(_=None):
        for k, s in sliders.items():
            vals[k] = s.val
        applies = {k: a for k, *_, a in SLIDERS}
        for k, s in sliders.items():
            active = applies[k] is None or state["kind"] in applies[k]
            s.ax.set_alpha(1.0 if active else 0.3)
            s.label.set_color("black" if active else "0.7")
        try:
            draw(fig, axes, state["kind"], vals, reference(vals))
        except Exception as e:  # parametros imposibles (p. ej. SVP sin solucion)
            ax_m.set_title(f"parametros invalidos: {e}", color="red")
            fig.canvas.draw_idle()

    def on_radio(label):
        state["kind"] = PERFILES[label]
        refresh()

    radio.on_clicked(on_radio)
    for s in sliders.values():
        s.on_changed(refresh)

    bexp = Button(fig.add_axes([0.57, 0.46, 0.1, 0.045]), "Exportar CSV")
    bexp.on_clicked(lambda _: export_csv(state["kind"], vals))
    breset = Button(fig.add_axes([0.57, 0.40, 0.1, 0.045]), "Restablecer")
    breset.on_clicked(lambda _: [s.reset() for s in sliders.values()])

    refresh()
    plt.show()


if __name__ == "__main__":
    main()
