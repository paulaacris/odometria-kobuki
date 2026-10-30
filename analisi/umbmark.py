#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 4: UMBmark (Borenstein i Feng) i calibracio.   [P] esquelet

    python3 analisi/umbmark.py <nominal|fabrica|calibrat>

Aquest esquelet ja fa la feina que no es objectiu del tema: llegir els dos
finals.csv, escriure config/kobuki_calibrat.yaml i dibuixar la figura de
dispersio. Has d'implementar les CINC funcions marcades amb TODO, que son les
formules de les diapositives 29 i 30.

Convenis: eps = real - estimat; el robot surt cap a +X; alfa i beta en GRAUS.
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")          # l'escriptori no te GPU: desem la figura
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

ARREL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTATS = os.path.join(ARREL, "resultats")
CONFIG = os.path.join(ARREL, "config")
FONT = "/odom_propi"


# ======================================================================
# Ja fet a l'esquelet
# ======================================================================
def llegir_errors(calibracio, sentit):
    cami = os.path.join(RESULTATS, "umbmark_%s_%s" % (calibracio, sentit), "finals.csv")
    if not os.path.isfile(cami):
        raise SystemExit("Falta %s\n  Executa: roslaunch pl1_cognom umbmark.launch "
                         "calibracio:=%s sentit:=%s" % (cami, calibracio, sentit))
    df = pd.read_csv(cami)
    df = df[df["font"] == FONT]
    if df.empty:
        raise SystemExit("No hi ha cap fila de %s a %s" % (FONT, cami))
    return (df["x_real"] - df["x"]).to_numpy(), (df["y_real"] - df["y"]).to_numpy()


def llegir_params(calibracio):
    cami = os.path.join(CONFIG, "kobuki_%s.yaml" % calibracio)
    if not os.path.isfile(cami):
        extra = ("\n  El genera 'python3 analisi/umbmark.py fabrica'."
                 if calibracio == "calibrat" else "")
        raise SystemExit("Falta %s%s" % (cami, extra))
    with open(cami) as f:
        return yaml.safe_load(f)


def llegir_costat():
    with open(os.path.join(CONFIG, "executor.yaml")) as f:
        return float(yaml.safe_load(f)["costat"])


def escriure_yaml(params_nous, ruta):
    sortida = {c: float(params_nous[c]) for c in
               ("radi_dreta", "radi_esquerra", "distancia_rodes", "comptes_per_volta")}
    with open(ruta, "w") as f:
        f.write("# Generat per analisi/umbmark.py. No editar a ma.\n")
        yaml.safe_dump(sortida, f, default_flow_style=False, sort_keys=False)
    print("  calibracio -> %s" % ruta)


def figura_dispersio(errors, cgs, emax, calibracio):
    fig, ax = plt.subplots(figsize=(6.5, 6))
    for sentit, color in (("cw", "tab:red"), ("ccw", "tab:blue")):
        ex, ey = errors[sentit]
        ax.plot(ex * 1000, ey * 1000, "o", color=color, alpha=0.65,
                label="quadrats %s" % sentit)
        ax.plot(cgs[sentit][0] * 1000, cgs[sentit][1] * 1000, "*", color=color,
                markersize=18, markeredgecolor="k", label="centroide %s" % sentit)
    a = np.linspace(0, 2 * np.pi, 200)
    ax.plot(emax * 1000 * np.cos(a), emax * 1000 * np.sin(a), "k--", lw=1.2,
            label="Emax,syst = %.1f mm" % (emax * 1000))
    ax.axhline(0, color="gray", lw=0.6); ax.axvline(0, color="gray", lw=0.6)
    ax.set_xlabel("error de retorn en x [mm]")
    ax.set_ylabel("error de retorn en y [mm]")
    ax.set_title("UMBmark - calibracio '%s'" % calibracio)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    desti = os.path.join(RESULTATS, "umbmark_%s.png" % calibracio)
    fig.savefig(desti, dpi=150, bbox_inches="tight")
    print("  figura -> %s" % desti)


# ======================================================================
# Les cinc funcions que completa l'alumne
# ======================================================================
def centre_gravetat(ex, ey):
    """Centroide dels errors de retorn d'un sentit (diap. 29)."""
    # TODO (diap. 29)
    raise NotImplementedError("centre_gravetat")


def emax_syst(cg_cw, cg_ccw):
    """r_cw, r_ccw i Emax,syst = max(r_cw, r_ccw) (diap. 29)."""
    # TODO (diap. 29)
    raise NotImplementedError("emax_syst")


def alfa_beta(cg_cw, cg_ccw, L):
    """alfa i beta en GRAUS (diap. 30)."""
    # TODO (diap. 30)
    raise NotImplementedError("alfa_beta")


def factors(alfa, beta, L, b):
    """R, Ed i Eb (diap. 30). beta arriba en graus: a radians abans del sinus."""
    # TODO (diap. 30)
    raise NotImplementedError("factors")


def parametres_corregits(ed, eb, params):
    """b_nou, DR i DL (diap. 30).

    MATIS: la formula de la diapositiva, DR/DL = Ed, suposa que l'odometria
    partia de dues rodes IGUALS, que es el cas de Borenstein. Aqui parteix dels
    valors de fabrica, que ja tenen una rao equivocada, i el que mesura
    l'UMBmark es la DISCREPANCIA respecte del que es creia. Per aixo Ed
    s'aplica MULTIPLICANT la rao que ja es feia servir. Si s'assigna
    directament DR/DL = Ed, la correccio no esmena l'error: el canvia de signe.
    """
    # TODO (diap. 30)
    raise NotImplementedError("parametres_corregits")


# ======================================================================
def main():
    if len(sys.argv) != 2:
        raise SystemExit("us: python3 analisi/umbmark.py <nominal|fabrica|calibrat>")
    calibracio = sys.argv[1]

    params = llegir_params(calibracio)
    L = llegir_costat()
    errors = {s: llegir_errors(calibracio, s) for s in ("cw", "ccw")}
    try:
        cgs = {s: centre_gravetat(*errors[s]) for s in ("cw", "ccw")}
        r_cw, r_ccw, emax = emax_syst(cgs["cw"], cgs["ccw"])
        alfa, beta = alfa_beta(cgs["cw"], cgs["ccw"], L)
        radi_gir, ed, eb = factors(alfa, beta, L, params["distancia_rodes"])
    except NotImplementedError as falta:
        raise SystemExit("Encara no has implementat %s(). Mira els TODO "
                         "d'aquest fitxer." % falta)

    print("UMBmark - calibracio '%s' (L = %.2f m, %d+%d quadrats)"
          % (calibracio, L, len(errors["cw"][0]), len(errors["ccw"][0])))
    print("  centroide cw  : (%+.4f, %+.4f) m   r_cw  = %.4f m"
          % (cgs["cw"][0], cgs["cw"][1], r_cw))
    print("  centroide ccw : (%+.4f, %+.4f) m   r_ccw = %.4f m"
          % (cgs["ccw"][0], cgs["ccw"][1], r_ccw))
    print("  Emax,syst     : %.4f m" % emax)
    print("  alfa          : %+.5f graus" % alfa)
    print("  beta          : %+.5f graus" % beta)
    print("  R             : %.3f m" % radi_gir)
    print("  Ed            : %.6f" % ed)
    print("  Eb            : %.6f" % eb)

    if calibracio == "fabrica":
        try:
            nous = parametres_corregits(ed, eb, params)
        except NotImplementedError as falta:
            raise SystemExit("Encara no has implementat %s()." % falta)
        print("  corregits     : radi_dreta=%.7f radi_esquerra=%.7f b=%.7f"
              % (nous["radi_dreta"], nous["radi_esquerra"], nous["distancia_rodes"]))
        escriure_yaml(nous, os.path.join(CONFIG, "kobuki_calibrat.yaml"))

    figura_dispersio(errors, cgs, emax, calibracio)


if __name__ == "__main__":
    main()
