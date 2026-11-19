#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ARREL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTATS = os.path.join(ARREL, "resultats")
SERIES = {"fabrica": "umbmark_fabrica_ccw", "calibrat": "umbmark_calibrat_ccw"}
FONTS = {"/odom_propi": "odometria pròpia", "imu": "IMU"}


def llegir_serie(ruta):
    """ Comprueba si el archivo CSV existe"""
    if not os.path.isfile(ruta):
        raise SystemExit("Falta %s" % ruta)
    return pd.read_csv(ruta) # archivo cargado como un dataframe de pandas 


def error_orientacio(serie, font):
    d = serie[serie["font"] == font] # nso quedamos solo con las filas que pertenecen a la fuente
    e = d["theta_real"].to_numpy() - d["theta"].to_numpy() # error = orientacion estimada - orientación real Gazebo
    e = np.arctan2(np.sin(e), np.cos(e)) # error angular normalizado 
    return d["t"].to_numpy(), e 


def figura(series):
    """crea y guarda una imagen con dos gráficos, 
    el de la izqauierda muestra los errores sin calibrar y el de la derecha ya calibrados"""
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), sharey=True) 
    for ax, (cal, serie) in zip(axs, series.items()):
        for font, nom in FONTS.items():
            t, e = error_orientacio(serie, font)
            ax.plot(t, np.degrees(e), label=nom)
        ax.set_title("calibració '%s' (ccw, sèrie 1)" % cal)
        ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend()
    axs[0].set_ylabel("error de θ (real − estimat) [°]")
    desti = os.path.join(RESULTATS, "orientacio_imu_odom.png")
    fig.savefig(desti, dpi=150, bbox_inches="tight")
    print("figura ->", desti)


def main():
    """carga los datos, imprime un resumen y llama a figura() para guardar la imagen"""
    series = {c: llegir_serie(os.path.join(RESULTATS, d, "serie_1.csv"))
              for c, d in SERIES.items()}
    for c, s in series.items():
        for font in FONTS:
            _, e = error_orientacio(s, font)
            print("%-9s %-12s error final = %+.3f°  |max| = %.3f°"
                  % (c, font, np.degrees(e[-1]), np.degrees(np.abs(e).max())))
    figura(series)


if __name__ == "__main__":
    main()