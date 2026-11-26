#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 3: comparacio dels metodes d'integracio.

    python3 analisi/integracio.py <freq>        (p. ex. 2, 4 o 5)

Llegeix resultats/integracio_<freq>/finals.csv, calcula l'error de posicio final
de cada metode respecte de l'odometria de referencia /ref/odom_propi i desa la
taula a resultats/integracio_taula_<freq>.csv.
"""
import os
import sys

import numpy as np
import pandas as pd

# Carpeta del paquete (la que contiene analisi/, config/, resultats/...).
# __file__ es este script, dirname una vez da analisi/ y dos veces el paquete.
ARREL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTATS = os.path.join(ARREL, "resultats")

# Nombre de las fuentes tal y como las escribe registre_csv.py en la columna
# "font" (son los topics de odometria que lista integracio.launch).
REFERENCIA = "/ref/odom_propi"
METODES = {
    "euler": "/lenta/euler/odom_propi",
    "punt_mig": "/lenta/punt_mig/odom_propi",
    "exacta": "/lenta/exacta/odom_propi",
}


def llegir_finals(ruta):
    """Comprueba que existe finals.csv y lo carga como DataFrame de pandas.
    El fichero tiene una fila por fuente y por recorrido (columna 'run')."""
    if not os.path.isfile(ruta):
        raise SystemExit("Falta %s\n  Executa abans: roslaunch pl1_COGNOM integracio.launch "
                         "freq:=<freq>" % ruta)
    return pd.read_csv(ruta)


def error_final(df):
    """Para cada recorrido (run) y cada metodo, calcula la distancia entre la
    posicion final del metodo y la de la referencia /ref/odom_propi.

    Se compara con /ref y no con la pose real de Gazebo porque las cuatro
    odometrias siguen al mismo robot: el deslizamiento real de las ruedas les
    afecta a todas por igual, asi que al restar la referencia solo queda el error
    que anade el metodo de integracion.

    Devuelve un DataFrame con tres columnas: run, metode, error_m."""
    # Posicion final (x, y) de la referencia, indexada por recorrido para poder
    # emparejarla con la de cada metodo.
    ref = df[df["font"] == REFERENCIA].set_index("run")[["x", "y"]]
    if ref.empty:
        raise SystemExit("No hi ha cap fila de %s a finals.csv" % REFERENCIA)

    files = []
    for metode, font in METODES.items():
        # Nos quedamos solo con las filas de esta fuente.
        m = df[df["font"] == font].set_index("run")[["x", "y"]]
        if m.empty:
            raise SystemExit("No hi ha cap fila de %s a finals.csv" % font)
        # Solo comparamos los recorridos que existen en las dos fuentes.
        comu = m.index.intersection(ref.index)
        # Pitagoras: raiz de (x - x_ref)^2 + (y - y_ref)^2, en metros.
        err = np.hypot(m.loc[comu, "x"] - ref.loc[comu, "x"],
                       m.loc[comu, "y"] - ref.loc[comu, "y"])
        for run, e in err.items():
            files.append({"run": run, "metode": metode, "error_m": e})
    return pd.DataFrame(files)


def taula_metodes(err):
    """Resume el error de cada metodo: media de los recorridos (si solo hay uno,
    es ese valor), pasada a milimetros y ordenada de mayor a menor error."""
    t = err.groupby("metode")["error_m"].mean().to_frame("error_m")
    t["error_mm"] = t["error_m"] * 1000.0
    t = t.sort_values("error_m", ascending=False)
    return t[["error_mm"]].reset_index()


def comparar_freqs():
    """Si ya se han generado las tablas de 2 y 4 Hz, calcula el cociente
    error(2 Hz) / error(4 Hz) de cada metodo. Segun la teoria (diap. 21, 22 y 24)
    deberia salir ~2 para Euler (metodo de orden 1) y ~4 para el punto medio
    (orden 2): al doblar la frecuencia el error baja 2 y 4 veces."""
    t2 = os.path.join(RESULTATS, "integracio_taula_2.csv")
    t4 = os.path.join(RESULTATS, "integracio_taula_4.csv")
    if not (os.path.isfile(t2) and os.path.isfile(t4)):
        return
    a = pd.read_csv(t2).set_index("metode")["error_mm"]
    b = pd.read_csv(t4).set_index("metode")["error_mm"]
    print("\nQuocient error(2 Hz) / error(4 Hz)   (esperat: euler ~2, punt_mig ~4)")
    for m in METODES:
        print("  %-9s %.2f" % (m, a[m] / b[m]))


def main():
    # La frecuencia se pasa como argumento: python3 analisi/integracio.py 5
    if len(sys.argv) != 2:
        raise SystemExit("Us: python3 analisi/integracio.py <freq>")
    freq = sys.argv[1]

    df = llegir_finals(os.path.join(RESULTATS, "integracio_%s" % freq, "finals.csv"))
    taula = taula_metodes(error_final(df))

    print("\nError final respecte de %s  (freq = %s Hz)\n" % (REFERENCIA, freq))
    print(taula.to_string(index=False, float_format=lambda v: "%.4f" % v))

    # La tabla se guarda para usarla en la memoria y para el cociente 2 Hz / 4 Hz.
    sortida = os.path.join(RESULTATS, "integracio_taula_%s.csv" % freq)
    taula.to_csv(sortida, index=False)
    print("\nTaula desada a %s" % sortida)

    comparar_freqs()


if __name__ == "__main__":
    main()