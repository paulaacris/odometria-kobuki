# PL1 — Fitxers que et dona el professorat

Aquests vuit fitxers no formen part dels objectius del tema: te'ls donem fets perquè et
puguis concentrar en l'odometria. Són els que apareixen marcats amb **[P]** a l'apartat 1.1
de l'enunciat.

| Fitxer | Què fa | El necessites a l'apartat |
|---|---|---|
| `scripts/registre_csv.py` | Desa en CSV les odometries, la IMU i la pose real de Gazebo | 2 |
| `launch/odometria.launch` | Kobuki a Gazebo + el teu emulador + la teva odometria | 1 |
| `launch/prova.launch` | Sistema base + el teu executor + registre (dos quadrats) | 2 |
| `launch/integracio.launch` | Quatre odometries en paral·lel + executor + registre | 3 |
| `launch/umbmark.launch` | Sistema base amb la calibració triada + executor + registre | 4 |
| `launch/incertesa.launch` | Rectes repetides amb soroll a l'emulador + registre (només grups de tres) | 5 |
| `config/kobuki_fabrica.yaml` | Els paràmetres de fàbrica erronis **del teu grup** | 4 |
| `analisi/umbmark.py` | Esquelet de l'UMBmark; hi has de completar cinc funcions | 4 |

## Com els col·loques

Copia cada fitxer a la carpeta del teu paquet que té el mateix nom (hi arribes amb l'àlies
`cpl1`):

```bash
cp -r material_alumnes/* ~/catkin_ws/src/student/PL1/pl1_cognom/
```

## Important: canvia el nom del paquet

Els `.launch` i les rutes internes fan servir el nom `pl1_cognom`. El teu paquet es diu
`pl1_<el_teu_cognom>`, així que els has d'adaptar. Des de la carpeta del teu paquet:

```bash
grep -rl pl1_cognom launch/ | xargs sed -i 's/pl1_cognom/pl1_elteucognom/g'
```

Comprova que no en queda cap:

```bash
grep -r pl1_cognom launch/        # no ha de retornar res
```

Si no ho fas, `roslaunch` et dirà que no troba el paquet `pl1_cognom`.

## Comprovació ràpida

Un cop copiats i adaptats, i amb el teu emulador i la teva odometria ja escrits:

```bash
cm                                             # compila el workspace
roslaunch pl1_elteucognom odometria.launch     # ha d'obrir Gazebo amb el Kobuki
```

## Nota sobre `kobuki_fabrica.yaml`

Els valors són els del teu grup i no coincideixen amb els de ningú altre: els `Ed` i `Eb`
que ha de trobar el teu UMBmark en depenen. No els comparteixis ni els copiïs, i no els
facis servir per corregir l'odometria a mà — la gràcia de l'apartat 4 és deduir-los a
partir de les mesures.
