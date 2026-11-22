# PL1 — Odometría en robots con ruedas: Kobuki simulado

**Ampliació de Robòtica (22439) · UIB**
Autores: _Paula Cristina Morro_, _Toni_

## 1. Resumen

Hemos implementado un sistema de odometría diferencial para el Kobuki en Gazebo, conectando los nodos necesarios para estimar su pose a partir del giro de las ruedas y guiar sus recorridos:

```
/joint_states ──► encoder_emulat ──► encoders ──► odometria_propia ──► odom_propi ──► executor_recorregut
 (ángulo rueda)    (cuentas 16 bit)  (SensorState)   (x, y, θ + tf)        (Odometry)       (control P) ──► /mobile_base/commands/velocity
```

Resultado principal: con los parámetros de fábrica erróneos de nuestro grupo, el robot se desvia **417 mm (E_max,syst)** al intentar cerrar el cuadrado de 3 m. Tras aplicar el UMBmark y recalibrar, el error baja a **27 mm** (una reducción de unas 15 veces), quedándose prácticamente al mismo nivel que el error base del ejecutor con parámetros nominales (**24 mm**). 

| Calibración | Ed | Eb | E_max,syst |
|---|---|---|---|
| fábrica | 1,002347 | 1,007925 | **417,1 mm** |
| calibrado | 1,000186 | 0,999770 | **27,1 mm** |
| nominal | 0,999894 | 1,000926 | 24 mm |

## 2. Apartado 1 — Encoders y odometría

### 2.1 Cálculo de N (cuentas por vuelta de rueda)

Con los datos del fabricante (52 cuentas por vuelta del eje del motor y una reducción de 6545/132), calculamos el número de cuents por vuelta completa de la rueda:

```
N = 52 · 6545/132 = 2578,33 cuentas/vuelta 
```

Coincide con el valor por defecto del enunciado. Si aplicáramos `N = 4 · PPR · G` con PPR = 52 obtendríamos 10 313,3, cuatro veces más. Por tanto los **52 cuentas indicadas por el fabricante ya incluyen el ×4 de la cuadratura** y no hay que multiplicar otra vez.

Con un radio de rueda de r = 0,035 m, la distancia que avanza la rueda por cada cuenta del encoder es:

```
Δs = 2π · r / N = 2π · 0,035 / 2578,33 ≈ 0,0853 mm/cuenta
```

Este valor marca la resolución de la odometría:  cualquier desplazamiento de la rueda inferior a 0,085 mm no genera un cambio de cuenta y pasa desapercibido para el sistema.

### 2.2 Datos del apartado 0

Antes de implementar los nodos, inspeccionamos los topics del Kobuki en Gazebo para obtener los parámetros base necesarios para el resto de la práctica:

| Dato | Valor |
|---|---|
| Articulaciones de las ruedas | `wheel_left_joint`, `wheel_right_joint` |
| Frecuencia de `/joint_states` | _<completar con `rostopic hz`>_ Hz (el enunciado indica que la referencia de `integracio.launch` usa 100 Hz, así que debe de estar por debajo) |
| Nombre del modelo en Gazebo | `mobile_base` |

### 2.3 `encoder_emulat.py`: decisiones de diseño

- **Control de frecuencia por `header.stamp`:** En lugar de usar el reloj del sistema, comparamos el sello de tiempo de cada mensaje con el de la última publicación y solo publicamos si ha transcurrido al menos `1/freq_publicacio` (con una tolerancia de 1 µs para evitar problemas de redondeo). Si `dt < 0` (la simulación se ha reiniciado) se publica y se resincroniza para evitar bloqueos.
- **Búscada de ruedas por nombre:** Localizamos las articulaciones con `msg.name.index(...)` en lugar de usar índices fijos, ya que el orden de la lista en `/joint_states` puede variar. Si no se encuentran, se avisa con `logwarn_throttle` y se descarta el mensaje.
- **Conversión y desbordamiento de 16 bits:** Calculamos las cuentas con `n = round(φ·N/2π) + comptador_inicial` y aplicamos el módulo `n % 65536`. En Python `%` con negativos devuelve siempre un valor entre 0 y 65.535, el desbordamiento queda cubierto automáticamente tanto al avanzar como al dar marcha atrás.
- **Marca de tiempo original:** Se conserva el `header.stamp` publicado es el del `/joint_states` original, de modo que el `Δt` de la odometría refleja el tiempo de simulación real.
- **Topic de salida relativo:** Publicamos en `encoders` (sin `/` inical) para que `integracio.launch` pueda instanciar varios emuladores en distintos namespaces sin conflictos.

### 2.4 `odometria_propia.py`: decisiones de diseño

- **Ejecución por eventos (`cb_encoders`):** Todo el cálculo de la odometría se hace dentro del callback de los encoders (`cb_encoders`) y el nodo se mantiene en espera con `rospy.spin()`. En cada lectura se sigue este orden: primera lectura guarda el estado → calcula `Δt` → odena `Δn` corrigiendo el desbordamiento → calcula el avance de cada rueda → ordenar `(Δs, Δθ)` → integrar la pose → publicar.

- **Cálculo de Δt con marcas de tiempo:** Calculamos Δt a partir de la diferencia entre el `header.stamp` actual y el anterior, nunca con `rospy.Time.now()`. Si `Δt ≤ 0`,  el nodo vuelve al estado de primera lectura manteniendo la pose actual, lanza un aviso y descarta el paso para evitar cálculos erróneos.

- **Corrección del desbordamiento:** Calculamos `Δn = n − n_ant` y lo ajustamos al rango al rango `[−32768, 32767]` sumando o restando 65 536. Esto garantiza que el salto de 16 bits no afecte al cálculo mientras la rueda no gire más de media vuelta del contador entre dos mensajes, algo imposible a las velocidades del Kobuki.

- **Radios independientes por rueda:**
```
Δs_R = r_R · 2π·Δn_R / N        Δs_L = r_L · 2π·Δn_L / N
Δs = (Δs_R + Δs_L) / 2          Δθ = (Δs_R − Δs_L) / b
```

Mantener r_R y r_L por separado es lo que permite corregir posteriormente el error de diámetros (`E_d`) junto con la distancia entre ruedas (`E_b$`) durante la calibración del apartado 4. 

- **Métodos de integración:** Implementamos `euler`, `punt_mig` y `exacta` con la misma estructura `(pose, Δs, Δθ) → pose` y normalizando siempree θ al rango [−π, π] con `atan2(sin, cos)`:

| Método | Pose nueva |
|---|---|
| Euler | `x + Δs·cos θ`, `y + Δs·sin θ`, `θ + Δθ` |
| Punto medio | igual, pero con `θ + Δθ/2` |
| Exacta | avanza la cuerda `c = 2·(Δs/Δθ)·sin(Δθ/2)` con `θ + Δθ/2`; si `|Δθ| < llindar_dtheta`, `c = Δs` |

El umbral `llindar_dtheta` evita la división por cero cuando el robot avanza en linea recta, haciendo que en ese caso el método exacto se comporte igual que el punto medio.

- **Seguridad en concurrencia y servicio `~reset`:** Como el callback de encoders y el servicio`~reset` se ejecutan en hilos distintos, protegemos las variables de estado con `threading.Lock`. Al llamar a `~reset` pone `x = y = θ = 0` pero **conserva los últimos contadores leidos**. Si se borraran, el siguiente mensaje generaría un `Δn` artificialmente grande y la pose daría un salto.
- **Cuaterniones y marcos de referencia (`tf`):** Internamente trabajamos siempre con `(x, y, θ)` en radianes y solo convertimos a quaternion (`quaternion_from_euler(0, 0, θ)`) al publicar en `odom_propi` y el `tf`. Usamos los marcos `odom_propi`y `base_footprint_propi` para no colisionar con los del simulador, y el parámetro `~publicar_tf` permite desactivar la publicación de`tf` en las instancias extra de `integracio.launch`.

- **Matriz de confianza:** Publicamos una matriz diagonal constante con valores razonados a partir de las pruebas:

| Elemento | Valor | Razonamiento |
|---|---|---|
| `xx`, `yy` | 1·10⁻³ m² | Equivale a σ ≈ 3,2 cm, , del orden de magnitud de nuestro error de fondo en el cuadrado de 3 m (2-3 cm) |
| `yaw` | 1·10⁻² rad² | Equivale a σ ≈ 0,1 rad ≈ 5,7°, un margen conservador frente al error angular máximo medido con parámetros de fábrica (≈4,6°) |
| `zz`, `roll`, `pitch` | 1·10⁶ | El robot se mueve en un plano 2D y no estima estas variables. Un valor muy alto indica "sin información" según la convención de ROS. |

Esta covarianza es una estimación estática (no crece con la distancia ni incluye correlaciones cruzadas), una limitación que retomamos en las conclusiones.

### 2.5 Verificación

Para validar tanto el emulador como el nodo de odometría, lanzamos `odometria.launch` junto con `kobuki_keyop` y realizamos las siguientes comprobaciones:

- **Contadores de los encoders:** Mediante rostopic echo /encoders, verificamos que ambos contadores crecen al avanzar en línea recta y que, al girar sobre sí mismo, uno aumenta mientras el otro disminuye.
- **Comparación de trayectorias:** Al realizar una recta, un giro en el sitio y una curva, comprobamos en `rqt_plot` que las coordenadas `x` e `y` de `/odom_propi` coinciden con las de `/odom`.  
- **Visualización en RViz:** Configurando `odom_propi` como Fixed Frame, confirmamos que el marco `base_footprint_propi` sigue al robot.
- **Desbordamiento de 16 bits:** Lanzando el sistema con `comptador_inicial:=65000`, comprobamos que los contadores pasan limpiamente de 65.,535 a 0 sin provocar saltos en la odometría.
-  **Reinicio de la pose:** Al llamar a `rosservice call /odometria_propia/reset`, la odometría vuelve a (0, 0, 0) y sigue integrando sin saltos en las lecturas posteriores.   


> **Capturas pendientes de añadir:** `rqt_graph` (nodos y topics del apartado 1) y rviz con los dos marcos.
> `![rqt_graph](resultats/captura_rqt_graph.png)` · `![rviz](resultats/captura_rviz.png)`

## 3. Apartado 2 — Ejecutor de recorridos

### 3.1 Diseño

`executor_recorregut.py` funciona como una máquina de estados que recorre una lista de segmentos `(tipo, valor)` generada a partir de `~tipus` y `~sentit`:

| Tipo | Secuencia |
|---|---|
| cuadrado ccw / cw | 4 × [recta de `costat` m, giro ±90°] |
| recta | una recta de `longitud_recta` m |
| arco | un arco de `±angle_arc`° (positiva si es ccw) |

Todos los segmentos se ejecutan en un bucle a **50 Hz** y entre segmentos el robot se detiene 0,5 s.

- **Recta:** Guarda posición y rumbo al empezar el tramo. La distancia recorrida se calcula como la euclídea desde el punto inicial y termina cuando `recorrido ≥ d − tol_dist`. Durante el avance se corrige el rumbo con `ω = kp·e` (saturada a `±w_max`), donde `e = normaliza(rumbo_inicial − θ)`.
- **Giro:** El robot gira en el sitio (`v = 0`) aplicando `ω = kp·e` (saturada a `±w_max`), con `e = normaliza(objetivo − θ)`. Termina con `|e| < tol_angle`.
- **Arco:** Se conduce en **lazo abierto** con velocidades constantes `v = v_max`, `ω = v_max/radi_arc` (saturada a `w_max` y con signo según el sentido). No se aplica control de rumbo porque corregiría la curvatura que precisamente queremos medir. Para determinar el final del arco se acumulan incrementos normalizados de θ en lugar de comparar con un ángulo objetivo, evitando así la ambigüedad en giros de 180º o más (donde +180º y -180º son el mismo ángulo).

### 3.2 Decisiones de diseño

- **Control exclusivo con `odom_propi`:** El controlador no lee nunca `/odom` ni `/gazebo/model_states`. Esto es imprescindible para el UMBmark, ya que el método mide precisamente el error de un robot que confía en su propia odometría aunque esté mal calibrada.
- **Sincronización tras el reset.** En `preparar_repeticio()`, el nodo lleva la cuenta de los mensajes recibidos (`n_odom`) y se espera a recibir uno posterior al reset (con un timeout de 2 s). SDe este modo evitamos que `self.x/y/theta` conserven el valor del recorrido anterior y falseen el punto de partida del primer tramo.
- **Parada de emergencia.** Como `rospy.signal_shutdown` no es instantáneo, utilizamos una bandera (`emergencia`) que obliga a salir inmediatamente de todos los bucles (`ok()`) cuando salta el parachoques (`BumperEvent.PRESSED`). Además, `rospy.on_shutdown(aturar)` asegura que se publique velocidad cero siempre que el nodo termine, sea cual sea el motivo.
- **Ciclo de repeticiones.** Para cada iteración `n`, el nodo ejecuta esta secuencia: recolocar el robot con`set_model_state` (pose, orientación y velocidades a cero) → pausa → `reset` de todas las odometrías de `~serveis_reset` → espera de odometría nueva → pubica `inici n` → recorre los segmentos → se detiene y espera la pausa → publica `fi n`. Al terminar la última repetición cierra el nodo con `signal_shutdown`.
- **Parámetros externalizados:** Todos loas valores numéricos de ajuste (velocidades, tolerancias, `kp`) se leen de `config/executor.yaml`, dejando en el código solo tres constantes de implementación (`FREQ_CONTROL`, `PAUSA_ENTRE_SEGMENTS`, `TIMEOUT_ODOM`).

### 3.3 Prueba

Al lanzar `prova.launch` hace dos cuadrados seguidos y genera `finals.csv`, `serie_1.csv`, `serie_2.csv` dentro de `resultats/prova`. Con parámetros nominales, el resultado final de `/odom_propi` queda a ≈3 cm del origen, y el yaw de la IMU coincide con el `theta_real` de Gazebo (diferencias de ≈0,01°), lo que confirma que los datos registrados en los CSV son coherentes. 

Esta prueba ya adelanta una observación clave del apartado 4: al acabar el cuadrado, nuestra odometría estima una orientación final de θ ≈ -0,016 rad, mientras que tanto Gazebo como la IMU registran un valor real de entre -0,02 y -0,05 rad.   
