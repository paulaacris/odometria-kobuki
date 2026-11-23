#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 1.1 - Emulador d'encoders.

Converteix l'angle acumulat de cada roda (/joint_states, radians, continu) en
comptes enters de 16 bits (0..65535, amb desbordament) i els publica amb el mateix
missatge que el Kobuki real (kobuki_msgs/SensorSta te). Aixi odometria_propia.py
treballa com amb encoders reals.
"""
# math: nos hace falta solo para pi en la conversion angulo -> cuentas.
import math

# rospy: libreria de ROS 1 para Python (nodos, parametros, topics, logs...).
import rospy
# SensorState es el mensaje del Kobuki real que lleva los contadores de los
# encoders (left_encoder y right_encoder). Lo usamos para SALIR igual que el robot.
from kobuki_msgs.msg import SensorState
# JointState es lo que publica Gazebo: el angulo acumulado de cada rueda (rad).
from sensor_msgs.msg import JointState

# Los contadores del Kobuki son de 16 bits: van de 0 a 65535 y despues de 65535
# vuelven a 0. 2^16 = 65536 es el "modulo" con el que emulamos ese desbordamiento.
MODUL = 65536          # 2^16: el comptador torna a 0 despres del 65535
# Margen para comparar tiempos en coma flotante: evita que un 0.02 s "exacto"
# falle por errores de redondeo de una millonesima.
TOLERANCIA = 1e-6      # s, per evitar problemes d'arrodoniment en comparar temps


class EncoderEmulat(object):

    def __init__(self):
        # --- Parametres privats (valors per defecte de l'enunciat) ---
        # Todos los parametros son privados (~) y con valor por defecto, para que el
        # launch pueda cambiarlos sin tocar el codigo. Nada de numeros fijos aqui.

        # Nombres de las dos ruedas en /joint_states (los vimos con rostopic echo).
        self.roda_dreta = rospy.get_param("~roda_dreta", "wheel_right_joint")
        self.roda_esquerra = rospy.get_param("~roda_esquerra", "wheel_left_joint")
        # N = cuentas por vuelta COMPLETA DE RUEDA = 52 * 6545/132 = 2578.33.
        # No es entero, y no se redondea. float() por si el yaml lo trae como texto.
        self.N = float(rospy.get_param("~comptes_per_volta", 2578.33))
        # Valor del contador cuando la rueda esta en angulo 0. Con 65000 forzamos el
        # desbordamiento sin tener que dar muchas vueltas (sirve para probarlo).
        self.comptador_inicial = int(rospy.get_param("~comptador_inicial", 0))
        # Frecuencia a la que queremos publicar. Si es 0 o negativa, periodo 0 =
        # publicar todo lo que llegue (evita dividir por cero).
        freq = float(rospy.get_param("~freq_publicacio", 50.0))
        self.periode = 1.0 / freq if freq > 0.0 else 0.0

        # Estat: instant (header.stamp) de l'ultima publicacio
        # Guardamos el stamp (no el reloj del PC) de la ultima publicacion para saber
        # cuando toca la siguiente. None = todavia no hemos publicado nada.
        self.stamp_ultim = None

        # Nom RELATIU: integracio.launch el posa dins de /ref o /lenta.
        # "encoders" SIN barra inicial: si el nodo vive en un namespace (/ref, /lenta)
        # el topic pasa a ser /ref/encoders, /lenta/encoders... y asi conviven varios.
        self.pub = rospy.Publisher("encoders", SensorState, queue_size=10)
        # Topic de Gazebo: ABSOLUT.
        # "/joint_states" CON barra: lo publica Gazebo en la raiz y no debe cambiar
        # aunque el nodo este dentro de un namespace.
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states,
                         queue_size=10)

    def cb_joint_states(self, msg):
        # Esta funcion se ejecuta CADA VEZ que llega un JointState (~100 Hz).
        # Usamos el tiempo del propio mensaje (stamp), que es el de la simulacion.
        stamp = msg.header.stamp

        # 1) Toca publicar? Submostreig segons ~freq_publicacio.
        # Submuestreo: Gazebo manda a ~100 Hz y nosotros queremos publicar a freq.
        # Solo publicamos si ha pasado al menos un periodo desde la ultima vez.
        if self.stamp_ultim is not None:
            dt = (stamp - self.stamp_ultim).to_sec()
            # dt < 0 vol dir que el temps ha anat enrere (reinici de la simulacio):
            # en aquest cas publiquem i ens tornem a sincronitzar.
            # Si dt < 0 el tiempo ha ido atras (se reinicio Gazebo): no hacemos "return",
            # publicamos y nos resincronizamos. Si 0 <= dt < periodo aun no toca y salimos.
            if 0.0 <= dt < self.periode - TOLERANCIA:
                return

        # 2) Busquem cada roda PEL NOM (l'ordre de la llista no es fiable).
        # No podemos fiarnos de que la rueda izquierda sea position[0]: buscamos el
        # indice de cada rueda por su nombre dentro de msg.name y con ese indice
        # sacamos su angulo acumulado en msg.position (en radianes).
        try:
            phi_d = msg.position[msg.name.index(self.roda_dreta)]
            phi_e = msg.position[msg.name.index(self.roda_esquerra)]
        except ValueError:
            # Si el nombre no esta en la lista, .index() lanza ValueError: avisamos
            # (como mucho cada 5 s para no inundar el terminal) y no publicamos.
            rospy.logwarn_throttle(5.0, "encoder_emulat: no trobo les rodes '%s' / '%s' "
                                   "a /joint_states", self.roda_dreta, self.roda_esquerra)
            return

        # 3) Angle -> comptes i publicar.
        # Montamos el mensaje de salida igual que lo publicaria el Kobuki real.
        sortida = SensorState()
        sortida.header.stamp = stamp            # el del /joint_states original
        # (copiamos el stamp original: odometria_propia calcula dt a partir de el)
        sortida.left_encoder = self.angle_a_comptes(phi_e)
        sortida.right_encoder = self.angle_a_comptes(phi_d)
        self.pub.publish(sortida)
        # Recordamos cuando publicamos, para el submuestreo de la proxima vez.
        self.stamp_ultim = stamp

    def angle_a_comptes(self, phi):
        """n = round(phi * N / 2pi) + comptador_inicial, amb desbordament de 16 bits."""
        # phi * N / (2*pi) = vueltas de rueda * cuentas por vuelta = cuentas.
        # round() porque un encoder real solo da numeros enteros.
        n = int(round(phi * self.N / (2.0 * math.pi))) + self.comptador_inicial
        # El % MODUL emula el contador de 16 bits: 65536 -> 0, 65537 -> 1...
        # Con angulos negativos tambien funciona: en Python -38 % 65536 = 65498.
        return n % MODUL        # a Python, % amb negatius dona sempre 0..65535


def main():
    # Registra el nodo en ROS con el nombre del enunciado.
    rospy.init_node("encoder_emulat")
    # Crear el objeto ya conecta el publisher y el subscriber (esta en __init__).
    EncoderEmulat()
    # Bloquea el programa escuchando mensajes hasta Ctrl+C.
    rospy.spin()


if __name__ == "__main__":
    # Ctrl+C lanza ROSInterruptException: lo capturamos para salir sin traza de error.
    try:
        main()
    except rospy.ROSInterruptException:
        pass