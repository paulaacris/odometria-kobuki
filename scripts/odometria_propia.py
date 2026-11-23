#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 1.2 - Odometria diferencial a partir dels comptes dels encoders.

Cada missatge d'encoders fa UN pas complet:
  comptes -> increment de comptes (amb desbordament) -> distancia de cada roda
  -> (ds, dtheta) -> integracio (euler | punt_mig | exacta) -> publicar.
Dins del codi la pose es sempre (x, y, theta) amb theta en radians; el
quaternio nomes apareix a la frontera amb ROS (Odometry i tf).
"""
# math: sin, cos, atan2, pi...
import math
# threading: para el Lock que protege la pose (callback y servicio van en hilos distintos).
import threading

import rospy
# tf2_ros: para publicar la transformada tf odom_propi -> base_footprint_propi.
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
# SensorState: el mensaje de entrada (cuentas de los encoders).
from kobuki_msgs.msg import SensorState
# Odometry: el mensaje de salida (pose, velocidad y covarianza).
from nav_msgs.msg import Odometry
# Empty/EmptyResponse: servicio ~reset, que no lleva datos ni de ida ni de vuelta.
from std_srvs.srv import Empty, EmptyResponse
# Unica funcion que usamos para pasar de angulo theta a cuaternion (solo al publicar).
from tf.transformations import quaternion_from_euler

# Tamano del contador de 16 bits (2^16) y su mitad (2^15). Se usan en la regla del
# desbordamiento: un salto de mas de media vuelta del contador es en realidad un wrap.
MODUL = 65536
MIG_RANG = 32768       # 2^15


class OdometriaPropia(object):

    def __init__(self):
        # --- Parametres privats (valors per defecte de l'enunciat) ---
        # Radios de cada rueda por separado (en el apartado 4 seran distintos).
        self.r_d = float(rospy.get_param("~radi_dreta", 0.035))
        self.r_e = float(rospy.get_param("~radi_esquerra", 0.035))
        # b = distancia entre las dos ruedas (m), necesaria para el giro dtheta.
        self.b = float(rospy.get_param("~distancia_rodes", 0.230))
        # N = cuentas por vuelta de rueda (2578.33, mismo valor que el emulador).
        self.N = float(rospy.get_param("~comptes_per_volta", 2578.33))
        # Metodo de integracion: euler | punt_mig | exacta. Por defecto punto medio.
        self.metode = rospy.get_param("~metode", "punt_mig")
        # Si |dtheta| es menor que este umbral, el robot va "recto": en la formula
        # exacta evita el 0/0 de rho = ds/dtheta.
        self.llindar = float(rospy.get_param("~llindar_dtheta", 1.0e-6))
        # Nombres de los marcos de referencia. Con sufijo _propi para no pisar los
        # odom / base_footprint del simulador.
        self.frame_odom = rospy.get_param("~frame_odom", "odom_propi")
        self.frame_base = rospy.get_param("~frame_base", "base_footprint_propi")
        # En integracio.launch solo la odometria de /ref publica tf; las otras no, para
        # no tener varias transformadas distintas con el mismo nombre.
        self.publicar_tf = bool(rospy.get_param("~publicar_tf", True))

        # Diccionario nombre -> funcion. Asi elegir el metodo es solo una busqueda,
        # y el resto del codigo no se entera de cual se usa.
        integradors = {"euler": self.euler, "punt_mig": self.punt_mig,
                       "exacta": self.exacta}
        # Si el nombre no es valido, paramos el nodo al arrancar con un error claro,
        # en vez de fallar mas tarde con un mensaje confuso.
        if self.metode not in integradors:
            rospy.logfatal("~metode ha de ser euler | punt_mig | exacta (rebut '%s')",
                           self.metode)
            raise rospy.ROSInitException("metode invalid")
        self.integrar = integradors[self.metode]

        # --- Estat ---
        # Pose actual del robot (x, y en m; theta en rad), empezando en el origen.
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.n_ant = None          # (comptes_dreta, comptes_esquerra) anteriors
        self.stamp_ant = None      # header.stamp anterior
        # None significa "aun no hemos recibido ninguna lectura": la primera no se integra.
        # El callback i el servei corren en fils diferents: protegim l'estat.
        # Candado para que callback y servicio ~reset no toquen la pose a la vez.
        self.lock = threading.Lock()

        # --- Interficie ROS (noms relatius, excepte el que diu l'enunciat) ---
        # "odom_propi" y "encoders" son RELATIVOS (sin barra): cambian segun el namespace.
        self.pub = rospy.Publisher("odom_propi", Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        # "~reset" es privado: queda como /<nodo>/reset, p. ej. /odometria_propia/reset.
        rospy.Service("~reset", Empty, self.servei_reset)
        rospy.Subscriber("encoders", SensorState, self.cb_encoders, queue_size=10)

    # ------------------------------------------------------------------
    # Un pas complet d'odometria
    # ------------------------------------------------------------------
    def cb_encoders(self, msg):
        # Se ejecuta cada vez que llegan cuentas. Todo el calculo va dentro del lock
        # para que ~reset no cambie la pose a mitad de un paso.
        with self.lock:
            stamp = msg.header.stamp
            # Guardamos el par de cuentas como (derecha, izquierda).
            n = (msg.right_encoder, msg.left_encoder)

            # Primera lectura: nomes desem, no integrem.
            # Con una sola muestra no hay incremento posible: guardamos y salimos.
            if self.n_ant is None:
                self.n_ant, self.stamp_ant = n, stamp
                return

            # dt SEMPRE dels stamps (mai rospy.Time.now()): funciona amb temps de simulacio.
            # dt sale de los stamps del mensaje (instante de la medida), no del reloj del
            # PC: asi funciona aunque Gazebo vaya mas lento o mas rapido que el tiempo real.
            dt = (stamp - self.stamp_ant).to_sec()
            # Si dt <= 0 el tiempo no avanza o va atras (reinicio de la simulacion):
            # avisamos, "olvidamos" la lectura anterior y CONSERVAMOS la pose.
            if dt <= 0.0:
                rospy.logwarn("odometria_propia: dt = %.6f <= 0 (el temps ha anat "
                              "enrere?). Reinicio la lectura i conservo la pose.", dt)
                self.n_ant = None
                self.stamp_ant = None
                return

            # Los 6 pasos de la cadena, en orden:
            # (A/B) incremento de cuentas de cada rueda, con la regla del desbordamiento
            dn_d = self.increment_comptes(n[0], self.n_ant[0])
            dn_e = self.increment_comptes(n[1], self.n_ant[1])
            # (C) cuentas -> distancia recorrida por cada rueda
            ds_d, ds_e = self.distancies_rodes(dn_d, dn_e)
            # (D) distancias de las ruedas -> avance y giro del robot
            ds, dtheta = self.increment_robot(ds_d, ds_e)

            # (E) integrar: pose anterior + incremento = pose nueva (metodo elegido)
            self.x, self.y, self.theta = self.integrar((self.x, self.y, self.theta),
                                                       ds, dtheta)
            # Preparamos la siguiente vuelta: estas cuentas y este stamp seran los "anteriores".
            self.n_ant, self.stamp_ant = n, stamp

            # (F) publicar. Velocidades: v = ds/dt y w = dtheta/dt.
            self.publicar_odometria(stamp, ds / dt, dtheta / dt)
            if self.publicar_tf:
                self.publicar_transformacio(stamp)

    # ------------------------------------------------------------------
    # Pas 1-3: comptes -> (ds, dtheta)
    # ------------------------------------------------------------------
    @staticmethod
    def increment_comptes(n, n_ant):
        """Delta n amb correccio del desbordament de 16 bits (diap. 15)."""
        # Diferencia directa entre las cuentas actual y anterior (como enteros).
        dn = int(n) - int(n_ant)
        # Entre dos lecturas la rueda se mueve poco (< media vuelta del contador), asi
        # que el incremento real esta siempre entre -32768 y +32767.
        # Si sale mas negativo, el contador paso de 65535 a 0 hacia delante: sumamos 65536.
        if dn < -MIG_RANG:
            dn += MODUL
        # Si sale mas positivo, paso de 0 a 65535 hacia atras: restamos 65536.
        elif dn > MIG_RANG - 1:
            dn -= MODUL
        # Ejemplo: 65000 -> 300 da -64700, y -64700 + 65536 = +836.
        return dn

    def distancies_rodes(self, dn_d, dn_e):
        """Delta n -> distancia rodada per cada roda, cadascuna amb el seu radi (diap. 14)."""
        # cuentas -> angulo de rueda: dphi = 2*pi * dn / N ;  angulo -> distancia: ds = r * dphi.
        # Cada rueda con SU radio (r_d, r_e): en la calibracion pueden ser distintos.
        ds_d = self.r_d * (2.0 * math.pi * dn_d / self.N)
        ds_e = self.r_e * (2.0 * math.pi * dn_e / self.N)
        return ds_d, ds_e

    def increment_robot(self, ds_d, ds_e):
        """(ds, dtheta) del centre del robot (diap. 19)."""
        # Avance del centro = media de lo que recorren las ruedas.
        # Giro = diferencia entre ruedas / separacion b (positivo = antihorario).
        return (ds_d + ds_e) / 2.0, (ds_d - ds_e) / self.b

    # ------------------------------------------------------------------
    # Pas 4: integracio. Reben (x, y, theta), ds, dtheta; retornen la pose nova.
    # ------------------------------------------------------------------
    # Los tres metodos tienen la misma "firma" (pose, ds, dtheta -> pose nueva), por eso
    # se pueden intercambiar con el parametro ~metode. Todos normalizan theta a [-pi, pi].
    def euler(self, pose, ds, dtheta):
        """Avanca ds amb l'orientacio d'INICI i despres gira (diap. 22)."""
        x, y, th = pose
        # Euler: proyecta TODO el avance con la orientacion del principio del paso.
        return (x + ds * math.cos(th),
                y + ds * math.sin(th),
                self.normalitza(th + dtheta))

    def punt_mig(self, pose, ds, dtheta):
        """Avanca ds amb l'orientacio MITJANA th + dtheta/2 (diap. 24)."""
        x, y, th = pose
        # Punto medio: el robot gira mientras avanza, asi que usamos la orientacion
        # "a mitad del arco" (th + dtheta/2). Error de orden 2: mucho menor que Euler.
        fi = th + dtheta / 2.0
        return (x + ds * math.cos(fi),
                y + ds * math.sin(fi),
                self.normalitza(th + dtheta))

    def exacta(self, pose, ds, dtheta):
        """Avanca la CORDA de l'arc amb orientacio mitjana (diap. 26)."""
        x, y, th = pose
        # Exacta: el robot recorre un ARCO de longitud ds, pero el desplazamiento real
        # entre inicio y final es la CUERDA, que es un poco mas corta que el arco.
        if abs(dtheta) < self.llindar:
            c = ds                                  # practicament recte: = punt mig
            # (si casi no gira, la cuerda es el arco; y evitamos dividir ds entre ~0)
        else:
            rho = ds / dtheta                       # radi de l'arc
            c = 2.0 * rho * math.sin(dtheta / 2.0)  # corda
        # La cuerda se proyecta con la orientacion a mitad del arco.
        fi = th + dtheta / 2.0
        return (x + c * math.cos(fi),
                y + c * math.sin(fi),
                self.normalitza(th + dtheta))

    @staticmethod
    def normalitza(angle):
        """Angle a [-pi, pi]."""
        # atan2(sin, cos) devuelve siempre el mismo angulo envuelto en [-pi, pi].
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    # Publicacio (aqui, i nomes aqui, apareix el quaternio)
    # ------------------------------------------------------------------
    def publicar_odometria(self, stamp, v, w):
        odom = Odometry()
        # Mismo stamp que las cuentas: la odometria queda fechada en el instante de la medida.
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom       # marco en el que esta la pose
        odom.child_frame_id = self.frame_base        # marco que se mueve (el robot)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # ROS no guarda el yaw como angulo sino como cuaternion (x, y, z, w). Aqui es
        # donde se convierte: para un giro solo en yaw sale (0, 0, sin(th/2), cos(th/2)).
        odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.theta))
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Covariancia 6x6 (x, y, z, roll, pitch, yaw), diagonal constant.
        # Matriz 6x6 guardada en un vector de 36 numeros; la diagonal esta en 0, 7, 14, 21, 28, 35.
        cov = [0.0] * 36
        cov[0] = 1.0e-3      # xx
        cov[7] = 1.0e-3      # yy
        # z, roll y pitch: el robot es plano y no los estima, asi que ponemos una
        # incertidumbre enorme (1e6) para decir "no te fies de esto".
        cov[14] = 1.0e6      # zz   (no s'estima: robot pla)
        cov[21] = 1.0e6      # roll
        cov[28] = 1.0e6      # pitch
        cov[35] = 1.0e-2     # yaw
        odom.pose.covariance = cov
        self.pub.publish(odom)

    def publicar_transformacio(self, stamp):
        # Misma pose que en Odometry, pero como transformada tf: asi rviz puede dibujar
        # el marco base_footprint_propi moviendose respecto a odom_propi.
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, self.theta))
        self.br.sendTransform(t)

    # ------------------------------------------------------------------
    # Servei ~reset: pose a zero, pero CONSERVEM els darrers comptes llegits
    # (si no, el pas seguent veuria un salt enorme).
    # ------------------------------------------------------------------
    def servei_reset(self, _req):
        # El lock evita que el callback de encoders cambie la pose mientras la ponemos a 0.
        with self.lock:
            self.x, self.y, self.theta = 0.0, 0.0, 0.0
        rospy.loginfo("odometria_propia: pose a zero")
        # Un servicio siempre debe devolver una respuesta; la de Empty va vacia.
        return EmptyResponse()


def main():
    # Registra el nodo en ROS. Crear el objeto conecta publisher, subscriber y servicio.
    rospy.init_node("odometria_propia")
    OdometriaPropia()
    # Mantiene el nodo vivo escuchando mensajes hasta Ctrl+C.
    rospy.spin()


if __name__ == "__main__":
    # Ctrl+C lanza ROSInterruptException: la capturamos para salir limpios.
    try:
        main()
    except rospy.ROSInterruptException:
        pass