#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 1.1 - Emulador d'encoders.

Converteix l'angle acumulat de cada roda (/joint_states, radians, continu) en
comptes enters de 16 bits (0..65535, amb desbordament) i els publica amb el mateix
missatge que el Kobuki real (kobuki_msgs/SensorSta te). Aixi odometria_propia.py
treballa com amb encoders reals.
"""
import math

import rospy
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import JointState

MODUL = 65536          # 2^16: el comptador torna a 0 despres del 65535
TOLERANCIA = 1e-6      # s, per evitar problemes d'arrodoniment en comparar temps


class EncoderEmulat(object):

    def __init__(self):
        # --- Parametres privats (valors per defecte de l'enunciat) ---
        self.roda_dreta = rospy.get_param("~roda_dreta", "wheel_right_joint")
        self.roda_esquerra = rospy.get_param("~roda_esquerra", "wheel_left_joint")
        self.N = float(rospy.get_param("~comptes_per_volta", 2578.33))
        self.comptador_inicial = int(rospy.get_param("~comptador_inicial", 0))
        freq = float(rospy.get_param("~freq_publicacio", 50.0))
        self.periode = 1.0 / freq if freq > 0.0 else 0.0

        # Estat: instant (header.stamp) de l'ultima publicacio
        self.stamp_ultim = None

        # Nom RELATIU: integracio.launch el posa dins de /ref o /lenta.
        self.pub = rospy.Publisher("encoders", SensorState, queue_size=10)
        # Topic de Gazebo: ABSOLUT.
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_states,
                         queue_size=10)

    def cb_joint_states(self, msg):
        stamp = msg.header.stamp

        # 1) Toca publicar? Submostreig segons ~freq_publicacio.
        if self.stamp_ultim is not None:
            dt = (stamp - self.stamp_ultim).to_sec()
            # dt < 0 vol dir que el temps ha anat enrere (reinici de la simulacio):
            # en aquest cas publiquem i ens tornem a sincronitzar.
            if 0.0 <= dt < self.periode - TOLERANCIA:
                return

        # 2) Busquem cada roda PEL NOM (l'ordre de la llista no es fiable).
        try:
            phi_d = msg.position[msg.name.index(self.roda_dreta)]
            phi_e = msg.position[msg.name.index(self.roda_esquerra)]
        except ValueError:
            rospy.logwarn_throttle(5.0, "encoder_emulat: no trobo les rodes '%s' / '%s' "
                                   "a /joint_states", self.roda_dreta, self.roda_esquerra)
            return

        # 3) Angle -> comptes i publicar.
        sortida = SensorState()
        sortida.header.stamp = stamp            # el del /joint_states original
        sortida.left_encoder = self.angle_a_comptes(phi_e)
        sortida.right_encoder = self.angle_a_comptes(phi_d)
        self.pub.publish(sortida)
        self.stamp_ultim = stamp

    def angle_a_comptes(self, phi):
        """n = round(phi * N / 2pi) + comptador_inicial, amb desbordament de 16 bits."""
        n = int(round(phi * self.N / (2.0 * math.pi))) + self.comptador_inicial
        return n % MODUL        # a Python, % amb negatius dona sempre 0..65535


def main():
    rospy.init_node("encoder_emulat")
    EncoderEmulat()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
