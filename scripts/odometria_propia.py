#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apartat 1.2 - Odometria diferencial a partir dels comptes dels encoders.

Cada missatge d'encoders fa UN pas complet:
  comptes -> increment de comptes (amb desbordament) -> distancia de cada roda
  -> (ds, dtheta) -> integracio (euler | punt_mig | exacta) -> publicar.
Dins del codi la pose es sempre (x, y, theta) amb theta en radians; el
quaternio nomes apareix a la frontera amb ROS (Odometry i tf).
"""
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import quaternion_from_euler

MODUL = 65536
MIG_RANG = 32768       # 2^15


class OdometriaPropia(object):

    def __init__(self):
        # --- Parametres privats (valors per defecte de l'enunciat) ---
        self.r_d = float(rospy.get_param("~radi_dreta", 0.035))
        self.r_e = float(rospy.get_param("~radi_esquerra", 0.035))
        self.b = float(rospy.get_param("~distancia_rodes", 0.230))
        self.N = float(rospy.get_param("~comptes_per_volta", 2578.33))
        self.metode = rospy.get_param("~metode", "punt_mig")
        self.llindar = float(rospy.get_param("~llindar_dtheta", 1.0e-6))
        self.frame_odom = rospy.get_param("~frame_odom", "odom_propi")
        self.frame_base = rospy.get_param("~frame_base", "base_footprint_propi")
        self.publicar_tf = bool(rospy.get_param("~publicar_tf", True))

        integradors = {"euler": self.euler, "punt_mig": self.punt_mig,
                       "exacta": self.exacta}
        if self.metode not in integradors:
            rospy.logfatal("~metode ha de ser euler | punt_mig | exacta (rebut '%s')",
                           self.metode)
            raise rospy.ROSInitException("metode invalid")
        self.integrar = integradors[self.metode]

        # --- Estat ---
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.n_ant = None          # (comptes_dreta, comptes_esquerra) anteriors
        self.stamp_ant = None      # header.stamp anterior
        # El callback i el servei corren en fils diferents: protegim l'estat.
        self.lock = threading.Lock()

        # --- Interficie ROS (noms relatius, excepte el que diu l'enunciat) ---
        self.pub = rospy.Publisher("odom_propi", Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Service("~reset", Empty, self.servei_reset)
        rospy.Subscriber("encoders", SensorState, self.cb_encoders, queue_size=10)

    # ------------------------------------------------------------------
    # Un pas complet d'odometria
    # ------------------------------------------------------------------
    def cb_encoders(self, msg):
        with self.lock:
            stamp = msg.header.stamp
            n = (msg.right_encoder, msg.left_encoder)

            # Primera lectura: nomes desem, no integrem.
            if self.n_ant is None:
                self.n_ant, self.stamp_ant = n, stamp
                return

            # dt SEMPRE dels stamps (mai rospy.Time.now()): funciona amb temps de simulacio.
            dt = (stamp - self.stamp_ant).to_sec()
            if dt <= 0.0:
                rospy.logwarn("odometria_propia: dt = %.6f <= 0 (el temps ha anat "
                              "enrere?). Reinicio la lectura i conservo la pose.", dt)
                self.n_ant = None
                self.stamp_ant = None
                return

            dn_d = self.increment_comptes(n[0], self.n_ant[0])
            dn_e = self.increment_comptes(n[1], self.n_ant[1])
            ds_d, ds_e = self.distancies_rodes(dn_d, dn_e)
            ds, dtheta = self.increment_robot(ds_d, ds_e)

            self.x, self.y, self.theta = self.integrar((self.x, self.y, self.theta),
                                                       ds, dtheta)
            self.n_ant, self.stamp_ant = n, stamp

            self.publicar_odometria(stamp, ds / dt, dtheta / dt)
            if self.publicar_tf:
                self.publicar_transformacio(stamp)

    # ------------------------------------------------------------------
    # Pas 1-3: comptes -> (ds, dtheta)
    # ------------------------------------------------------------------
    @staticmethod
    def increment_comptes(n, n_ant):
        """Delta n amb correccio del desbordament de 16 bits (diap. 15)."""
        dn = int(n) - int(n_ant)
        if dn < -MIG_RANG:
            dn += MODUL
        elif dn > MIG_RANG - 1:
            dn -= MODUL
        return dn

    def distancies_rodes(self, dn_d, dn_e):
        """Delta n -> distancia rodada per cada roda, cadascuna amb el seu radi (diap. 14)."""
        ds_d = self.r_d * (2.0 * math.pi * dn_d / self.N)
        ds_e = self.r_e * (2.0 * math.pi * dn_e / self.N)
        return ds_d, ds_e

    def increment_robot(self, ds_d, ds_e):
        """(ds, dtheta) del centre del robot (diap. 19)."""
        return (ds_d + ds_e) / 2.0, (ds_d - ds_e) / self.b

    # ------------------------------------------------------------------
    # Pas 4: integracio. Reben (x, y, theta), ds, dtheta; retornen la pose nova.
    # ------------------------------------------------------------------
    def euler(self, pose, ds, dtheta):
        """Avanca ds amb l'orientacio d'INICI i despres gira (diap. 22)."""
        x, y, th = pose
        return (x + ds * math.cos(th),
                y + ds * math.sin(th),
                self.normalitza(th + dtheta))

    def punt_mig(self, pose, ds, dtheta):
        """Avanca ds amb l'orientacio MITJANA th + dtheta/2 (diap. 24)."""
        x, y, th = pose
        fi = th + dtheta / 2.0
        return (x + ds * math.cos(fi),
                y + ds * math.sin(fi),
                self.normalitza(th + dtheta))

    def exacta(self, pose, ds, dtheta):
        """Avanca la CORDA de l'arc amb orientacio mitjana (diap. 26)."""
        x, y, th = pose
        if abs(dtheta) < self.llindar:
            c = ds                                  # practicament recte: = punt mig
        else:
            rho = ds / dtheta                       # radi de l'arc
            c = 2.0 * rho * math.sin(dtheta / 2.0)  # corda
        fi = th + dtheta / 2.0
        return (x + c * math.cos(fi),
                y + c * math.sin(fi),
                self.normalitza(th + dtheta))

    @staticmethod
    def normalitza(angle):
        """Angle a [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------------
    # Publicacio (aqui, i nomes aqui, apareix el quaternio)
    # ------------------------------------------------------------------
    def publicar_odometria(self, stamp, v, w):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.theta))
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Covariancia 6x6 (x, y, z, roll, pitch, yaw), diagonal constant.
        cov = [0.0] * 36
        cov[0] = 1.0e-3      # xx
        cov[7] = 1.0e-3      # yy
        cov[14] = 1.0e6      # zz   (no s'estima: robot pla)
        cov[21] = 1.0e6      # roll
        cov[28] = 1.0e6      # pitch
        cov[35] = 1.0e-2     # yaw
        odom.pose.covariance = cov
        self.pub.publish(odom)

    def publicar_transformacio(self, stamp):
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
        with self.lock:
            self.x, self.y, self.theta = 0.0, 0.0, 0.0
        rospy.loginfo("odometria_propia: pose a zero")
        return EmptyResponse()


def main():
    rospy.init_node("odometria_propia")
    OdometriaPropia()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
