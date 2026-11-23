#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Registre en CSV de cada recorregut (apartat 2).   [P] el dona el professorat

Escriu dos tipus de fitxer dins de ~directori:
  finals.csv     una fila per font i repeticio, amb la pose final i la covariancia
  serie_<n>.csv  la serie temporal del recorregut n, mostrejada a ~freq_serie
"""

import csv
import os

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

CAPCALERA_FINALS = ["run", "font", "x", "y", "theta",
                    "cov_xx", "cov_xy", "cov_yy",
                    "x_real", "y_real", "theta_real"]
CAPCALERA_SERIE = ["t", "font", "x", "y", "theta",
                   "x_real", "y_real", "theta_real"]


class RegistreCSV(object):

    def __init__(self):
        self.topics_odom = rospy.get_param("~topics_odom", ["/odom_propi", "/odom"])
        self.model = rospy.get_param("~model", "mobile_base")
        self.directori = rospy.get_param("~directori", "resultats/prova")
        self.freq_serie = float(rospy.get_param("~freq_serie", 10.0))

        os.makedirs(self.directori, exist_ok=True)

        # Ultim valor de cada font: {nom_font: (x, y, theta, cov_xx, cov_xy, cov_yy)}
        self.ultim = {}
        self.pose_gazebo = None       # (x, y, theta) absoluta
        self.origen = None            # (x0, y0, theta0) de la repeticio en curs
        self.run_actual = None
        self.t_inici = None

        self.fitxer_serie = None
        self.writer_serie = None

        ruta_finals = os.path.join(self.directori, "finals.csv")
        self.fitxer_finals = open(ruta_finals, "w", newline="")
        self.writer_finals = csv.writer(self.fitxer_finals)
        self.writer_finals.writerow(CAPCALERA_FINALS)
        self.fitxer_finals.flush()

        for topic in self.topics_odom:
            rospy.Subscriber(topic, Odometry, self.cb_odom,
                             callback_args=topic, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.cb_gazebo, queue_size=1)
        rospy.Subscriber("/mobile_base/sensors/imu_data", Imu,
                         self.cb_imu, queue_size=10)
        rospy.Subscriber("/recorregut_estat", String, self.cb_estat, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.freq_serie), self.cb_timer)
        rospy.on_shutdown(self.tancar_fitxers)

        rospy.loginfo("registre_csv: %d fonts -> %s",
                      len(self.topics_odom) + 1, self.directori)

    # ------------------------------------------------------------------
    # Entrades
    # ------------------------------------------------------------------
    def cb_odom(self, msg, font):
        cov = msg.pose.covariance
        self.ultim[font] = (msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            self.yaw(msg.pose.pose.orientation),
                            cov[0], cov[1], cov[7])

    def cb_imu(self, msg):
        # De la IMU nomes en surt orientacio: x, y i covariancia queden buides.
        self.ultim["imu"] = (None, None, self.yaw(msg.orientation), None, None, None)

    def cb_gazebo(self, msg):
        try:
            idx = msg.name.index(self.model)
        except ValueError:
            rospy.logwarn_throttle(5.0, "El model '%s' no hi es a /gazebo/model_states",
                                   self.model)
            return
        pose = msg.pose[idx]
        self.pose_gazebo = (pose.position.x, pose.position.y, self.yaw(pose.orientation))

    def cb_estat(self, msg):
        parts = msg.data.split()
        if len(parts) != 2:
            return
        accio, n = parts[0], parts[1]

        if accio == "inici":
            if self.pose_gazebo is None:
                rospy.logwarn("Encara no tinc pose de Gazebo: no puc fixar l'origen")
                return
            self.run_actual = n
            self.origen = self.pose_gazebo
            self.t_inici = rospy.Time.now()
            self.obrir_serie(n)
        elif accio == "fi":
            self.escriure_finals(n)
            self.tancar_serie()
            self.run_actual = None

    def cb_timer(self, _event):
        if self.writer_serie is None or self.origen is None:
            return
        t = (rospy.Time.now() - self.t_inici).to_sec()
        x_real, y_real, theta_real = self.al_marc_sortida()
        for font, valor in sorted(self.ultim.items()):
            x, y, theta = valor[0], valor[1], valor[2]
            self.writer_serie.writerow([
                "%.3f" % t, font,
                self.num(x), self.num(y), self.num(theta),
                self.num(x_real), self.num(y_real), self.num(theta_real)])
        self.fitxer_serie.flush()

    # ------------------------------------------------------------------
    # Fitxers
    # ------------------------------------------------------------------
    def obrir_serie(self, n):
        self.tancar_serie()
        ruta = os.path.join(self.directori, "serie_%s.csv" % n)
        self.fitxer_serie = open(ruta, "w", newline="")
        self.writer_serie = csv.writer(self.fitxer_serie)
        self.writer_serie.writerow(CAPCALERA_SERIE)
        self.fitxer_serie.flush()

    def tancar_serie(self):
        if self.fitxer_serie is not None:
            self.fitxer_serie.close()
            self.fitxer_serie = None
            self.writer_serie = None

    def escriure_finals(self, n):
        x_real, y_real, theta_real = self.al_marc_sortida()
        for font, valor in sorted(self.ultim.items()):
            x, y, theta, cxx, cxy, cyy = valor
            self.writer_finals.writerow([
                n, font,
                self.num(x), self.num(y), self.num(theta),
                self.num(cxx), self.num(cxy), self.num(cyy),
                self.num(x_real), self.num(y_real), self.num(theta_real)])
        self.fitxer_finals.flush()

    def tancar_fitxers(self):
        self.tancar_serie()
        if self.fitxer_finals is not None:
            self.fitxer_finals.close()
            self.fitxer_finals = None

    # ------------------------------------------------------------------
    # Geometria
    # ------------------------------------------------------------------
    def al_marc_sortida(self):
        """Pose real de Gazebo expressada al marc de sortida de la repeticio."""
        if self.pose_gazebo is None or self.origen is None:
            return None, None, None
        x, y, theta = self.pose_gazebo
        x0, y0, theta0 = self.origen
        dx, dy = x - x0, y - y0
        cos0, sin0 = np.cos(theta0), np.sin(theta0)
        return (cos0 * dx + sin0 * dy,
                -sin0 * dx + cos0 * dy,
                self.normalitza(theta - theta0))

    @staticmethod
    def yaw(q):
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def normalitza(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    @staticmethod
    def num(v):
        """Metres i radians amb punt decimal; cadena buida si no n'hi ha valor."""
        return "" if v is None else "%.6f" % v


def main():
    rospy.init_node("registre_csv")
    RegistreCSV()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
