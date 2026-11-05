#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

FREQ_CONTROL = 50.0   # Hz a los que se ejecuta el bucle de control
PAUSA_ENTRE_SEGMENTS = 0.5  # entre segments el robot s'atura i espera mig segon
TIMEOUT_ODOM = 2.0    # s máximos esperando una odometría nueva tras un reset

"""hacer que el robot móvil (el Kobuki) 
recorra figuras geométricas (como un cuadrado, una línea recta o un arco) 
utilizando exclusivamente su propia odometría (/odom_propi)"""
class ExecutorRecorregut:
    def __init__(self):
        # parametros 
        self.tipus = rospy.get_param("~tipus", "quadrat")          # quadrat | recta | arc
        self.sentit = rospy.get_param("~sentit", "ccw")            # cw | ccw
        self.costat = rospy.get_param("~costat", 3.0)             
        self.longitud_recta = rospy.get_param("~longitud_recta", 5.0)  
        self.radi_arc = rospy.get_param("~radi_arc", 0.5)         
        self.angle_arc = rospy.get_param("~angle_arc", 180.0)     
        self.repeticions = rospy.get_param("~repeticions", 1)
        self.v_max = rospy.get_param("~v_max", 0.2)               
        self.w_max = rospy.get_param("~w_max", 0.5)               
        self.kp_angular = rospy.get_param("~kp_angular", 1.5)     
        self.tol_dist = rospy.get_param("~tol_dist", 0.01)        
        self.tol_angle = rospy.get_param("~tol_angle", 0.002)     
        self.pausa = rospy.get_param("~pausa", 1.0)               
        self.model = rospy.get_param("~model", "mobile_base")
        self.pose_inicial = rospy.get_param("~pose_inicial", [0.0, 0.0, 0.0])  
        self.serveis_reset = rospy.get_param("~serveis_reset", ["/odometria_propia/reset"])

        # Signo del sentido de giro: ccw = +1 (izquierda), cw = -1 (derecha)
        self.signe = 1.0 if self.sentit == "ccw" else -1.0

        # Pose actual según nuestra odometría (se actualiza en cb_odom).
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.n_odom = 0            # contador de mensajes de odometría recibidos
        # en preparar_repeticio, cuando el robot se teletransporta y resetea la odometria, el codigo hace un bucle de espera hasta que n_odom aumenta
        # esto garantiza que el programa no intente avanzar al siguiente tramo con datos viejos, sino que espera activamente a recibir odometria nueva despues del reset
        self.emergencia = False    # True si el bumper ha saltado (necesario porque el shudown no es instantaneo)

        # · Subscriber odom_propi (Odometry) → cb_odom
        rospy.Subscriber("odom_propi", Odometry, self.cb_odom)
        # · Subscriber /mobile_base/events/bumper (BumperEvent) → cb_bumper
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.cb_bumper)

        # · Publisher /mobile_base/commands/velocity (Twist)
        self.pub_vel = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
        # · Publisher /recorregut_estat (String)
        self.pub_estat = rospy.Publisher("/recorregut_estat", String, queue_size=10)

        # · ServiceProxy /gazebo/set_model_state (SetModelState)
        self.srv_set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        # · ServiceProxy cada element de serveis_reset (Empty)
        self.srv_resets = [rospy.ServiceProxy(nom, Empty) for nom in self.serveis_reset]
        # · on_shutdown → aturar
        rospy.on_shutdown(self.aturar)

        # para dar tiempo a conectarse y que no se pierda nada 
        rospy.sleep(1.0)

    # --- CALLBACKS ---
    # Cada vez que lelga un mensaje nuevo desde el nodo ROS llama automáticamente a laguna de estas funciones

    def cb_odom(self, msg): 
        """Guarda la pose (x, y, theta) de nuestra odometría"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # ROS no guarda la orientacion en angulos o radianes, sino mediante un quaternió (x, y, z, w)
        # necesitamos pasar theta (giro sobre el plano horizontal) a angulos de euler 
        o = msg.pose.pose.orientation
        # roll, pitch, yaw
        _, _, self.theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.n_odom += 1

    def cb_bumper(self, msg): 
        """si el bumper se pulsa (state == 1), parar y terminar."""
        if msg.state == BumperEvent.PRESSED:  # PRESSED vale 1
            self.emergencia = True
            self.aturar()
            rospy.logerr("Bumper activat: s'atura el robot i finalitza l'executor")
            rospy.signal_shutdown("bumper") # el script debe terminar su ejecución por la razon "bumper", asi no se queda colgado o en segundo plano

    # --- HELPERS ---

    @staticmethod
    def normalitza(angle): 
        """Lleva un ángulo a [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def satura(valor, limit):
        """Limita `valor` al intervalo [-limit, limit]"""
        # esto es porque el robot tiene límites físicos 
        return max(-limit, min(limit, valor))

    def publicar_velocitat(self, v, w):
        """Publica una orden de velocidad: v (m/s) lineal, w (rad/s) angular"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_vel.publish(cmd)

    def aturar(self):
        """Velocidad cero. También se ejecuta con on_shutdown"""
        self.publicar_velocitat(0.0, 0.0)

    def parat_i_espera(self, segons):
        """Para el robot y espera `segons`"""
        self.aturar()
        rospy.sleep(segons)

    def ok(self):
        """True mientras podemos seguir (sin Ctrl+C ni bumper)"""
        return (not rospy.is_shutdown()) and (not self.emergencia)

    # --- CONSTRUCCIÓN DE LA SECUENCIA DE SEGMENTOS ---

    def construir_segments(self):
        """Devuelve una lista de tuplas (tipo, valor)"""
        segments = []
 
        if self.tipus == "quadrat":
            # 4 veces: avanzar `costat` y girar +90° (ccw) o -90° (cw)
            for _ in range(4):
                segments.append(("recta", self.costat))
                segments.append(("gir", self.signe * math.pi / 2.0))
 
        elif self.tipus == "recta":
            # Un único tramo recto
            segments.append(("recta", self.longitud_recta))
 
        elif self.tipus == "arc":
            # Un único arco, ángulo positivo si ccw, negativo si cw
            segments.append(("arc", self.signe * math.radians(self.angle_arc)))

        else:
            rospy.logerr("~tipus desconegut: '%s' (quadrat | recta | arc)", self.tipus)
        
        return segments

    # --- PREPARACIÓN DE REPETICIÓN ---

    def preparar_repeticio(self):
        """1) Recoloca el robot en Gazebo, 2) pone a cero las odometrías."""
        # ---- 1) set_model_state: posición, orientación y velocidades a cero ----
        x0, y0, yaw0 = self.pose_inicial
        estat = ModelState()
        estat.model_name = self.model
        estat.reference_frame = "world"
        estat.pose = Pose()
        estat.pose.position.x = x0
        estat.pose.position.y = y0
        estat.pose.position.z = 0.0
        # * para el desempaquetado. Quaternion espera recibir (x, y, z, w) no ([x, y, z, w]) 
        estat.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw0))
 
        rospy.wait_for_service("/gazebo/set_model_state") # pausa hasta que Gazebo está abierto y listo para recibir órdenes 
        try:
            self.srv_set_state(estat)
        except rospy.ServiceException as e:
            rospy.logerr("Error cridant /gazebo/set_model_state: %s", e)
 
        # que el robot se estabilice tras el teletransporte
        self.parat_i_espera(self.pausa)
 
        # ---- 2) Reset de las odometrías ----
        for nom, srv in zip(self.serveis_reset, self.srv_resets): # zip empareja nomb con srv
            rospy.wait_for_service(nom)
            try:
                srv() # ejecuta la función que resetea la odometría
            except rospy.ServiceException as e:
                rospy.logerr("Error cridant %s: %s", nom, e)
 
        # Esperamos a recibir una odometría nueva (ya a cero)
        # Sin esto, self.x/y/theta podrían tener aún el valor de antes del reset
        # y el primer tramo calcularía mal su punto de partida
        n_abans = self.n_odom
        t0 = rospy.get_time() # para controlar el tiempo máximo de espera
        r = rospy.Rate(FREQ_CONTROL)
        while self.n_odom == n_abans and self.ok() and rospy.get_time() - t0 < TIMEOUT_ODOM:
            r.sleep()

    # --- SEGMENTOS ---

    def tram_recte(self, d):
        """Avanza `d` metros a v_max corrigiendo el rumbo
 
        - Guardamos la posición y el rumbo al empezar el tramo
        - Distancia recorrida = distancia euclídea desde el punto de inicio
        - Termina cuando recorrido >= d - tol_dist
        - Corrección: w = kp * e, con e = rumbo_inicial - rumbo_actual
          (normalizado). Así, si el robot se desvía, gira para volver al rumbo"""
        x0, y0 = self.x, self.y
        rumb_ref = self.theta
        r = rospy.Rate(FREQ_CONTROL)
 
        while self.ok():
            recorregut = math.hypot(self.x - x0, self.y - y0) # distancia en linea recta entre el inicio y la actual (pitagoras)
            if recorregut >= d - self.tol_dist:
                break  # hemos llegado
 
            e = self.normalitza(rumb_ref - self.theta)
            w = self.satura(self.kp_angular * e, self.w_max) # ai el robto se tuerce se genera una velocidad angular para corregirlo
            self.publicar_velocitat(self.v_max, w)
            r.sleep()
 
        self.parat_i_espera(PAUSA_ENTRE_SEGMENTS)

    def gir(self, alfa):
        """Gira `alfa` radianes en el sitio (v = 0).
 
        - Rumbo objetivo = rumbo al empezar + alfa.
        - e = normaliza(objetivo - rumbo_actual); w = kp * e saturada a ±w_max.
        - Termina cuando |e| < tol_angle"""
        rumb_obj = self.theta + alfa
        r = rospy.Rate(FREQ_CONTROL)
 
        while self.ok():
            e = self.normalitza(rumb_obj - self.theta)
            if abs(e) < self.tol_angle:
                break
 
            w = self.satura(self.kp_angular * e, self.w_max)
            self.publicar_velocitat(0.0, w)
            r.sleep()
 
        self.parat_i_espera(PAUSA_ENTRE_SEGMENTS)

    def arc(self, angle):
        """Arco de radio ~radi_arc y ángulo `angle` (rad, con signo)
 
        - v = v_max y w = v_max / radi_arc (saturada a w_max), constantes
        - Lazo abierto: no hay control de rumbo
        - Condición de fin: rumbo acumulat >= |angle|. Se acumulan
          incrementos pequeños (normalizados) en vez de comparar con un
          rumbo final, porque con 180° o más esa comparación es ambigua
          (+180° y -180° son el mismo ángulo)"""
        w_abs = self.satura(self.v_max / self.radi_arc, self.w_max) # w = v / R
        w = math.copysign(w_abs, angle)   # signo según ccw (+) / cw (-)
 
        acumulat = 0.0
        theta_ant = self.theta
        r = rospy.Rate(FREQ_CONTROL)
 
        while self.ok() and abs(acumulat) < abs(angle): # compara el angulo total girado hasta ahora con el angulo total
            self.publicar_velocitat(self.v_max, w) # publicas para que se te devuelva una nueva odometría
            r.sleep()
            acumulat += self.normalitza(self.theta - theta_ant)
            theta_ant = self.theta
 
        self.parat_i_espera(PAUSA_ENTRE_SEGMENTS)

    # --- BUCLE DE REPETICIONES ---

    def executar(self):
        # Esperar a tener al menos una odometría antes de empezar
        r = rospy.Rate(FREQ_CONTROL)
        while self.n_odom == 0 and not rospy.is_shutdown():
            rospy.loginfo_throttle(2.0, "Esperant odom_propi...") # maximo imprime una vez cada dos segundos
            r.sleep()
 
        segments = self.construir_segments()
        if not segments:
            rospy.signal_shutdown("cap segment per executar")
            return
 
        for n in range(1, int(self.repeticions) + 1):
            if not self.ok():
                break
 
            # a) recolocar robot + reset de odometrías
            self.preparar_repeticio()
            if not self.ok():
                break
 
            # b) avisar al registro de que empieza el recorrido n
            self.pub_estat.publish(String(data="inici %d" % n))
            rospy.loginfo("Recorregut %d/%d: inici", n, self.repeticions)
 
            # c) ejecutar los segmentos en orden
            for tipus, valor in segments:
                if not self.ok():
                    break
                if tipus == "recta":
                    self.tram_recte(valor)
                elif tipus == "gir":
                    self.gir(valor)
                elif tipus == "arc":
                    self.arc(valor)
 
            # d) parar, estabilizar y avisar del fin (robot ya detenido)
            if not self.ok():
                break
            self.parat_i_espera(self.pausa)
            self.pub_estat.publish(String(data="fi %d" % n))
            rospy.loginfo("Recorregut %d/%d: fi", n, self.repeticions)
 
        # Todas las repeticiones hechas: apagamos el nodo (si saltó el bumper, ya se había llamado a signal_shutdown)
        if not rospy.is_shutdown():
            rospy.signal_shutdown("recorreguts acabats")

if __name__ == '__main__':
    try:
        rospy.init_node("executor_recorregut") # registrar nuevo nodo
        node = ExecutorRecorregut()
        node.executar()
    except rospy.ROSInterruptException:
        pass