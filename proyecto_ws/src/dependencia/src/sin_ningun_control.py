#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from abb_functions import *
from roslib import packages
import rbdl
if __name__ == '__main__':
  rospy.init_node("control_pdg")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
  bmarker_actual  = BallMarker(color['RED'])
  bmarker_deseado = BallMarker(color['GREEN'])
  # Archivos donde se almacenara los datos
  fqact = open("qactual.csv", "w")
  fqdes = open("qdeseado.csv", "w")
  fxact = open("xactual.csv", "w")
  fxdes = open("xdeseado.csv", "w")
  
  # Nombres de las articulaciones
  jnames = ("joint_1", "joint_2", "joint_3","joint_4", "joint_4n", "joint_5", "joint_6")
  # Objeto (mensaje) de tipo JointState
  jstate = JointState()
  # Valores del mensaje
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
  


  # =============================================================
  # Configuracion articular inicial (en radianes)
  q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6, 0.1])
  # Velocidad inicial
  dq = np.array([0., 0., 0., 0., 0., 0., 0.])
  # Configuracion articular deseada
  qdes = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0.1])
  # =============================================================



  # Posicion resultante de la configuracion articular deseada
  xdes = fkine_irb(qdes)[0:3,3]
  # Copiar la configuracion articular en el mensaje a ser publicado
  jstate.position = q
  pub.publish(jstate)
  
  # Modelo RBDL
  modelo = rbdl.loadModel('/home/user/proyecto_ws/src/dependencia/urdf/gazebo.urdf')
  ndof   = modelo.q_size     # Grados de libertad
  M     = np.zeros([ndof, ndof])
  b = np.zeros(ndof)
  
  # Frecuencia del envio (en Hz)
  freq = 20
  dt = 1.0/freq
  rate = rospy.Rate(freq)
  
  # Simulador dinamico del robot
  robot = Robot(q, dq, ndof, dt)
  # Se definen las ganancias del controlador
  # Hay que cambiar estos valores a la vez 3 veces
  kp_chiquito = 4
  kd_chiquito = 10

  Kp = np.diag([kp_chiquito, kp_chiquito, kp_chiquito, kp_chiquito, kp_chiquito, kp_chiquito, kp_chiquito])
  Kd = np.diag([kd_chiquito, kd_chiquito, kd_chiquito, kd_chiquito, kd_chiquito, kd_chiquito, kd_chiquito])
  
  # Bucle de ejecucion continua
  t = 0.0
  while not rospy.is_shutdown():
  
    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine_irb(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()


    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')


    # ----------------------------
    # Control dinamico
    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
    rbdl.NonlinearEffects(modelo, q, dq, b)
   
    # Ley de control + compensación de gravedad
    u_d = Kp.dot(qdes - q) + Kd.dot(-dq)
    u = 0                                       #la ley de control es 0, haci evitamos q controle
    # Integración de las ecuaciones de movimiento
    ddqc = np.linalg.inv(M).dot(u - b)
    qc = q + dt * dq
    dqc = dq + dt * ddqc

    
    
    # Simulacion del robot
    robot.send_command(u)
    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()
    if np.linalg.norm(q-dq) < 0.001:
      break

  fqact.close()
  fqdes.close()
  fxact.close()
  fxdes.close()