#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from abb_functions import *
from roslib import packages
import rbdl
import numpy as np

# Inicializar el nodo de ROS
rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

# Archivos donde se almacenará los datos
fqact = open("/tmp/qactual.txt", "w")
fqdes = open("/tmp/qdeseado.txt", "w")
fxact = open("/tmp/xactual.txt", "w")
fxdes = open("/tmp/xdeseado.txt", "w")

# Nombres de las articulaciones
jnames = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_4n", "joint_5", "joint_6")

# Objeto (mensaje) de tipo JointState
jstate = JointState()
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# Configuración articular inicial (en radianes)
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6, 0.1])
dq = np.array([0., 0., 0., 0., 0., 0., 0.])
ddq = np.array([0., 0., 0., 0., 0., 0., 0.])

# Configuración articular deseada
qdes = np.array([0, -0.3, -0.5, 1.2, 0.16, 1.8, 4.2])
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Posición resultante de la configuración articular deseada
xdes = fkine_irb(qdes)[0:3, 3]
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('/home/user/proyecto_ws/src/dependencia/urdf/gazebo.urdf')
ndof = modelo.q_size  # Grados de libertad

# Arrays numpy
zeros = np.zeros(ndof)  # Vector de ceros
g = np.zeros(ndof)  # Para la gravedad
M = np.zeros([ndof, ndof])
b = np.zeros(ndof)

# Frecuencia del envío (en Hz)
freq = 20
dt = 1.0 / freq
rate = rospy.Rate(freq)

# Definir robot
robot = Robot(q, dq, ndof, dt)

# Ganancias del controlador
valores = 1.2 * np.array([0.6, 0.7, 0.8, 0.9, 1.,1.2,1.3])
Kp = np.diag(valores)
Kd = 2 * np.sqrt(Kp)

# Bucle de ejecución continua
t = 0.0
cnt = 1
while not rospy.is_shutdown():
    # Leer valores del simulador 
    q = robot.read_joint_positions()
    dq = robot.read_joint_velocities()

    # Posición actual del efector final
    x = fkine_irb(q)[0:3, 3]
    jstate.header.stamp = rospy.Time.now()
    
    # Almacenamiento de datos
    fxact.write(f'{t} {x[0]} {x[1]} {x[2]}\n')
    fxdes.write(f'{t} {xdes[0]} {xdes[1]} {xdes[2]}\n')
    fqact.write(f'{t} {q[0]} {q[1]} {q[2]} {q[3]} {q[4]} {q[5]} {q[6]}\n')
    fqdes.write(f'{t} {qdes[0]} {qdes[1]} {qdes[2]} {qdes[3]} {qdes[4]} {qdes[5]} {qdes[6]}\n')
    
    # Control dinámico
    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
    rbdl.NonlinearEffects(modelo, q, dq, b)
    u_d = ddqdes + Kd.dot(dqdes - dq) + Kp.dot(qdes - q)  # Ley de control + compensador de gravedad
    u = M.dot(u_d) + b
    
    ddqc = np.linalg.inv(M).dot(u - b)
    qc = q + dt * dq
    dqc = dq + dt * ddqc
    
    # Simulación del robot
    robot.send_command(u)
    
    # Publicación del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    
    # Condición de parada
    if np.linalg.norm(xdes - x) < 1e-3:
        print(f"Se llegó al punto deseado en {cnt * dt:.3} segundos")
        break

    cnt += 1
    t += dt
    rate.sleep()

# Cerrar archivos
fqact.close()
fqdes.close()
fxact.close()
fxdes.close()