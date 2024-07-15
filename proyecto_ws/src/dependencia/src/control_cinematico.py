#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from markers import *

# Definir los parámetros DH
def dh(d, theta, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Cinemática directa del robot de 7 grados de libertad
def fkine_irb(q):
    T1 = dh(0.68, q[0], 0.2, -np.pi/2)
    T2 = dh(0, np.pi/2 + q[1], -0.89, 0)
    T3 = dh(0, q[2], -0.15, np.pi/2)
    T4 = dh(1.733, q[3], 0, 0)
    T5 = dh(q[4], 0, 0, -np.pi/2)
    T6 = dh(0, q[5], 0, np.pi/2)
    T7 = dh(0.14, q[6], 0, 0)

    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T

# Jacobiano usando diferencias finitas
def jacobiano(q, delta=1e-6):
    J = np.zeros((6, 7))
    T0 = fkine_irb(q)
    
    for i in range(7):
        dq = np.copy(q)
        dq[i] += delta
        Ti = fkine_irb(dq)
        J[0:3, i] = (Ti[0:3, 3] - T0[0:3, 3]) / delta
        Ri = Ti[0:3, 0:3].dot(T0[0:3, 0:3].T)
        J[3:6, i] = (1 / delta) * 0.5 * np.array([
            Ri[2, 1] - Ri[1, 2],
            Ri[0, 2] - Ri[2, 0],
            Ri[1, 0] - Ri[0, 1]
        ])
    return J

# Inicializar el nodo
rospy.init_node("KineControlPositionOrientation2")
# Publicador: publicar en el tópico joint_states
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

# Crear los marcadores
bmarker_actual = BallMarker(color['GREEN'])
bmarker_deseado = BallMarker(color['RED'])

# Nombres de las articulaciones
jnames = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_4n", "joint_5", "joint_6")
# Constantes
k = 4            # Constante de ganancia cinemática  
dt = 0.05           # Tiempo de control (cada cuánto se envía la señal al robot)
epsilon = 1e-3      # Máximo error admisible

# Configuración articular inicial del robot
q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Posición deseada del efector final  # Ajustada a una posición alcanzable en el espacio cartesiano
#xd = np.array([1.0, 0.5, 1.5, 0, 0, 0])  # caso 1
xd = np.array([1.5, 1, 2, 0, 0, 0])  # caso 2

# Crear una instancia del mensaje JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Añadir el valor de la articulación principal (con valor 0) a las articulaciones
jstate.position = q
# Frecuencia (en Hz) y período de control 
freq = 100
dt = 1.0 / freq
rate = rospy.Rate(freq)

# Listas para almacenar el error y el tiempo (para realizar los gráficos posteriores)
ee = []
xx = []
qq = []
tt = []
t = 0

# Control cinemático
cnt = 1    # Contador

while not rospy.is_shutdown():
    # Hora actual (necesaria para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Jacobiano para la configuración actual q
    J = jacobiano(q)
    # Cinemática directa dada la configuración actual q
    T = fkine_irb(q)
    x = T[0:3, 3]
    R = T[0:3, 0:3]

    # Convertir la matriz de rotación a ángulos de Euler (o cualquier otra representación de ángulos)
    # Aquí se asume que queremos alcanzar una orientación específica (identidad)
    e_rot = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    # Error en la posición y orientación
    e = np.concatenate((xd[:3] - x, e_rot))

    # Almacenamiento de valores
    ee.append(e)
    xx.append(x)
    tt.append(t)
    qq.append(q)

    # Verificación si se llegó al punto deseado
    if np.linalg.norm(e) < epsilon:
        print("Se llegó al punto deseado en {:.3f} segundos".format(cnt * dt))
        print("Se llegó a la posición con un error de:", np.linalg.norm(e))
        break

    # Derivada del error
    de = k * e
    # Variación de la configuración articular
    dq = np.linalg.pinv(J).dot(de)
    # Integración para obtener la nueva configuración articular
    q = q + dt * dq
    # Actualizar el tiempo
    t = t + dt
    
    # Solamente para evitar un bucle infinito si algo sale mal
    cnt += 1
    if cnt > 1e5:
        print("Se excedió el número de iteraciones")
        break

    # Log values
    with open('fxact.txt', 'a') as fxact:
        fxact.write(f"{x[0]} {x[1]} {x[2]}\n")
    with open('fxdes.txt', 'a') as fxdes:
        fxdes.write(f"{e[0]} {e[1]} {e[2]}\n")
    with open('fq.txt', 'a') as fq:
        fq.write(" ".join(map(str, q)) + '\n')

    # Publish the message
    jstate.position = q
    pub.publish(jstate)

    # Actualizar el marcador actual
    bmarker_actual.xyz([x[0], x[1], x[2]])
    # Publicar la posición deseada
    bmarker_deseado.xyz([xd[0], xd[1], xd[2]])
    
    # Esperar para la siguiente iteración
    rate.sleep()

# Visualización de la configuración final del robot
plt.plot(xd[0], xd[1], 'og')
plt.title("Configuración Final del Robot")
plt.show()

# Gráficos del error en el espacio cartesiano
e = np.array(ee)
plt.figure(figsize=(12, 12))
plt.suptitle("Errores en el Espacio Cartesiano y de Rotación")

# Gráfico del error en x
plt.subplot(3, 3, 1)
plt.plot(np.array(tt), e[:, 0])
plt.xlabel("tiempo [s]")
plt.ylabel("error en x [m]")
plt.title("Error en x")
plt.grid()

# Gráfico del error en y
plt.subplot(3, 3, 2)
plt.plot(np.array(tt), e[:, 1])
plt.xlabel("tiempo [s]")
plt.ylabel("error en y [m]")
plt.title("Error en y")
plt.grid()

# Gráfico del error en z
plt.subplot(3, 3, 3)
plt.plot(np.array(tt), e[:, 2])
plt.xlabel("tiempo [s]")
plt.ylabel("error en z [m]")
plt.title("Error en z")
plt.grid()

# Gráfico del error en rotación x
plt.subplot(3, 3, 4)
plt.plot(np.array(tt), e[:, 3])
plt.xlabel("tiempo [s]")
plt.ylabel("error en rot x")
plt.title("Error en rot x")
plt.grid()

# Gráfico del error en rotación y
plt.subplot(3, 3, 5)
plt.plot(np.array(tt), e[:, 4])
plt.xlabel("tiempo [s]")
plt.ylabel("error en rot y")
plt.title("Error en rot y")
plt.grid()

# Gráfico del error en rotación z
plt.subplot(3, 3, 6)
plt.plot(np.array(tt), e[:, 5])
plt.xlabel("tiempo [s]")
plt.ylabel("error en rot z")
plt.title("Error en rot z")
plt.grid()

plt.tight_layout(pad=3.0)
plt.show()

# Gráficos de la trayectoria (cartesiana) del efector final
x = np.array(xx)
plt.figure(figsize=(10, 5))
plt.suptitle("Trayectoria del Efector Final en el Espacio Cartesiano")

# Gráfico de x
plt.subplot(1, 3, 1)
plt.plot(np.array(tt), x[:, 0])
plt.plot(np.array(tt), xd[0] * np.ones(x.shape[0]), '--')
plt.xlabel("tiempo [s]")
plt.ylabel("x [m]")
plt.title("Trayectoria en x")
plt.grid()
plt.legend(['x(t)', '$x_{des}$'])

# Gráfico de y
plt.subplot(1, 3, 2)
plt.plot(np.array(tt), x[:, 1])
plt.plot(np.array(tt), xd[1] * np.ones(x.shape[0]), '--')
plt.xlabel("tiempo [s]")
plt.ylabel("y [m]")
plt.title("Trayectoria en y")
plt.grid()
plt.legend(['y(t)', '$y_{des}$'])

# Gráfico de z
plt.subplot(1, 3, 3)
plt.plot(np.array(tt), x[:, 2])
plt.plot(np.array(tt), xd[2] * np.ones(x.shape[0]), '--')
plt.xlabel("tiempo [s]")
plt.ylabel("z [m]")
plt.title("Trayectoria en z")
plt.grid()
plt.tight_layout(pad=3.0)
plt.legend(['z(t)', '$z_{des}$'])
plt.show()

# Gráfico de la trayectoria de las articulaciones
q = np.array(qq)
plt.figure(figsize=(15, 10))
plt.suptitle("Trayectorias Articulares")

# Gráfico de q1
plt.subplot(3, 3, 1)
plt.plot(np.array(tt), q[:, 0])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_1$ [rad]")
plt.title("Trayectoria articular q1")
plt.grid()

# Gráfico de q2
plt.subplot(3, 3, 2)
plt.plot(np.array(tt), q[:, 1])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_2$ [rad]")
plt.title("Trayectoria articular q2")
plt.grid()

# Gráfico de q3
plt.subplot(3, 3, 3)
plt.plot(np.array(tt), q[:, 2])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_3$ [rad]")
plt.title("Trayectoria articular q3")
plt.grid()

# Gráfico de q4
plt.subplot(3, 3, 4)
plt.plot(np.array(tt), q[:, 3])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_4$ [rad]")
plt.title("Trayectoria articular q4")
plt.grid()

# Gráfico de q5
plt.subplot(3, 3, 5)
plt.plot(np.array(tt), q[:, 4])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_5$ [rad]")
plt.title("Trayectoria articular q5")
plt.grid()

# Gráfico de q6
plt.subplot(3, 3, 6)
plt.plot(np.array(tt), q[:, 5])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_6$ [rad]")
plt.title("Trayectoria articular q6")
plt.grid()

# Gráfico de q7
plt.subplot(3, 3, 7)
plt.plot(np.array(tt), q[:, 6])
plt.xlabel("tiempo [s]")
plt.ylabel("$q_7$ [rad]")
plt.title("Trayectoria articular q7")
plt.grid()

plt.tight_layout(pad=3.0)
plt.show()

# Trayectoria cartesiana en el plano XY
plt.figure(figsize=(8, 8))
plt.plot(x[:, 0], x[:, 1])
plt.plot(xd[0], xd[1], 'or')
plt.legend(['Trayectoria', 'Punto deseado'])
plt.title("Trayectoria cartesiana del efector final (XY)")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.grid()
plt.show()

# Trayectoria cartesiana en el plano XZ
plt.figure(figsize=(8, 8))
plt.plot(x[:, 0], x[:, 2])
plt.plot(xd[0], xd[2], 'or')
plt.legend(['Trayectoria', 'Punto deseado'])
plt.title("Trayectoria cartesiana del efector final (XZ)")
plt.xlabel("x [m]")
plt.ylabel("z [m]")
plt.grid()
plt.show()