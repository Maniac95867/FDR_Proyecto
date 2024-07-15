#!/usr/bin/env python3

import numpy as np
from copy import copy
import matplotlib.pyplot as plt
import rbdl

cos = np.cos
sin = np.sin
pi = np.pi

# Definición de la función DH
def dh(d, theta, a, alpha):
    cth = np.cos(theta)
    sth = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    T = np.array([[cth, -ca * sth, sa * sth, a * cth],
                  [sth, ca * cth, -sa * cth, a * sth],
                  [0, sa, ca, d],
                  [0, 0, 0, 1]])
    return T

# Cinemática directa del robot IRB
def fkine_irb(q):

    # Longitudes (en metros)
    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.68, q[0],0.2,-pi/2)
    T2 = dh(0,pi/2+q[1],-0.89,0)
    T3 = dh(0,q[2],-0.15,pi/2)
    T4 = dh(1.733, q[3],0,0)
    T5 = dh(q[4], 0, 0, -pi/2)
    T6 = dh(0,q[5],0,pi/2)
    T7 = dh(0.14,q[6],0,0)

    T = T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6.dot(T7))))))
    return T

# Jacobiano analítico usando diferencias finitas
def jacobian_analitico(q, delta=0.0001):
    J = np.zeros((3, 7))
    T = fkine_irb(q)
    for i in range(7):
        dq = copy(q)
        dq[i] += delta
        dT = fkine_irb(dq)
        J[:, i] = 1 / delta * (dT[0:3, 3] - T[0:3, 3])
    return J

# Método de Newton-Raphson para la cinemática inversa
def ikine_irb_newton(xd, q0, max_iter=10000, epsilon=0.00001, delta=0.001):
    q = copy(q0)
    q_hist = [q0]
    error_hist = []
    for _ in range(max_iter):
        J = jacobian_analitico(q, delta)
        T = fkine_irb(q)
        e = xd - T[0:3, 3]
        q = q + np.dot(np.linalg.pinv(J), e)
        q_hist.append(q)
        error_hist.append(np.linalg.norm(e))
        if np.linalg.norm(e) < epsilon:
            break
    return q, q_hist, error_hist

# Método de gradiente para la cinemática inversa
def ikine_irb_gradient(xd, q0, max_iter=10000, epsilon=0.0001, delta=0.0001, alpha=0.1):
    q = copy(q0)
    q_hist = [q0]
    error_hist = []
    for _ in range(max_iter):
        J = jacobian_analitico(q, delta)
        T = fkine_irb(q)
        e = xd - T[0:3, 3]
        q = q + alpha * np.dot(J.T, e)
        q_hist.append(q)
        error_hist.append(np.linalg.norm(e))
        if np.linalg.norm(e) < epsilon:
            break
    return q, q_hist, error_hist

# Función para graficar las trayectorias y errores
def plot_results(q_hist_newton, q_hist_gradient, error_hist_newton, error_hist_gradient):
    fig, ax = plt.subplots(3, 1, figsize=(10, 15))

    # Trayectoria del efector final
    traj_newton = [fkine_irb(q)[0:3, 3] for q in q_hist_newton]
    traj_gradient = [fkine_irb(q)[0:3, 3] for q in q_hist_gradient]

    traj_newton = np.array(traj_newton)
    traj_gradient = np.array(traj_gradient)

    ax[0].plot(traj_newton[:, 0], traj_newton[:, 1], label='Newton-Raphson')
    ax[0].plot(traj_gradient[:, 0], traj_gradient[:, 1], label='Gradiente')
    ax[0].set_title('Trayectoria del efector final')
    ax[0].set_xlabel('X (m)')
    ax[0].set_ylabel('Y (m)')
    ax[0].legend()
    ax[0].grid()

    # Posiciones articulares
    q_hist_newton = np.array(q_hist_newton)
    q_hist_gradient = np.array(q_hist_gradient)

    for i in range(7):
        ax[1].plot(q_hist_newton[:, i], label=f'Joint {i+1} - Newton-Raphson')
        ax[1].plot(q_hist_gradient[:, i], label=f'Joint {i+1} - Gradiente', linestyle='dashed')

    ax[1].set_title('Posiciones articulares')
    ax[1].set_xlabel('Iteración')
    ax[1].set_ylabel('Ángulo (rad)')
    ax[1].legend()
    ax[1].grid()

    # Error en función de las iteraciones
    ax[2].plot(error_hist_newton, label='Newton-Raphson')
    ax[2].plot(error_hist_gradient, label='Gradiente')
    ax[2].set_title('Error en función de las iteraciones')
    ax[2].set_xlabel('Iteración')
    ax[2].set_ylabel('Error')
    ax[2].legend()
    ax[2].grid()

    plt.tight_layout()
    plt.show()

    # Ejemplo de uso de las funciones desarrolladas
    # Configuración articular inicial
    q0 = np.array([0, 0, 0, 0, 0, 0, 0])

    # Posición deseada en el espacio cartesiano
    xd = np.array([0.5, 0.5, 0.5])

    # Resolver la cinemática inversa usando el método de Newton-Raphson
    q_sol_newton, q_hist_newton, error_hist_newton = ikine_irb_newton(xd, q0)
    print("Solución de la cinemática inversa (Newton-Raphson):", np.round(q_sol_newton, 2))

    # Resolver la cinemática inversa usando el método de gradiente
    q_sol_gradient, q_hist_gradient, error_hist_gradient = ikine_irb_gradient(xd, q0)
    print("Solución de la cinemática inversa (Gradiente):", np.round(q_sol_gradient, 3))

    # Graficar las trayectorias y errores
    plot_results(q_hist_newton, q_hist_gradient, error_hist_newton, error_hist_gradient)

    return 0
#====================================================================================
# Para control dinamico

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('/home/user/proyecto_ws/src/dependencia/urdf/gazebo.urdf')
    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq
    def read_joint_positions(self):
        return self.q
    def read_joint_velocities(self):
        return self.dq



