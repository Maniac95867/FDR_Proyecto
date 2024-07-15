#!/usr/bin/env python3

import rbdl
import numpy as np

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  modelo = rbdl.loadModel('/home/user/proyecto_ws/src/dependencia/urdf/gazebo.urdf')
  # Grados de libertad
  ndof = modelo.q_size

  # Configuracion articular
  q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6, 0.1])
  # Velocidad articular
  dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0, 0.1])
  # Aceleracion articular
  ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0.1])
  
  # Arrays numpy
  zeros = np.zeros(ndof)          # Vector de ceros
  tau   = np.zeros(ndof)          # Para torque
  g     = np.zeros(ndof)          # Para la gravedad
  c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
  M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
  e     = np.eye(6)               # Vector identidad
  M2 = np.zeros((ndof, ndof))  # Matriz de inercia
  b = np.zeros(ndof)  # Efectos no lineales
  

  
  # Torque dada la configuracion del robot
  rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
  print("Imprime TAU", np.round(tau, 3))

  # Calculo del vector gravedad
  rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
  print("Imprime GRAVEDAD:", np.round(g, 3))
  
  
  # Calculo del vector de Coriolis
  rbdl.InverseDynamics(modelo, q, dq, zeros, c)
  c = c - g
  print("Imprime CORIOLISIS:", np.round(c, 3))
  
  # Calculo de la matriz de Coriolisis
  for i in range(ndof-1):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
  print("Imprime M:", np.round(M, 3))

  tau_2 = M.dot(ddq)+c+g
  print("Imprime tau_2:", np.round(tau_2, 3))