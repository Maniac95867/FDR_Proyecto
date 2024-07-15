#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

if __name__ == "__main__":

    rospy.init_node("sendJointsGzNode")
    
    # Definir los tópicos para cada articulación
    topic1 = '/robot/joint1_position_controller/command'
    topic2 = '/robot/joint2_position_controller/command'
    topic3 = '/robot/joint3_position_controller/command'
    topic4 = '/robot/joint4_position_controller/command'
    topic4n = '/robot/joint4n_position_controller/command'
    topic5 = '/robot/joint5_position_controller/command'
    topic6 = '/robot/joint6_position_controller/command'
    
    # Crear publicadores para cada articulación
    pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
    pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
    pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
    pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
    pub4n = rospy.Publisher(topic4n, Float64, queue_size=10, latch=True)
    pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
    pub6 = rospy.Publisher(topic6, Float64, queue_size=10, latch=True)
    
    # Crear objetos Float64 para almacenar los comandos de posición
    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()
    j4n = Float64()
    j5 = Float64()
    j6 = Float64()
    
    # Asignar los valores deseados a cada articulación (ejemplo: 0.5 para todas)
    j1.data = 0.2
    j2.data = 0.5
    j3.data = 0.3
    j4.data = 0.5
    j4n.data = 1.2
    j5.data = 0.1
    j6.data = 0.6

    # Publicar los comandos de posición para cada articulación
    pub1.publish(j1)
    pub2.publish(j2)
    pub3.publish(j3)
    pub4.publish(j4)
    pub4n.publish(j4n)
    pub5.publish(j5)
    pub6.publish(j6)

    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)
    while not rospy.is_shutdown():
        rate.sleep()
