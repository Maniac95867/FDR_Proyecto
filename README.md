# FDR_Proyecto
Proyecto de Fundamentos de Robotica

Toda la informacion del proyecto se encuentra en la carpeta de "proyecto_ws".  
Antes de usar los archivos del proyecto, se requiere de ejecutar los siguientes comandos en 2 terminales diferentes.  

terminal 1:  
source /home/<user>/proyecto_ws/devel/setup.bash
roslaunch dependencia rviz.launch

Con el primer comando se configura el entorno del sistema para que los paquetes y las dependencias de ROS estén disponibles.  
Con el segundo se ejecuta el simulador RVIZ


terminal 2:  
export PYTHONPATH=$PYTHONPATH:/home/<user>/proyecto_ws/src/rbdl/build/python  
python3  
import rbdl  
quit()  
rosrun dependencia control_dinamico_pdg.py  

Con el primer comando se agrega la libreria RBDL al modulo que Python puede buscar para importar.  
Con la segunda y tercera linea se buscar entrar al entorno de PYTHON e importar la libreria RBDL.  
Con la tercera linea se sale del entorno de PYTHON.  
Con la ultima linea se ejecuta el script de control dinamico  



