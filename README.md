ESSE REPOSITORIO POSSUI OS ARQUIVOS NECESSARIOS PARA REALIZAR AS SIMULACOES DO TURTLEBOT3 COM OPEN MANIPULATOR,
BEM COMO UM PASSO A PASSO DOS COMANDOS NECESSARIOS TANTO PARA A A MANIPULACAO QUANTO NAVEGACAO, ESTE ULTIMO
INCLUINDO MAPEAMENTO, LOCALIZACAO E PLANEJAMENTO DE TRAJETORIA.

VERSAO ROS: NOETIC
____________________________________________________________________________________________________________________________________

REFERENCIAS:

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/

https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation

____________________________________________________________________________________________________________________________________

PASSO A PASSO:

CRIAR UMA PASTA CATKIN_WS VAZIA E INCLUIR UMA SUBPASTA SRC

$ cd ~/catkin_ws/src/

sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git

$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git

$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git

$ sudo apt install ros-noetic-ros-control* && ros-noetic-control* 

$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
  
$ sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*

$ sudo apt install ros-noetic-robotis-manipulator

source /opt/ros/noetic/setup.bash

$ cd ~/catkin_ws && catkin_make 


____________________________________________________________________________________________________________________________________

DEPOIS EXECUTAR EM CADA TERMINAL QUE FOR ABRIR:

source devel/setup.bash && export TURTLEBOT3_MODEL=waffle_pi

____________________________________________________________________________________________________________________________________

PARA MANIPULACAO

- no arquivo:
~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/turtlebot3_manipulation_gazebo.launch
incluir abaixo da linha 10:
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/> 
mudar na linha 24 os seguintes valores:
    args="-urdf -param robot_description -model robot -x -3.0 -y 1.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>
Salvar e fechar o arquivo. 

TERMINAL 1: APOS DIGITAR O COMANDO ABAIXO, LEMBRAR DE DAR PLAY NA SIMULACAO DO GAZEBO

roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2:

roslaunch turtlebot3_manipulation_moveit_config move_group.launch

TERMINAL 3:

roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch

OU

roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch

TERMINAL 4:

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


____________________________________________________________________________________________________________________________________

PARA MAPEAMENTO, LOCALIZACAO E NAVEGACAO (COM MANIPULATOR) 

- no arquivo:
~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/turtlebot3_manipulation_gazebo.launch
incluir abaixo da linha 10:
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/> 
mudar na linha 24 os seguintes valores:
    args="-urdf -param robot_description -model robot -x -3.0 -y 1.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>
Salvar e fechar o arquivo.

TERMINAL 1:
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

TERMINAL 2:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

TERMINAL 3:
roslaunch turtlebot3_manipulation_slam slam.launch


APOS ANDAR COM O ROBO PELO AMBIENTE PARA MAPEAR PODE PARAR
O TEMRINAL 2 E DAR O SEGUINTE COMANDO PARA SALVAR O MAPA:
rosrun map_server map_saver -f ~/map
O MAPA SERA SALVO NO DIRETORIO ~/ COM O NOME MAP

EM SEGUIDA PODE PARAR O TERMINAL 3 E DAR O SEGUINTE COMANDO
roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=$HOME/map.yaml

NO RVIZ USAR O 2D POSE ESTIMATION PARA DAR UMA ESTIMATIVA DA 
POSICAO INICIAL. NO TERMINAL 2 PODE REABRIR O TELEOP ATE QUE
A LOCALIZACAO TENHA CONVERGIDO. POR FIM, DEVE-SE PARAR O TERMINAL 2
E EM SEGUIDA PODE-SE USAR O 2D NAV GOAL
PARA DAR A POSICAO FINAL QUE SE DESEJA PARA O ROBO MOVER.


____________________________________________________________________________________________________________________________________

PARA MAPEAMENTO, LOCALIZACAO E NAVEGACAO (SEM MANIPULATOR)

TERMINAL 1:
roslaunch turtlebot3_gazebo turtlebot3_house.launch

TERMINAL 2:
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

TERMINAL 3:
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

APOS ANDAR COM O ROBO PELO AMBIENTE PARA MAPEAR PODE PARAR
O TEMRINAL 2 E DAR O SEGUINTE COMANDO PARA SALVAR O MAPA:
rosrun map_server map_saver -f ~/map
O MAPA SERA SALVO NO DIRETORIO ~/ COM O NOME MAP

EM SEGUIDA PODE PARAR O TERMINAL 3 E DAR O SEGUINTE COMANDO
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

NO RVIZ USAR O 2D POSE ESTIMATION PARA DAR UMA ESTIMATIVA DA 
POSICAO INICIAL. NO TERMINAL 2 PODE REABRIR O TELEOP ATE QUE
A LOCALIZACAO TENHA CONVERGIDO. POR FIM, DEVE-SE PARAR O TERMINAL 2
E EM SEGUIDA PODE-SE USAR O 2D NAV GOAL
PARA DAR A POSICAO FINAL QUE SE DESEJA PARA O ROBO MOVER.

____________________________________________________________________________________________________________________________________



