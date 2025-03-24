# Robôs autônomos
Esse repositório contém a implementação de um sistema de navegação autônoma no ROS 2, usando RRT e um robô equipado com sensores para explorar e se mover por um ambiente a fim de detectar uma pessoa.
## 📂 **Instalação e Configuração**
### 1️⃣ Clonar o repositório
```bash
https://github.com/elizabeth21S/Robot-exploration.git
```
### 2️⃣Configurar o ambiente do ROS 2
Certifique-se de que o ROS 2 Humble está instalado e configurado corretamente. Em seguida, compile e configure o ambiente:
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
### 3️⃣ Iniciar a simulação e a navegação

Para iniciar a simulação e o sistema de navegação:
```bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic

ros2 launch rrt_navigation rrt_worldlaunch.py headless:=False world:=ruta/to/world slam:=True
```

### 4️⃣ Iniciar a detecção de obstáculos
Para iniciar a detecção, executar em outro terminal:
```bash
ros2 run rrt_navigation test_camera
```


### 🛠 Requisitos

✅ ROS 2 Humble

✅ Clearpath Simulator: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install

✅ OpenCV : https://ibrahimmansur4.medium.com/integrating-opencv-with-ros2-a-comprehensive-guide-to-computer-vision-in-robotics-66b97fa2de92
