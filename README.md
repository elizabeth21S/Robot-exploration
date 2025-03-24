# Robôs autônomos
Esse repositório contém a implementação de um sistema de navegação autônoma no ROS 2, usando Rapidly-exploring Random Trees Connect (RRT-Connect) e um robô equipado com sensores para explorar e se mover por um ambiente a fim de detectar uma pessoa.
## 📂 **Instalação e Configuração**
### 1️⃣ Configurar o ambiente do ROS 2 e Clearpath
Use o arquivo de texto no repositório denominado instruções para instalar e configurar o Clearpath Simulator em seguida, siga as etapas abaixo.

### 2️⃣Clonar o repositório
Entre no espaço de trabalho e clone o repositório em src.
```bash
https://github.com/elizabeth21S/Robot-exploration.git
```

### 3️⃣ Iniciar a simulação
Para iniciar a simulação:
```bash
ros2 launch clearpath_gz simulantion.launch.py rviz:=true world:=name/world
```

### 4️⃣ Iniciar a detecção de pessoas e navegação 
Para iniciar a detecção, executar em outro terminal:
```bash
ros2 run rrt_explorer rrt_detect
```
Para iniciar a navegação, executar em outro terminal:
```bash
ros2 run rrt_explorer rrt_node
```

### 🛠 Requisitos

✅ ROS 2 Humble

✅ Clearpath Simulator: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install

✅ OpenCV : https://ibrahimmansur4.medium.com/integrating-opencv-with-ros2-a-comprehensive-guide-to-computer-vision-in-robotics-66b97fa2de92
