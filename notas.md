Para ejecutar:
```bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica_1_interfaces practica_1
source install/setup.bash
```

```bash
cd ~/ros_workspace/src/practica1Robotica/practica_1/practica_1
ros2 run practica_1 extractor_cinematico
```