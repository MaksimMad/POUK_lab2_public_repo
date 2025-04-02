## Установка ros1_bridge

```sudo apt-get install ros-foxy-ros1-bridge```

## Запуск моста
В первом терминале запустите ROS1

```source /opt/ros/noetic/setup.bash```

```roscore```

В другом терминале запустите мост на ROS2:

```source /opt/ros/noetic/setup.bash```

```source /opt/ros/foxy/setup.bash```

```ros2 run ros1_bridge dynamic_bridge```


## Запуск симулятора Stage
Установка Stage

```sudo apt-get install ros-noetic-stage-ros```

Запуск Stage

```source /opt/ros/noetic/setup.bash```

из корневой папки (POUK_lab2_public_repo):

```rosrun stage_ros stageros src/worlds/task1.world```

## Сборка и запуск проекта на ROS2
из корневой папки (POUK_lab2_public_repo):

```source /opt/ros/foxy/setup.bash```

```colcon build```

```source install/setup.bash```

```ros2 run lab_2 robot_control```

