<h1 align="center">Визуализатор .osm файлов</h1>
<p align="center">
🌏Модуль планирования траектории движения транспортного средства на базе ROS2, позволяющий строить маршрут из одной произвольной точки на карте в другую, а также иметь визуализацию построенного маршрута🌏
</p>

<h2>Установка</h2>
<h3>Ubuntu</h3>
<p>Установите Ubuntu 22.04 c <a href="https://releases.ubuntu.com/jammy/">официального сайта</a></p>
<h3>ROS2</h3>
<p>Установите ROS2 (Iron Irwini) по инструкциям с <a href="https://docs.ros.org/en/iron/Releases/Release-Iron-Irwini.html#installation">официального сайта</a></p>

<h2>Настройка</h2>


Пересоберите проект при каждом изменении или первоначальной установке: <br>

``
colcon build --packages-select osm_cartography
``
<br>
<br>
При первом запуске после сборки:
<br>
<br>
``
source install/setup.bash
``
<br>

### Настройте Rviz2 (или запустите вместе с окружением см п. "Запуск"):

1. Установите фиксированную рамку на "map"
2. Добавьте отображение MarkerArray
3. Установите тему MarkerArray на «/osm_markers».
4. Добавьте RobotModel
5. Добавьте TF 

Запустите:
<br>
<br>
### Запуск без окружения (запустите и настройте Rviz вручную)
``
ros2 run osm_cartography osm_cartography_node
``
### Запуск вместе с окружением
``
ros2 launch osm_cartography osm_cartography.launch.py
``