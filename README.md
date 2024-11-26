# Использование для сборки пакетов colcon
Цель: Создание рабочей области ROS 2 с помощью .colcon
colcon является итерацией инструментов сборки ROS

# Необходимые условия
## Установка colcon
```sudo apt install python3-colcon-common-extensions```

## Установка ROS 2
# Основные
Рабочая область ROS — это директория с определенной структурой. Обычно есть поддиректория. Внутри этого подкаталога находится исходный код пакетов ROS. Как правило, каталог начинается с пустого места.src

Colcon выполняет сборки из исходного кода. По умолчанию он создаст следующие директории в качестве одноранговых узлов каталога:src

В директории будут храниться промежуточные файлы. Для каждого пакета будет создана подпапка, в которой, например, будет вызываться CMake.build

Каталог — это место, куда будет установлен каждый пакет. По умолчанию каждый пакет будет установлен в отдельный подкаталог.install

Каталог содержит различную информацию о каждом вызове colcon.log

##Создание рабочего пространства
Сначала создадим каталог (), в котором будет содержаться наша рабочая область:ros2_ws
```mkdir -p ~/ros2_ws/src```
```cd ~/ros2_ws```
На этом этапе рабочая область содержит одну пустую директорию:src


![image](https://i.postimg.cc/fyLTMRVH/2024-11-26-085504.png)


# Добавьте несколько источников
Давайте клонируем репозиторий примеров в директорию рабочей области:src

```git clone https://github.com/ros2/examples src/examples -b rolling```
Теперь в рабочей области должен быть исходный код для примеров ROS 2:

```.
└── src```
  ```  └── examples```
       ``` ├── CONTRIBUTING.md```
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files```
# Источник подложки
Важно, чтобы мы нашли среду для существующей установки ROS 2, которая предоставит нашему рабочему пространству необходимые зависимости сборки для примеров пакетов. Это достигается путем использования установочного скрипта, предоставляемого двоичной установкой или исходной установкой, т.е. еще одно рабочее пространство colcon (см. раздел Установка). Мы называем эту среду подложкой.

Наше рабочее пространство будет наложено поверх существующей установки ROS 2. Как правило, рекомендуется использовать оверлей, когда вы планируете выполнять итерации по небольшому количеству пакетов, а не размещать все пакеты в одном рабочем пространстве.ros2_ws
# Создание рабочего пространства
В корневом каталоге рабочей области выполните . Поскольку такие типы сборки, как не поддерживают концепцию пространства и требуют установки пакета, colcon поддерживает опцию . Это позволяет изменять установленные файлы, изменяя файлы в пространстве (например, файлы Python или другие нескомпилированные ресурсы) для более быстрой итерации.colcon buildament_cmakedevel--symlink-installsource
```colcon build --symlink-install```
После того, как сборка будет завершена, мы должны увидеть каталоги , и следующие:buildinstalllog
