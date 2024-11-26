# Создание рабочего пространства
# 1 Что такое пакет ROS 2?
Пакет – это организационная единица для кода ROS 2. Если вы хотите иметь возможность установить свой код или поделиться им с другими, вам потребуется организовать его в пакете. С помощью пакетов вы можете выпустить свою работу ROS 2 и позволить другим пользователям легко собирать и использовать ее.

Создание пакетов в ROS 2 использует ament в качестве системы сборки и colcon в качестве инструмента сборки. Вы можете создать пакет с помощью CMake или Python, которые официально поддерживаются, хотя существуют и другие типы сборки.
# 2 Из чего состоит пакет ROS 2?
Пакеты ROS 2 Python и CMake имеют собственное минимально необходимое содержимое:
Питон:
```
package.xml файл, содержащий метаинформацию о пакете

resource/<package_name> Маркерный файл для пакета

setup.cfg требуется, когда пакет содержит исполняемые файлы, поэтому может их найтиros2 run

setup.py содержащий инструкции по установке пакета

<package_name> - каталог с тем же именем, что и ваш пакет, используемый инструментами ROS 2 для поиска вашего пакета, содержит __init__.py
```
Самый простой из возможных пакетов может иметь файловую структуру, которая выглядит следующим образом:
```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```
# 3 Пакеты в рабочем пространстве
Одно рабочее пространство может содержать любое количество пакетов, каждый из которых находится в своей папке. Вы также можете иметь пакеты разных типов сборки в одном рабочем пространстве (CMake, Python и т. д.). У вас не может быть вложенных пакетов.

Рекомендуется иметь папку в рабочем пространстве и создавать в ней свои пакеты. Это позволяет сохранить верхний уровень рабочего пространства «чистым».src

Тривиальное рабочее пространство может выглядеть следующим образом:

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```
# Необходимые условия
У вас должно быть рабочее пространство ROS 2 после выполнения инструкций в предыдущем руководстве. Вы создадите свой пакет в этой рабочей области.
# Задачи
## 1 Создайте пакет
Во-первых, установите исходный код для установки ROS 2.

Давайте воспользуемся рабочим пространством, созданным в предыдущем руководстве, для нового пакета.ros2_ws

Убедитесь, что вы находитесь в папке, прежде чем запускать команду создания пакета.src

```
cd ~/ros2_ws/src

```
Синтаксис команды для создания нового пакета в ROS 2 следующий:
```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```
В этом руководстве вы будете использовать необязательные аргументы и . создает простой исполняемый файл типа Hello World в пакете и объявляет информацию о лицензии для пакета.--node-name--license--node-name--license

Введите следующую команду в терминале:

```
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
```
Теперь у вас будет новая папка в каталоге рабочей области с именем .srcmy_package

После выполнения команды ваш терминал вернет сообщение:
```
going to create a new package
package name: my_package
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['Apache-2.0']
build type: ament_python
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source folder
creating folder ./my_package/my_package
creating ./my_package/setup.py
creating ./my_package/setup.cfg
creating folder ./my_package/resource
creating ./my_package/resource/my_package
creating ./my_package/my_package/__init__.py
creating folder ./my_package/test
creating ./my_package/test/test_copyright.py
creating ./my_package/test/test_flake8.py
creating ./my_package/test/test_pep257.py
creating ./my_package/my_package/my_node.py
```
Вы можете увидеть автоматически сгенерированные файлы для нового пакета.
# 2 Соберите пакет
Размещение пакетов в рабочем пространстве особенно ценно, так как вы можете создать много пакетов одновременно, выполняя их в корневом каталоге рабочего пространства. В противном случае вам придется собирать каждый пакет отдельно.colcon build

Вернитесь в корень рабочего пространства:
```
cd ~/ros2_ws
```
Теперь вы можете собирать свои пакеты:
```
colcon build
```
Вспомните из предыдущего руководства, что у вас также есть пакеты в вашем домене . Возможно, вы заметили, что running также собирает пакет. Это нормально, когда у вас всего несколько пакетов в рабочем пространстве, но когда пакетов много, это может занять много времени.ros_tutorialsros2_wscolcon buildturtlesimcolcon build

Чтобы в следующий раз собрать только пакет, вы можете выполнить:my_package  
```
colcon build --packages-select my_package
```
# 3 Источник установочного файла

Чтобы использовать новый пакет и исполняемый файл, сначала откройте новый терминал и установите основную установку ROS 2.

Затем в каталоге выполните следующую команду для получения исходного кода рабочей области:ros2_ws
```
source install/local_setup.bash
```
Теперь, когда ваше рабочее пространство добавлено в ваш путь, вы сможете использовать исполняемые файлы нового пакета.
# 4 Используйте пакет
Чтобы запустить созданный вами исполняемый файл с использованием аргумента во время создания пакета, введите команду:--node-name
```
ros2 run my_package my_node
```
Который вернет сообщение на ваш терминал:
```Hi from my_package.```

# 5 Изучите содержимое упаковки
Внутри вы увидите файлы и папки, которые автоматически сгенерированы:ros2_ws/src/my_packageros2 pkg create
```my_package  package.xml  resource  setup.cfg  setup.py  test```
my_node.py находится внутри директории. Именно туда в будущем будут попадать все ваши пользовательские узлы Python.my_package
# 6 Настройка package.xml
Возможно, вы заметили в ответном сообщении после создания вашего пакета, что поля и содержат примечания. Это связано с тем, что описание пакета и декларация лицензии не задаются автоматически, но необходимы, если вы когда-нибудь захотите выпустить свой пакет. Поле также может потребоваться заполнить.descriptionlicenseTODOmaintainer

Из , откройте с помощью предпочитаемого вами текстового редактора:ros2_ws/src/my_packagepackage.xml
```
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```
Введите свое имя и адрес электронной почты в строку, если она не была заполнена автоматически. Затем отредактируйте строку, чтобы обобщить пакет:maintainerdescription
```<description>Beginner client libraries tutorials practice package</description>```
Затем обновите линию. Подробнее о лицензиях с открытым исходным кодом можно прочитать здесь. Поскольку этот пакет предназначен только для практики, безопасно использовать любую лицензию. Мы будем использовать :licenseApache-2.0
```<license>Apache-2.0</license>```

Не забудьте сохранить текст после завершения редактирования.

Под тегом лицензии вы увидите некоторые имена тегов, заканчивающиеся на . Здесь вы должны перечислить его зависимости от других пакетов, чтобы colcon мог их найти. проста и не имеет зависимостей, но вы увидите, как это пространство используется в следующих уроках._dependpackage.xmlmy_package
Файл содержит те же поля description, maintainer и license, что и , поэтому вам нужно установить и их. Они должны точно совпадать в обоих файлах. Версия и имя () также должны точно совпадать и должны быть автоматически заполнены в обоих файлах.setup.pypackage.xmlpackage_name

Откройте с помощью предпочитаемого текстового редактора.setup.py
```
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
 name=package_name,
 version='0.0.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'my_node = my_py_pkg.my_node:main'
     ],
   },
)
Отредактируйте строки , , и для сопоставления .maintainermaintainer_emaildescriptionpackage.xml

Не забудьте сохранить файл.
```
# Сводка
Вы создали пакет для организации кода и упрощения его использования для других пользователей.

Ваш пакет был автоматически заполнен необходимыми файлами, а затем вы использовали colcon для его сборки, чтобы вы могли использовать его исполняемые файлы в своей локальной среде.
