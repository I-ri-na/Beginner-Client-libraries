# ROS 2 Publisher/Subscriber Tutorial

## Введение
В этом учебном пособии вы создадите узлы, которые передают информацию в виде строковых сообщений друг другу через топик. В качестве примера используется простая система «talker» и «listener»; один узел публикует данные, а другой подписывается на топик, чтобы получать эти данные.



## Предварительные условия
В предыдущих учебных пособиях вы узнали, как создать рабочее пространство и пакет.

Базовое понимание Python рекомендуется, но не является обязательным.

## Задачи

### 1. Создание пакета
Откройте новый терминал и инициализируйте вашу установку ROS 2, чтобы команды `ros2` работали.

Перейдите в директорию `ros2_ws`, созданную в предыдущем учебном пособии.

Вспомните, что пакеты должны создаваться в директории `src`, а не в корне рабочего пространства. Перейдите в `ros2_ws/src` и выполните команду создания пакета:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

Ваш терминал вернет сообщение, подтверждающее создание вашего пакета `py_pubsub` и всех необходимых файлов и папок.

### 2. Написание узла publisher
Перейдите в `ros2_ws/src/py_pubsub/py_pubsub`. Вспомните, что это директория Python-пакета с тем же именем, что и вложенный в него пакет ROS 2.

Скачайте пример кода talker, выполнив следующую команду:

```bash
wget https://raw.githubusercontent.com/ros2/examples/rolling/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

Теперь появится новый файл с именем `publisher_member_function.py` рядом с `__init__.py`.

Откройте файл с помощью вашего любимого текстового редактора.

```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = MinimalPublisher()

            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
```

#### 2.1 Изучение кода
Первые строки кода после комментариев импортируют `rclpy`, чтобы его класс мог быть использован.

```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
```

Следующее выражение импортирует встроенный тип строкового сообщения, который узел использует для структурирования данных, передаваемых по топику.

```python
from std_msgs.msg import String
```

Эти строки представляют зависимости узла. Вспомните, что зависимости должны быть добавлены в `package.xml`, что вы сделаете в следующем разделе.

Затем создается класс, который наследуется от (или является подклассом) `Node`.

```python
class MinimalPublisher(Node):
```

Далее следует определение конструктора класса. `super().__init__` вызывает конструктор класса и дает ему имя вашего узла, в данном случае `minimal_publisher`.

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

`create_publisher` объявляет, что узел публикует сообщения типа `String` (импортированного из модуля `std_msgs.msg`) по топику с именем `topic`, и что «размер очереди» равен 10. Размер очереди — это обязательная настройка QoS (quality of service), которая ограничивает количество сообщений в очереди, если подписчик не получает их достаточно быстро.

Затем создается таймер с обратным вызовом для выполнения каждые 0.5 секунды. `self.i` — это счетчик, используемый в обратном вызове.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

`timer_callback` создает сообщение с добавленным значением счетчика и публикует его в консоль с помощью `get_logger().info`.

Наконец, определяется основная функция.

```python
def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_publisher = MinimalPublisher()

            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
```

Сначала инициализируется библиотека, затем создается узел, и затем он «вращает» узел, чтобы его обратные вызовы были вызваны.

#### 2.2 Добавление зависимостей
Перейдите на уровень выше в директорию `ros2_ws/src/py_pubsub`, где для вас были созданы файлы `setup.py`, `setup.cfg` и `package.xml`.

Откройте `package.xml` с помощью вашего текстового редактора.

Как упоминалось в предыдущем учебном пособии, убедитесь, что заполнены теги `<description>`, `<maintainer>` и `<license>`:

```xml
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache-2.0</license>
```

После строк выше добавьте следующие зависимости, соответствующие вашим операторам import:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Это объявляет, что пакету требуются `rclpy` и `std_msgs` при выполнении его кода.

Не забудьте сохранить файл.

#### 2.3 Добавление точки входа
Откройте файл `setup.py`. Снова сопоставьте поля `maintainer`, `maintainer_email`, `description` и `license` с вашим `package.xml`.

```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache-2.0',
```

Добавьте следующую строку внутри скобок поля `entry_points`:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

Не забудьте сохранить файл.

#### 2.4 Проверка setup.cfg
Содержимое файла должно быть автоматически заполнено правильно, как показано ниже:

```ini
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

Это просто указывает `setuptools` поместить ваши исполняемые файлы в `lib`, потому что `ros2 run` будет искать их там.

Вы могли бы собрать свой пакет сейчас, инициализировать локальные файлы настройки и запустить его, но давайте сначала создадим узел subscriber, чтобы вы могли увидеть работу всей системы.

### 3. Написание узла subscriber
Вернитесь в `ros2_ws/src/py_pubsub/py_pubsub`, чтобы создать следующий узел. Введите следующую команду в вашем терминале:

```bash
wget https://raw.githubusercontent.com/ros2/examples/rolling/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

Теперь в директории должны быть следующие файлы:

```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

#### 3.1 Изучение кода
Откройте `subscriber_member_function.py` с помощью вашего текстового редактора.

```python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
```

Код узла subscriber практически идентичен коду publisher. Конструктор создает подписчика с теми же аргументами, что и publisher. Вспомните из учебного пособия по топикам, что имя топика и тип сообщения, используемые publisher и subscriber, должны совпадать, чтобы они могли общаться.

```python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

Конструктор и обратный вызов subscriber не включают определение таймера, потому что ему это не нужно. Его обратный вызов вызывается сразу после получения сообщения.

Определение обратного вызова просто выводит информационное сообщение в консоль вместе с полученными данными. Вспомните, что publisher определяет `msg.data = 'Hello World: %d' % self.i`.

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

Определение почти полностью совпадает, за исключением создания и вращения publisher на subscriber.

```python
minimal_subscriber = MinimalSubscriber()

rclpy.spin(minimal_subscriber)
```

Поскольку этот узел имеет те же зависимости, что и publisher, в `package.xml` нечего добавлять. Файл `setup.cfg` также может оставаться нетронутым.

#### 3.2 Добавление точки входа
Повторно откройте `setup.py` и добавьте точку входа для узла subscriber ниже точки входа publisher. Поле `entry_points` теперь должно выглядеть так:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

Не забудьте сохранить файл, и тогда ваша система pub/sub должна быть готова.

### 4. Сборка и запуск
Вероятно, у вас уже установлены пакеты `rclpy` и `std_msgs` как часть вашей системы ROS 2. Хорошей практикой является запуск `rosdep` в корне вашего рабочего пространства (`ros2_ws`), чтобы проверить отсутствующие зависимости перед сборкой:

```bash
rosdep install -i --from-path src --rosdistro rolling -y
```

Все еще находясь в корне вашего рабочего пространства, `ros2_ws`, соберите ваш новый пакет:

```bash
colcon build --packages-select py_pubsub
```

Откройте новый терминал, перейдите в `ros2_ws` и инициализируйте файлы настройки:

```bash
source install/setup.bash
```

Теперь запустите узел talker:

```bash
ros2 run py_pubsub talker
```

Терминал должен начать публиковать информационные сообщения каждые 0.5 секунды, например:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

Откройте другой терминал, инициализируйте файлы настройки из `ros2_ws` еще раз, а затем запустите узел listener:

```bash
ros2 run py_pubsub listener
```

Listener начнет выводить сообщения в консоль, начиная с текущего счетчика сообщений publisher, например:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

Введите `Ctrl+C` в каждом терминале, чтобы остановить вращение узлов.

