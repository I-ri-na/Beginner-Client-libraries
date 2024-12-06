# Создание простого сервиса и клиента (Python)

## Цель
Создать и запустить узлы сервиса и клиента с использованием Python.

## Введение
Когда узлы общаются с использованием сервисов, узел, отправляющий запрос на данные, называется клиентским узлом, а узел, отвечающий на запрос, называется узлом сервиса. Структура запроса и ответа определяется файлом `.srv`.

В данном примере используется простая система сложения целых чисел; один узел запрашивает сумму двух целых чисел, а другой отвечает результатом.

## Предварительные условия
В предыдущих учебных пособиях вы узнали, как создать рабочее пространство и пакет.

## Задачи

### 1. Создание пакета
Откройте новый терминал и инициализируйте вашу установку ROS 2, чтобы команды `ros2` работали.

Перейдите в директорию `ros2_ws`, созданную в предыдущем учебном пособии.

Вспомните, что пакеты должны создаваться в директории `src`, а не в корне рабочего пространства. Перейдите в `ros2_ws/src` и создайте новый пакет:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces
```

Ваш терминал вернет сообщение, подтверждающее создание вашего пакета `py_srvcli` и всех необходимых файлов и папок.

Аргумент `--dependencies` автоматически добавит необходимые строки зависимостей в `package.xml`. Пакет `example_interfaces` включает файл `.srv`, который вам понадобится для структурирования ваших запросов и ответов:

```
int64 a
int64 b
---
int64 sum
```

Первые две строки — это параметры запроса, а ниже черты — ответ.

#### 1.1 Обновление package.xml
Поскольку вы использовали опцию `--dependencies` при создании пакета, вам не нужно вручную добавлять зависимости в `package.xml`.

Однако, как всегда, убедитесь, что вы добавили описание, email и имя поддерживающего лица, а также информацию о лицензии в `package.xml`:

```xml
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache-2.0</license>
```

#### 1.2 Обновление setup.py
Добавьте ту же информацию в файл `setup.py` для полей `maintainer`, `maintainer_email`, `description` и `license`:

```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache-2.0',
```

### 2. Написание узла сервиса
В директории `ros2_ws/src/py_srvcli/py_srvcli` создайте новый файл с именем `service_member_function.py` и вставьте в него следующий код:

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def main():
    try:
        with rclpy.init():
            minimal_service = MinimalService()

            rclpy.spin(minimal_service)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
```

#### 2.1 Изучение кода
Первое оператор `import` импортирует тип сервиса `AddTwoInts` из пакета `example_interfaces`. Следующие операторы `import` импортируют необходимые интерфейсы библиотеки клиента Python ROS 2.

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
```

Конструктор класса `MinimalService` инициализирует узел с именем `minimal_service`. Затем он создает сервис и определяет тип, имя и обратный вызов.

```python
def __init__(self):
    super().__init__('minimal_service')
    self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```

Определение обратного вызова сервиса получает данные запроса, суммирует их и возвращает сумму в качестве ответа.

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    return response
```

Наконец, основной класс инициализирует библиотеку клиента Python ROS 2, создает экземпляр класса `MinimalService` для создания узла сервиса и вращает узел для обработки обратных вызовов.

#### 2.2 Добавление точки входа
Чтобы команда `ros2 run` могла запускать ваш узел, добавьте точку входа в `setup.py` (находится в директории `ros2_ws/src/py_srvcli`).

Добавьте следующую строку между скобками `'console_scripts'`:

```python
'service = py_srvcli.service_member_function:main',
```

### 3. Написание узла клиента
В директории `ros2_ws/src/py_srvcli/py_srvcli` создайте новый файл с именем `client_member_function.py` и вставьте в него следующий код:

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        return self.cli.call_async(self.req)

def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_client = MinimalClientAsync()
            future = minimal_client.send_request()
            rclpy.spin_until_future_complete(minimal_client, future)
            response = future.result()
            minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, response.sum))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
```

#### 3.1 Изучение кода
Как и в коде сервиса, сначала импортируются необходимые библиотеки.

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
```

Конструктор класса `MinimalClientAsync` инициализирует узел с именем `minimal_client_async`. Определение конструктора создает клиент с тем же типом и именем, что и узел сервиса. Тип и имя должны совпадать, чтобы клиент и сервис могли общаться. Цикл `while` в конструкторе проверяет, доступен ли сервис, соответствующий типу и имени клиента, один раз в секунду. Наконец, он создает новый объект запроса `AddTwoInts`.

```python
def __init__(self):
    super().__init__('minimal_client_async')
    self.cli = self.create_client(AddTwoInts, 'add_two_ints')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = AddTwoInts.Request()
```

Ниже конструктора находится метод `send_request`, который отправляет запрос и вращает узел до тех пор, пока не получит ответ или не потерпит неудачу.

```python
def send_request(self):
    self.req.a = 41
    self.req.b = 1
    return self.cli.call_async(self.req)
```

Наконец, у нас есть основной метод, который создает объект `MinimalClientAsync`, отправляет запрос с использованием переданных аргументов командной строки, вызывает `rclpy.spin_until_future_complete` для ожидания результата и регистрирует результаты.

```python
def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_client = MinimalClientAsync()
            future = minimal_client.send_request()
            rclpy.spin_until_future_complete(minimal_client, future)
            response = future.result()
            minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, response.sum))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
```

**Предупреждение**

Не используйте `rclpy.spin_until_future_complete` в обратном вызове ROS 2. Подробнее см. в статье о синхронной блокировке.

#### 3.2 Добавление точки входа
Как и в случае с узлом сервиса, вам также нужно добавить точку входа, чтобы иметь возможность запускать узел клиента.

Поле `entry_points` вашего файла `setup.py` должно выглядеть так:

```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

### 4. Сборка и запуск
Хорошей практикой является запуск `rosdep` в корне вашего рабочего пространства (`ros2_ws`), чтобы проверить отсутствующие зависимости перед сборкой:

```bash
rosdep install -i --from-path src --rosdistro rolling -y
```

Вернитесь в корень вашего рабочего пространства, `ros2_ws`, и соберите ваш новый пакет:

```bash
colcon build --packages-select py_srvcli
```

Откройте новый терминал, перейдите в `ros2_ws` и инициализируйте файлы настройки:

```bash
source install/setup.bash
```

Теперь запустите узел сервиса:

```bash
ros2 run py_srvcli service
```

Узел будет ожидать запроса клиента.

Откройте другой терминал и инициализируйте файлы настройки из `ros2_ws` еще раз. Запустите узел клиента:

```bash
ros2 run py_srvcli client
```

Клиент отправляет запрос сервису, который вычисляет сумму и возвращает результат. Клиент должен получить следующий ответ:

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 41 + 1 = 42
```

Вернитесь в терминал, где работает ваш узел сервиса. Вы увидите, что он опубликовал сообщения журнала, когда получил запрос:

```
[INFO] [minimal_service]: Incoming request
a: 41 b: 1
```

Введите `Ctrl+C` в терминале сервера, чтобы остановить вращение узла.

## Итог
Вы создали два узла для запроса и ответа на данные через сервис. Вы добавили их зависимости и исполняемые файлы в файлы конфигурации пакета, чтобы вы могли собрать и запустить их, что позволило вам увидеть работу системы сервис/клиент.
