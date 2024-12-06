# Создание простого сервиса и клиента (C++)

## Цель
Создать и запустить узлы сервиса и клиента с использованием C++.

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
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces
```

Ваш терминал вернет сообщение, подтверждающее создание вашего пакета `cpp_srvcli` и всех необходимых файлов и папок.

Аргумент `--dependencies` автоматически добавит необходимые строки зависимостей в `package.xml` и `CMakeLists.txt`. Пакет `example_interfaces` включает файл `.srv`, который вам понадобится для структурирования ваших запросов и ответов:

```
int64 a
int64 b
---
int64 sum
```

Первые две строки — это параметры запроса, а ниже черты — ответ.

#### 1.1 Обновление package.xml
Поскольку вы использовали опцию `--dependencies` при создании пакета, вам не нужно вручную добавлять зависимости в `package.xml` или `CMakeLists.txt`.

Однако, как всегда, убедитесь, что вы добавили описание, email и имя поддерживающего лица, а также информацию о лицензии в `package.xml`:

```xml
<description>C++ client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache-2.0</license>
```

### 2. Написание узла сервиса
В директории `ros2_ws/src/cpp_srvcli/src` создайте новый файл с именем `add_two_ints_server.cpp` и вставьте в него следующий код:

```cpp
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("minimal_service");
  auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
```

#### 2.1 Изучение кода
Первые операторы `#include` — это зависимости вашего пакета.

Функция `handle_service` добавляет два целых числа из запроса и передает сумму в ответ, уведомляя консоль о своем состоянии с помощью логов.

```cpp
void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}
```

Функция `main` выполняет следующие действия по строкам:

Инициализирует библиотеку клиента C++ ROS 2:

```cpp
rclcpp::init(argc, argv);
```

Создает узел с именем `minimal_service`:

```cpp
g_node = rclcpp::Node::make_shared("minimal_service");
```

Создает сервис с именем `add_two_ints` для этого узла и автоматически объявляет его в сети с методом `handle_service`:

```cpp
auto server = g_node->create_service<AddTwoInts>("add_two_ints", handle_service);
```

Вращает узел, делая сервис доступным:

```cpp
rclcpp::spin(g_node);
```

#### 2.2 Добавление исполняемого файла
Макрос `add_executable` генерирует исполняемый файл, который можно запустить с помощью `ros2 run`. Добавьте следующий блок кода в `CMakeLists.txt`, чтобы создать исполняемый файл с именем `server`:

```cmake
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
```

Чтобы `ros2 run` мог найти исполняемый файл, добавьте следующие строки в конец файла, сразу перед `ament_package()`:

```cmake
install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
```

Вы могли бы собрать свой пакет сейчас, инициализировать локальные файлы настройки и запустить его, но давайте сначала создадим узел клиента, чтобы вы могли увидеть работу всей системы.

### 3. Написание узла клиента
В директории `ros2_ws/src/cpp_srvcli/src` создайте новый файл с именем `add_two_ints_client.cpp` и вставьте в него следующий код:

```cpp
#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_client");
  auto client = node->create_client<AddTwoInts>("add_two_ints");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    client->remove_pending_request(result_future);
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(
    node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
    request->a, request->b, result->sum);
  rclcpp::shutdown();
  return 0;
}
```

#### 3.1 Изучение кода
Аналогично узлу сервиса, следующие строки кода создают узел, а затем создают клиент для этого узла:

```cpp
auto node = rclcpp::Node::make_shared("minimal_client");
auto client = node->create_client<AddTwoInts>("add_two_ints");
```

Затем код ожидает появления сервиса. Цикл `while` дает клиенту 1 секунду для поиска узлов сервиса в сети. Если он не может найти ни одного, он будет продолжать ожидать. Если клиент будет отменен (например, если вы введете `Ctrl+C` в терминал), он вернет сообщение об ошибке, указывающее, что он был прерван.

```cpp
while (!client->wait_for_service(std::chrono::seconds(1))) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
}
```

Затем создается запрос. Его структура определяется файлом `.srv`, упомянутым ранее.

```cpp
auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
request->a = 41;
request->b = 1;
```

Затем клиент отправляет свой запрос, и узел вращается до тех пор, пока не получит ответ или не потерпит неудачу.

#### 3.2 Добавление исполняемого файла
Вернитесь в `CMakeLists.txt`, чтобы добавить исполняемый файл и цель для нового узла. После удаления некоторых ненужных шаблонов из автоматически сгенерированного файла, ваш `CMakeLists.txt` должен выглядеть так:

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### 4. Сборка и запуск
Хорошей практикой является запуск `rosdep` в корне вашего рабочего пространства (`ros2_ws`), чтобы проверить отсутствующие зависимости перед сборкой:

```bash
rosdep install -i --from-path src --rosdistro rolling -y
```

Вернитесь в корень вашего рабочего пространства, `ros2_ws`, и соберите ваш новый пакет:

```bash
colcon build --packages-select cpp_srvcli
```

Откройте новый терминал, перейдите в `ros2_ws` и инициализируйте файлы настройки:

```bash
source install/setup.bash
```

Теперь запустите узел сервиса:

```bash
ros2 run cpp_srvcli server
```

Терминал должен ожидать входящих запросов.

Откройте другой терминал, инициализируйте файлы настройки из `ros2_ws` еще раз. Запустите узел клиента, затем введите любые два целых числа, разделенные пробелом:

```bash
ros2 run cpp_srvcli client
```

Клиент отправляет запрос сервису, который вычисляет сумму и возвращает результат. Клиент должен получить следующий ответ:

```
[INFO] [minimal_client]: result of 41 + 1: 42
```

Вернитесь в терминал, где работает ваш узел сервиса. Вы увидите, что он опубликовал сообщения журнала, когда получил запрос и данные, которые он получил, а также ответ, который он отправил обратно:

```
[INFO] [minimal_service]: request: 41 + 1
```

Введите `Ctrl+C` в терминале сервера, чтобы остановить вращение узла.

## Итог
Вы создали два узла для запроса и ответа на данные через сервис. Вы добавили их зависимости и исполняемые файлы в файлы конфигурации пакета, чтобы вы могли собрать и запустить их и увидеть работу системы сервис/клиент.
