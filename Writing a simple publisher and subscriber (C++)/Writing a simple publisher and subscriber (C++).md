# ROS 2 Publisher/Subscriber Tutorial

## Введение
Узлы (nodes) — это исполняемые процессы, которые обмениваются данными через граф ROS. В этом учебном пособии узлы будут передавать информацию в виде строковых сообщений друг другу через топик. В качестве примера используется простая система «talker» и «listener»; один узел публикует данные, а другой подписывается на топик, чтобы получать эти данные.


## Предварительные условия
В предыдущих учебных пособиях вы узнали, как создать рабочее пространство и пакет.

## Задачи

### 1. Создание пакета
Откройте новый терминал и инициализируйте вашу установку ROS 2, чтобы команды `ros2` работали.

Перейдите в директорию `ros2_ws`, созданную в предыдущем учебном пособии.

Вспомните, что пакеты должны создаваться в директории `src`, а не в корне рабочего пространства. Перейдите в `ros2_ws/src` и выполните команду создания пакета:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```

Ваш терминал вернет сообщение, подтверждающее создание вашего пакета `cpp_pubsub` и всех необходимых файлов и папок.

Перейдите в `ros2_ws/src/cpp_pubsub/src`. Вспомните, что это директория в любом пакете CMake, где находятся исходные файлы, содержащие исполняемые файлы.

### 2. Написание узла publisher
Скачайте пример кода talker, выполнив следующую команду:

```bash
wget -O publisher_lambda_function.cpp https://raw.githubusercontent.com/ros2/examples/rolling/rclcpp/topics/minimal_publisher/lambda.cpp
```

Теперь появится новый файл с именем `publisher_lambda_function.cpp`. Откройте файл с помощью вашего любимого текстового редактора.

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

#### 2.1 Изучение кода
В начале кода включены стандартные заголовки C++, которые вы будете использовать. После стандартных заголовков C++ следует `rclcpp/rclcpp.hpp`, который позволяет использовать наиболее распространенные части системы ROS 2. Последний заголовок `std_msgs/msg/string.hpp` включает встроенный тип сообщения, который вы будете использовать для публикации данных.

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```

Эти строки представляют зависимости узла. Вспомните, что зависимости должны быть добавлены в `package.xml` и `CMakeLists.txt`, что вы сделаете в следующем разделе.

Следующая строка создает класс узла `MinimalPublisher`, наследуя от `rclcpp::Node`. Каждое `this` в коде ссылается на узел.

```cpp
class MinimalPublisher : public rclcpp::Node
```

Публичный конструктор называет узел `minimal_publisher` и инициализирует `count_` до 0. Внутри конструктора publisher инициализируется с типом сообщения String, именем топика `topic` и необходимым размером очереди для ограничения сообщений в случае задержки. Затем объявляется лямбда-функция `timer_callback`. Она выполняет захват по ссылке текущего объекта `this`, не принимает входных аргументов и возвращает `void`. Функция `timer_callback` создает новое сообщение типа String, устанавливает его данные с нужной строкой и публикует его. Макрос `RCLCPP_INFO` обеспечивает печать каждого опубликованного сообщения в консоль. Наконец, инициализируется `timer_`, что вызывает выполнение функции `timer_callback` два раза в секунду.

```cpp
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
```

В конце класса объявляются поля таймера, publisher и счетчика.

```cpp
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
```

После класса `MinimalPublisher` следует `main`, где узел фактически выполняется. `rclcpp::init` инициализирует ROS 2, а `rclcpp::spin` начинает обработку данных от узла, включая обратные вызовы от таймера.

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

#### 2.2 Добавление зависимостей
Перейдите на уровень выше в директорию `ros2_ws/src/cpp_pubsub`, где для вас были созданы файлы `CMakeLists.txt` и `package.xml`.

Откройте `package.xml` с помощью вашего текстового редактора.

Как упоминалось в предыдущем учебном пособии, убедитесь, что заполнены теги `<description>`, `<maintainer>` и `<license>`:

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache-2.0</license>
```

Добавьте новую строку после зависимости `ament_cmake` и вставьте следующие зависимости, соответствующие вашим операторам include:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

Это объявляет, что пакету требуются `rclcpp` и `std_msgs` при сборке и выполнении его кода.

Не забудьте сохранить файл.

#### 2.3 CMakeLists.txt
Теперь откройте файл `CMakeLists.txt`. Ниже существующей зависимости `find_package(ament_cmake REQUIRED)` добавьте строки:

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

После этого добавьте исполняемый файл и назовите его `talker`, чтобы вы могли запускать свой узел с помощью `ros2 run`:

```cmake
add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

Наконец, добавьте раздел `install(TARGETS...)`, чтобы `ros2 run` мог найти ваш исполняемый файл:

```cmake
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

Вы можете очистить ваш `CMakeLists.txt`, удалив некоторые ненужные разделы и комментарии, чтобы он выглядел так:

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

Вы могли бы собрать свой пакет сейчас, инициализировать локальные файлы настройки и запустить его, но давайте сначала создадим узел subscriber, чтобы вы могли увидеть работу всей системы.

### 3. Написание узла subscriber
Вернитесь в `ros2_ws/src/cpp_pubsub/src`, чтобы создать следующий узел. Введите следующий код в вашем терминале:

```bash
wget -O subscriber_lambda_function.cpp https://raw.githubusercontent.com/ros2/examples/rolling/rclcpp/topics/minimal_subscriber/lambda.cpp
```

Убедитесь, что эти файлы существуют:

```
publisher_lambda_function.cpp  subscriber_lambda_function.cpp
```

Откройте `subscriber_lambda_function.cpp` с помощью вашего текстового редактора.

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

#### 3.1 Изучение кода
Код узла subscriber практически идентичен коду publisher. Теперь узел называется `minimal_subscriber`, и конструктор использует функцию `create_subscription` узла для выполнения обратного вызова.

Таймера нет, потому что subscriber просто реагирует каждый раз, когда данные публикуются в топик `topic`.

Функция `topic_callback` получает данные строкового сообщения, опубликованные в топике, и просто записывает их в консоль с помощью макроса `RCLCPP_INFO`.

Вспомните из учебного пособия по топикам, что имя топика и тип сообщения, используемые publisher и subscriber, должны совпадать, чтобы они могли общаться.

```cpp
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }
```

Единственное объявление поля в этом классе — это подписка.

```cpp
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

Функция `main` абсолютно такая же, только теперь она запускает узел `MinimalSubscriber`. Для узла publisher вращение означало запуск таймера, но для subscriber это просто означает подготовку к получению сообщений по мере их поступления.

Поскольку этот узел имеет те же зависимости, что и узел publisher, в `package.xml` нечего добавлять.

#### 3.2 CMakeLists.txt
Повторно откройте `CMakeLists.txt` и добавьте исполняемый файл и цель для узла subscriber ниже записей publisher.

```cmake
add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

Не забудьте сохранить файл, и тогда ваша система pub/sub должна быть готова.

### 4. Сборка и запуск
Вероятно, у вас уже установлены пакеты `rclcpp` и `std_msgs` как часть вашей системы ROS 2. Хорошей практикой является запуск `rosdep` в корне вашего рабочего пространства (`ros2_ws`), чтобы проверить отсутствующие зависимости перед сборкой:

```bash
rosdep install -i --from-path src --rosdistro rolling -y
```

Все еще находясь в корне вашего рабочего пространства, `ros2_ws`, соберите ваш новый пакет:

```bash
colcon build --packages-select cpp_pubsub
```

Откройте новый терминал, перейдите в `ros2_ws` и инициализируйте файлы настройки:

```bash
. install/setup.bash
```

Теперь запустите узел talker:

```bash
ros2 run cpp_pubsub talker
```

Терминал должен начать публиковать информационные сообщения каждые 0.5 секунды, например:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

Откройте другой терминал, инициализируйте файлы настройки из `ros2_ws` еще раз, а затем запустите узел listener:

```bash
ros2 run cpp_pubsub listener
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

