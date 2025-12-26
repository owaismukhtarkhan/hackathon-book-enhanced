---
title: Python-based ROS Packages
sidebar_label: Python-based ROS Packages
---

# Python-based ROS Packages

<div className="neon-border" style={{ padding: '20px', borderRadius: '8px', backgroundColor: 'var(--ifm-color-emphasis-100)' }}>

## Introduction to ROS Packages

A ROS package is the fundamental unit of organization in ROS. It contains:

</div>

- **Source code** (nodes, libraries, etc.)
- **Configuration files** (launch files, parameters)
- **Resource files** (meshes, URDF models, etc.)
- **Package manifest** (package.xml)
- **Build configuration** (CMakeLists.txt or setup.py)

## Creating a Python Package

### Package Structure

```
my_robot_package/
├── package.xml          # Package manifest
├── setup.py             # Python setup configuration
├── setup.cfg            # Installation configuration
├── resource/            # Resource files
├── launch/              # Launch files
├── config/              # Configuration files
├── test/                # Test files
└── my_robot_package/    # Python source code
    ├── __init__.py
    ├── publisher_member_function.py
    └── subscriber_member_function.py
```

### Package Manifest (package.xml)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Examples of minimal publisher/subscriber using rclpy</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Setup Configuration (setup.py)

```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_robot_package']),
        ('share/my_robot_package', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_package.publisher_member_function:main',
            'listener = my_robot_package.subscriber_member_function:main',
        ],
    },
)
```

## Creating a Publisher Node

```python
# publisher_member_function.py
import rclpy
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
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

```python
# subscriber_member_function.py
import rclpy
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
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes at once:

```python
# launch/talker_listener.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='talker',
            name='talker',
        ),
        Node(
            package='my_robot_package',
            executable='listener',
            name='listener',
        )
    ])
```

## Building and Running

### Building the Package

```bash
# From the workspace root
colcon build --packages-select my_robot_package
```

### Sourcing the Environment

```bash
source install/setup.bash
```

### Running Nodes

```bash
# Run the publisher
ros2 run my_robot_package talker

# In another terminal, run the subscriber
ros2 run my_robot_package listener
```

### Using Launch Files

```bash
ros2 launch my_robot_package talker_listener.launch.py
```

## Best Practices for Python ROS Packages

1. **Structure**: Follow the standard ROS package structure
2. **Naming**: Use snake_case for package names and file names
3. **Documentation**: Include proper docstrings and comments
4. **Testing**: Write unit tests for your nodes
5. **Error Handling**: Implement proper exception handling
6. **Logging**: Use ROS logging facilities appropriately
7. **Parameters**: Use ROS parameters for configurable values

## Learning Objectives

By the end of this module, you will be able to:
- Create properly structured Python-based ROS 2 packages
- Implement publisher and subscriber nodes in Python
- Configure package manifests and setup files
- Use launch files to manage multiple nodes
- Follow ROS 2 best practices for Python development

## What's Next

Now that you understand the fundamentals of ROS 2 and how to create Python packages, you'll learn about robot simulation environments using Gazebo in the next module.