---
date: 2025-02-12 22:11:38
---

## 检查 urdf/xacro 文件

```shell
xacro \
    /home/trantor/build_env/quadruped/install/robot_description/share/robot_description/config/xacro/robot.xacro \
        debug:=false \
        gazebo:=ignition \
        use_mock_hardware:=true \
        namespace:=robot_description
```

```shell
ros2 run urdfdom check_urdf $(ros2 pkg prefix robot_description)/share/robot_description/config/urdf/robot.urdf

check_urdf $(ros2 pkg prefix robot_description)/share/robot_description/config/urdf/robot.urdf
```

## `spawn_entity`

在通过 gazebo 进行仿真的时候通过 pkg:gazebo_ros 功能包中的 `spawn_entity` 方法加载指定的 urdf 模型。

其中 `spawn_entity` 的具体参数如下所示：

```bash
usage: spawn_entity.py [-h]
  (-file FILE_NAME | -topic TOPIC_NAME | -database ENTITY_NAME | -stdin)
  -entity ENTITY_NAME [-reference_frame REFERENCE_FRAME]
  [-gazebo_namespace GAZEBO_NAMESPACE]
  [-robot_namespace ROBOT_NAMESPACE] [-timeout TIMEOUT]
  [-unpause] [-wait ENTITY_NAME]
  [-spawn_service_timeout TIMEOUT] [-x X] [-y Y] [-z Z]
  [-R R] [-P P] [-Y Y] [-package_to_model] [-b]
```

实际使用中使用的参数如下：

```python
    # spawn entity in gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', LaunchConfiguration('robot_name'),
            '-reference_frame', 'world',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-unpause',
        ],
        output='screen'
    )
```

可以通过显示指定 `-z` 参数将机器人挂载到指定的高度，可以便于调试。
