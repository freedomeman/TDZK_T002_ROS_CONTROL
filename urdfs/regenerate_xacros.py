#!/usr/bin/env python3
"""从源 URDF 重新生成真机 + Gazebo xacro"""
import re

with open('/home/tuf/桌面/总装配体urdf/urdf/总装配体urdf.urdf') as f:
    text = f.read()
text = text.replace('package://总装配体urdf/meshes/', 'package://t002_description/meshes/')
body = re.sub(r'<\?xml[^?]*\?>', '', text)
body = re.sub(r'<robot[^>]*>', '', body, count=1)
body = re.sub(r'</robot>\s*$', '', body)
body = body.replace('\r\n','\n').replace('\r','\n')

deploy = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="T002_Robot">
  <xacro:include filename="$(find t002_description)/xacro/ros2control.xacro"/>
  <xacro:property name="joints_config_file" value="$(find t002_description)/xacro/joints_config.yaml"/>
  <xacro:property name="imu_config_file" value="$(find t002_description)/xacro/imu_config.yaml"/>
  <xacro:t002_ros2_control name="T002Hardware" joints_config_file="${{joints_config_file}}" imu_config_file="${{imu_config_file}}"/>
{body}
</robot>'''

gazebo = f'''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="T002_Robot">
  <xacro:include filename="$(find t002_description)/xacro/ros2control.xacro"/>
  <xacro:property name="joints_config_file" value="$(find t002_description)/xacro/joints_config.yaml"/>
  <xacro:property name="imu_config_file" value="$(find t002_description)/xacro/imu_config.yaml"/>
  <xacro:t002_ros2_control name="T002Hardware" hardware_plugin="gz_ros2_control/GazeboSimSystem" joints_config_file="${{joints_config_file}}" imu_config_file="${{imu_config_file}}"/>
{body}
  <gazebo>
    <plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(arg controller_config_file)</parameters>
      <controller_manager_name>controller_manager</controller_manager_name>
    </plugin>
  </gazebo>
</robot>'''

from pathlib import Path
out = Path(__file__).parent / 'T002_description/xacro'
(out / 'T002_description.urdf.xacro').write_text(deploy)
(out / 'T002_gazebo_des.urdf.xacro').write_text(gazebo)
print('Done')
