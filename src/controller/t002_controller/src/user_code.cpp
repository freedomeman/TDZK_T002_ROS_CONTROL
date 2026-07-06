#include "t002_controller/user_code.hpp"

#include <cstdio>

namespace t002_controller
{

void test()
{
  // 在这里写你的工具函数
}

void debug_motor(double pos, double vel)
{
  static int counter = 0;
  if (++counter % 100 == 0) {
    printf("[DEBUG] pos=%+.4f rad  vel=%+.4f rad/s\n", pos, vel);
  }
}

}  // namespace t002_controller





// 我需要你把机器人的urdf文件替换掉
// 然后改一下jointconfig文件，不知道电机类型的关节就是dm
// 改一下controller文件，我需要能够在gazebo中仿真也可以在现实中控制
// 我需要把脖子的控制逻辑直接写在控制器里面，然后封装一个函数，入口是关节的角度和角度度
// 出口是关节的roll和pitch的位置和速度（注意在controller中我们写的也是roll和pitch但是我们不要注意这些转换好的也叫roll和pitch就好了
// 在封装一个函数入口是roll和pitch的力矩，出口是两个驱动关节的力矩（这里应该也需要roll和pitch电机反馈的角度，我的想法是上面一个函数已经读取过电机数据了我们农一个结构体可以让两个函数一起访问定义在。hpp文件
// 或者我们可以写一和init函数在ros_control初始化结束，各种地址都设置好了以后，这个函数负责通过关节的名称查找关节的地址并且返回出来，然后在.hpp文件中我们定义一些指针用于指向返回的地址后续我们直接访问就可以了
// 这里有两个方案需要做辨别，然后我还有一个底盘的运动学解算代码同样这些都写在user_code中以便调用和改动
// FK + Jacobian 从 Python → C++。对于这个部分可能非常麻烦，所以我还有另一个办法，就是写一个robot_control的一个节点，通过读取joint_states话题发布的数据然后都在这个节点中计算计算出各个关节的目标以后在通过command_interface话题发布回到controller控制器。这两种办法那个好