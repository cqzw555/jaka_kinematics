# jaka_kinematics

杰卡机械臂的正逆运动学代码

杰卡不给正逆运动学代码，自己就手写一份，花了两天时间推导公式，按照[ur_kinematics](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_kinematics)的代码进行修改

## 目录结构
- include/   头文件 
- scripts/   公式推导过程中进行符号计算的代码，方便自己求解
- src/       代码文件，主要有正逆运动学实现和简单测试的代码
- test/      测试使用的数据文件

## TODOLIST
- 正运动学测试
- 逆运动学测试
  
**PS:**
1. 目前只有jaka zu5的参数，因为手上就只有这个机械臂。
2. 文件中的dh 参数和jaka 示教器api给出来的，和 示教器app调试模式中给出来的不一样，应该是jaka微调过，dh参数做过校正。
3. 进入jaka app调试模式的方法：连续点击app左上角图标10次，输入密码 JAKAAMAZING ，注意全是大写。[详见mech-vison文档](https://docs.mech-mind.net/zh/robot-integration/latest/full-control/jaka-setup-instructions.html)
4. 这东西不涉及商业机密，花两天就写出来的，为啥不给。 
5. 如想用给的符号计算示例代码来辅助进行其他机械臂运动学求解，需要安装sympy，不要安装ubuntu官方源力自带的，版本太老，用pypi源里的
