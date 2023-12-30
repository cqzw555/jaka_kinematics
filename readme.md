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

    正向运动学已经测试完成，测试结果如下：

    x:
    mean:0.0000 stdev:0.0000 min:0.0000 max:0.0001
    
    y:
    mean:0.0000 stdev:0.0000 min:0.0000 max:0.0001
    
    z:
    mean:0.0000 stdev:0.0000 min:0.0000 max:0.0001
    
    rx:
    mean:0.0001 stdev:0.0001 min:0.0000 max:0.0037
    
    ry:
    mean:0.0000 stdev:0.0000 min:0.0000 max:0.0002
    
    rz:
    mean:0.0001 stdev:0.0001 min:0.0000 max:0.0037

- 逆运动学测试

    正向运动学已经测试完成，测试结果如下：
    - 有7 / 10087个逆解失败
    - 有29 / 10087个没有找到正确结果,角度误差0.01弧度以上

- jaka api计算结果对比

    逆运动学计算结果角度误差2e-6左右


## 奇异点分析
- 在求解逆运动学的时候，解第一个关节J1的时候，$sqrt(pow(d6 * ay - py,2)+pow(-(d6 * ax - px),2)-pow(d4,2))$会有虚数的结果，即$pow(d6 * ay - py,2)+pow(-(d6 * ax - px),2)-pow(d4,2) < 0$

- 在求解关节6的时候，需要联立两个方程 $(-nx*s1 + ny*c1)*s6 + (-ox*s1 + oy*c1)*c6 = 0$ 和 $(-nx*s1 + ny*c1)*c6 - (-ox*s1 + oy*c1)*s6 = -s5$，这两个方程同时成立，就要求s5 != 0,不然求解出来s6=c6=0，解不出来对应的关节6，因此关节5不能是0,pi和-pi

- 在求关节3的时候，会求acos，求要求 $fabs((p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)) <= 1$ ==> $p_{13x}^2 +p_{13y}^2  <= (a_2+a_3)^2$ 或 $p_{13x}^2 +p_{13y}^2  <= (a_2 - a_3)^2$，这个没有分析出任何东西

- 在求关节2的时候，也是同时联立两个方程这就要求 $(a2 + a3 * c3)^2+(a3 * s3)^2 \neq 0$，只不过我程序里面省略掉了。带入关节3的公式中，就得到 $p_{13x}^2 +p_{13y}^2 \neq 0$

分析失败了，只求出一个奇异点，另外还有z轴线z2,z3,z4共面，工具末端位置在轴线 Z1 和 Z2 构成的平面上，分析不出来

**PS:**
1. 目前只有jaka zu5的参数，因为手上就只有这个机械臂。
2. 文件中的dh 参数和jaka 示教器api给出来的，和 示教器app调试模式中给出来的不一样，应该是jaka微调过，dh参数做过校正。
3. 进入jaka app调试模式的方法：连续点击app左上角图标10次，输入密码 JAKAAMAZING ，注意全是大写。[详见mech-vison文档](https://docs.mech-mind.net/zh/robot-integration/latest/full-control/jaka-setup-instructions.html)
4. 这东西不涉及商业机密，花两天就写出来的，为啥不给。 
5. 如想用给的符号计算示例代码来辅助进行其他机械臂运动学求解，需要安装sympy，不要安装ubuntu官方源力自带的，版本太老，用pypi源里的


