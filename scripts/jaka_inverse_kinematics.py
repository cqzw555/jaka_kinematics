from cmath import pi
from sympy import symbols,matrices,pprint,trigsimp,solve,eye,tan,simplify,atan,atan2,sqrt,factor,collect,acos
from dh import get_inverse_matrix_from_dh,get_matrix_from_dh

theta1 = symbols("theta1");
theta2 = symbols("theta2");
theta3 = symbols("theta3");
theta4 = symbols("theta4");
theta5 = symbols("theta5");
theta6 = symbols("theta6");

nx = symbols("nx")
ny = symbols("ny")
nz = symbols("nz")

ox = symbols("ox")
oy = symbols("oy")
oz = symbols("oz")

px = symbols("px")
py = symbols("py")
pz = symbols("pz")

ax = symbols("ax")
ay = symbols("ay")
az = symbols("az")

a0 = 0
a1 = 0
a2 = symbols("a2")
a3 = symbols("a3")
a4 = 0
a5 = 0
a6 = 0

d1 = symbols("d1")
# d2 = symbols("d2")
d2 = 0
# d3 = symbols("d3")
d3 = 0
d4 = symbols("d4")
d5 = symbols("d5")
d6 = symbols("d6")

alpha0 = 0
alpha1 = pi/2
alpha2 = 0
alpha3 = 0
alpha4 = pi/2
alpha5 = -pi/2
alpha6 = 0

# ur的参数 作为测试用
# alpha0 = pi/2
# alpha1 = 0
# alpha2 = 0
# alpha3 = pi/2
# alpha4 = -pi/2
# alpha5 = 0

#默认的参数
# alpha1 = symbols("alpha1")
# alpha2 = symbols("alpha2")
# alpha3 = symbols("alpha3")
# alpha4 = symbols("alpha4")
# alpha5 = symbols("alpha5")
# alpha6 = symbols("alpha6")

T = matrices.Matrix([[nx,ox,ax,px],[ny,oy,ay,py],[nz,oz,az,pz],[0,0,0,1]])

print("改进dh法建模：")
T1 = get_matrix_from_dh(theta1,alpha0,a0,d1,"modified")
T1_inv = get_inverse_matrix_from_dh(theta1,alpha0,a0,d1,"modified")
T2 = get_matrix_from_dh(theta2,alpha1,a1,d2,"modified")
T2_inv = get_inverse_matrix_from_dh(theta2,alpha1,a1,d2,"modified")
T3 = get_matrix_from_dh(theta3,alpha2,a2,d3,"modified")
T3_inv = get_inverse_matrix_from_dh(theta3,alpha2,a2,d3,"modified")
T4 = get_matrix_from_dh(theta4,alpha3,a3,d4,"modified")
T4_inv = get_inverse_matrix_from_dh(theta4,alpha3,a3,d4,"modified")
T5 = get_matrix_from_dh(theta5,alpha4,a4,d5,"modified")
T5_inv = get_inverse_matrix_from_dh(theta5,alpha4,a4,d5,"modified")
T6 = get_matrix_from_dh(theta6,alpha5,a5,d6,"modified")
T6_inv = get_inverse_matrix_from_dh(theta6,alpha5,a5,d6,"modified")

# print("标准dh法建模：")
# T1 = get_matrix_from_dh(theta1,alpha1,a1,d1)
# T1_inv = get_inverse_matrix_from_dh(theta1,alpha1,a1,d1)
# T2 = get_matrix_from_dh(theta2,alpha2,a2,d2)
# T2_inv = get_inverse_matrix_from_dh(theta2,alpha2,a2,d2)
# T3 = get_matrix_from_dh(theta3,alpha3,a3,d3)
# T3_inv = get_inverse_matrix_from_dh(theta3,alpha3,a3,d3)
# T4 = get_matrix_from_dh(theta4,alpha4,a4,d4)
# T4_inv = get_inverse_matrix_from_dh(theta4,alpha4,a4,d4)
# T5 = get_matrix_from_dh(theta5,alpha5,a5,d5)
# T5_inv = get_inverse_matrix_from_dh(theta5,alpha5,a5,d5)
# T6 = get_matrix_from_dh(theta6,alpha6,a6,d6)
# T6_inv = get_inverse_matrix_from_dh(theta6,alpha6,a6,d6)

A = T1_inv*T*T6_inv
A = trigsimp(A)

T2345 = T2*T3*T4*T5
print("以下两个矩阵相等：")
pprint(trigsimp(T2345))
pprint(A)

print("得到求解公式的方程：")
print("求解theta1的方程:")
# 直接进行求解的示例
print(T2345[7] - A[7],"= 0")
result = solve(T2345[7] - A[7],[theta1])
print("theta1 的结果： ",result)
print("求解theta5的方程:")
print(T2345[5] - A[5],"= 0")
result = solve(T2345[5] - A[5],[theta5])
print("theta5  的结果：  ",result)

print("求解theta6的方程组:")
print(T2345[6] - A[6],"= 0")
print(T2345[4] - A[4],"= 0")
print("自动解不出来，手动计算吧")
# result = solve([T2345[6] - A[6],T2345[4] - A[4]],[theta6])
# print("theta6  的结果：  ",result)

print("知道 theta1,theta5,theta6之后,以下矩阵的值全部可以算出来")
T234 = T2*T3*T4
B = T1_inv*T*T6_inv*T5_inv
pprint(trigsimp(B))
print("上面这个矩阵和下面矩阵相等")
pprint(trigsimp(T234))
print("联立以下方程,可以解出theta2和theta3")
print(T234[3] ,"= ", B[3])
print(T234[11] ,"= ", B[11])
print("theta5和theta6的关系")
print(T234[10] ,"= ", B[10])
print(trigsimp(B[3]*B[3]+B[11]*B[11]))

