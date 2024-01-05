from cmath import pi
from sympy import Symbol,matrices,cos,sin
from typing import Union
from sympy.matrices.dense import MutableDenseMatrix

_type = Union[Symbol,float]

# 得到各个关节的转移矩阵，默认使用标准dh参数
def get_matrix_from_dh(theta:_type,alpha:_type,a:_type,d:_type,typ:str='standard')->MutableDenseMatrix:
    c = cos(alpha)
    s = sin(alpha)
    if alpha == 0:
        c = 1
        s = 0
    elif abs(alpha - pi/2) < 1e-5:
        s = 1
        c = 0
    elif abs(alpha + pi/2) < 1e-5:
        s = -1
        c = 0
    if typ == "standard":# 标准dh
        return matrices.Matrix([[cos(theta),-sin(theta)*c,sin(theta)*s,a*cos(theta)],[sin(theta),cos(theta)*c,-cos(theta)*s,a*sin(theta)],[0,s,c,d],[0,0,0,1]])
    elif typ == "modified":# 改进型DH
        return matrices.Matrix([[cos(theta),-sin(theta),0,a],[sin(theta)*c,cos(theta)*c,-s,-s*d],[sin(theta)*s,cos(theta)*s,c,d*c],[0,0,0,1]])

# 得到各个关节的转移矩阵的逆矩阵，默认使用biaiozhundh参数
def get_inverse_matrix_from_dh(theta:_type,alpha:_type,a:_type,d:_type,typ:str='standard')->MutableDenseMatrix:
    c = cos(alpha)
    s = sin(alpha)
    if alpha == 0:
        c = 1
        s = 0
    elif abs(alpha - pi/2) < 1e-5:
        s = 1
        c = 0
    elif abs(alpha + pi/2) < 1e-5:
        s = -1
        c = 0
    if typ == "standard":# 标准dh
        A = matrices.Matrix([[cos(theta),-sin(theta)*c,sin(theta)*s],[sin(theta),cos(theta)*c,-cos(theta)*s],[0,s,c]])
        D = matrices.Matrix([a*cos(theta),a*sin(theta),d])
    elif typ == "modified":
        A = matrices.Matrix([[cos(theta),-sin(theta),0],[sin(theta)*c,cos(theta)*c,-s],[sin(theta)*s,cos(theta)*s,c]])
        D = matrices.Matrix([a,-s*d,d*c])
    A_T = A.transpose()
    B = -1 * A_T * D
    res = A_T.row_join(B).col_join(matrices.Matrix([0,0,0,1]).transpose())
    return res