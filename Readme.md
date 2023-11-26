# 出现的问题

> * (-215:Assertion failed) (D.cols == ((flags & 2) == 0 ? B.cols : B.rows)) in function 'cvGEMM'  
    解决方法：
The problem seems to be in the matrix translation. You have the error because you're trying to multiply rotation (3x3) with translation (1x3).