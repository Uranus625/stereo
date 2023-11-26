> ## 出现的问题
> * ***(-215:Assertion failed) (D.cols == ((flags & 2) == 0 ? B.cols : B.rows)) in function 'cvGEMM'***  
> 原因：
The problem seems to be in the matrix translation. You have the error because you're trying to multiply rotation (3x3) with translation (1x3).  

> ## 注意:  
> * matlab导出的内参矩阵及旋转矩阵需要转置。  
> * 需要保证标定结果数据的正确及精度，否则图像会严重失真。