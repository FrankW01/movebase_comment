
/* Eigen-decomposition for symmetric 3x3 real matrices.
   Public domain, copied from the public domain Java library JAMA. */

#ifndef _eig_h

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. *///将A矩阵特征值分解，特征向量是v矩阵的每列元素，d为对应的特征值，即Ax=kx
void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);

#endif
