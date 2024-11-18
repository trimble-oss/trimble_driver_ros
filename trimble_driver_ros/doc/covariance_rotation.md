# Covariance Rotation Math
As part of the conversion from the FRD, NED frame to the FLU, ENU frame, the covariance matrix also need updating.

The covariance is defined as
```math
C = \mathbb{E}(XX^T) - \mathbb{E}(X) \mathbb{E}(X^T)
```
During the conversion between frames, the original values are rotated on the left and right.
```math
R_{FLU,ENU} = R_{FLU,FRD} R_{FRD,NED} R_{NED,ENU}
```
where `R_{FRD,NED}` are the values measured from the system for RPY and the frame used to report covariance. 
```math
R_{FLU, FRD} = \begin{bmatrix}1 & 0 & 0\\0 & -1 & 0 \\ 0 & 0 & -1\\ \end{bmatrix}, 
R_{NED, ENU} = \begin{bmatrix}0 & 1 & 0\\1 & 0 & 0 \\ 0 & 0 & -1\\ \end{bmatrix}
```

To substitute in the original covariance equation:
```math
X' = R_1 X R_2
```
```math
R_1 = R_{FLU, FRD}
```
```math
R_2 = R_{NED, ENU}
```

Substituting the original matrix with the rotated one returns the following result: 
```math
C' = \mathbb{E}(R_1XR_2R_2^TX^TR_1^T) - \mathbb{E}(R_1XR_2) \mathbb{E}(R_2^TX^TR_1^T)
```
```math
C' = R_1\mathbb{E}(XR_2R_2^TX^T)R_1^T - R_1\mathbb{E}(X)R_2 R_2^T\mathbb{E}(X^T)R_1^T
```
```math
C' = R_1(\mathbb{E}(XR_2R_2^TX^T) -\mathbb{E}(X)R_2 R_2^T\mathbb{E}(X^T))R_1^T
```

Here we can take advantage of the fact that `R_2` can be cancelled out due to:
```math
I = R_2R_2^T
```

This allows the covariance matrix to take the form of:
```math
C' = R_1(\mathbb{E}(XX^T) -\mathbb{E}(X)\mathbb{E}(X^T))R_1^T
```
```math
C' =  R_1 C R_1^T
```

Lastly substituting in the matrix `R_1` and taking advantage of the fact `C` is a diagonal matrix:
```math
C' =  \begin{bmatrix}1 & 0 & 0\\0 & -1 & 0 \\ 0 & 0 & -1\\ \end{bmatrix} \begin{bmatrix} c_1 & 0 & 0\\0 & c_2 & 0 \\ 0 & 0 & c_3\end{bmatrix} \begin{bmatrix}1 & 0 & 0\\0 & -1 & 0 \\ 0 & 0 & -1\\ \end{bmatrix}
```

Finally:
```math
C' =  C
```