# Quaternion kinematics
## Rotation ambiguity: alias and abili
The coordinates of a point P may change due to either a rotation of the coordinate system CS (alias), or a rotation of the point P (alibi). Alibi and alias transformations are also known as active and passive transformations, respectively (from [wikipedia](https://en.wikipedia.org/wiki/Rotation_matrix#Ambiguities)).  
**Eigen use alias** while MATLAB use alibi.  
Now for a point $p$, which has coordinate $p^w$ in world frame and coordinate $p^b$ in world frame, will satisfy the equation:  
$$p^w=T^w_bp^b=R^w_bp^b+t^w_b$$  

## Quaternion
### Multiplication presentation
$$ q_a\otimes q_b = L(q_a)q_b=R(q_b)q_a $$
### Derivation
#### Jacobian dq w.r.t $t$
$$ \dot{q_t}=\mathop{lim}\limits_{t\rightarrow0} \frac{q\otimes dq}{dt}
= q\otimes    \begin{bmatrix} w/2\\1 \end{bmatrix}
$$  
See [euler-to-quaternion](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion) for reference.

#### Jacobian w.r.t $\delta\theta$ at $\delta\theta\rightarrow0$
$$ \frac{\partial R exp(\delta\theta^\wedge)p}{\partial\delta\theta}
= \mathop{lim}\limits_{\delta\theta\rightarrow0} \frac{R\delta\theta^\wedge p}{\delta\theta}
= -R\delta\theta^\wedge
$$  
