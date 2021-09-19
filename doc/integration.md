# Deriviation on IMU integration
## 1.
$$ \hat{a}_t = R_w^t(a_t+g^w)+b_{at}+n_{at}\\  \hat{\omega}_t = \omega_t+b_{gt}+n_{gt} $$
## 2.
In world frame, IMU kinetic equations can be written as follows:
$$
p_{b_{k+1}} = p_{b_k} + v_{b_k}\Delta{t} + \iint_{t\in[k,k+1]}(R_t^w(\hat{a}_t-b_{at})-g^w)dt^2 \\
v_{b_{k+1}} = v_{b_k} + \int_{t\in[k,k+1]}(R_t^w(\hat{a}_t-b_{at})-g^w)dt \\
q_{b_{k+1}} = q_{b_k} \otimes \int_{t\in[k,k+1]}\frac12q_t\otimes\begin{bmatrix}0\\\hat{\omega}-b_{gt}\end{bmatrix}dt := q_{b_k} \otimes \int_{t\in[k,k+1]}\frac12\Omega(\hat{\omega}-b_{gt})q_tdt
$$
In frame w.r.t. previous time,
$$
R_w^{b_k}p_{b_{k+1}} = R_w^{b_k}( p_{b_k} + v_{b_k}\Delta{t} - \frac12g^w\Delta{t}^2) + \iint_{t\in[k,k+1]}R_t^w(\hat{a}_t-b_{at})dt^2 := R_w^{b_k}( p_{b_k} + v_{b_k}\Delta{t} - \frac12g^w\Delta{t}^2) + \alpha_{b_{k+1}}^{b_k} \\
R_w^{b_k}v_{b_{k+1}} = R_w^{b_k}( v_{b_k} - g^w\Delta{t} ) + \int_{t\in[k,k+1]}R_t^w(\hat{a}_t-b_{at})dt := R_w^{b_k}( v_{b_k} - g^w\Delta{t} ) + \beta_{b_{k+1}}^{b_k} \\
q_{b_{k+1}} = q_{b_k} \otimes \int_{t\in[k,k+1]}\frac12\Omega(\hat{\omega}-b_{gt})q_tdt := q_{b_k} \otimes \gamma_{b_{k+1}}^{b_k}
$$
we get preintegration quantity $\alpha_{b_{k+1}}^{b_k}$, $\beta_{b_{k+1}}^{b_k}$ and $\gamma_{b_{k+1}}^{b_k}$. As we see preintegration quantity is only related with imu bias $b_{at}$ and $b_{gt}$.

