# IMU preintegration

[toc]

IMU is a sensor that measures a body's motion state, using a combination of accelerometers, gyroscopes, and sometimes magnetometers. In general, IMU reports angular velocity and acceleration with high frequency (higher than 400Hz).  
In IMU related integrated navigation problem, VIO for example, IMU can be used as the initial value of the consecutive frame matching. However, in optimization-based problem, integrating IMU measurement directly will bring complex computations. Forster proposed IMU preintegration witch integrates IMU data on body frame instead of world frame. This article will give preintegrations derivation outlines.
## Pre-knowledge
### Notations
* null
<!-- * $\hat{}$ means measurements -->

### IMU error model
$$
\hat{a}_t=a_t+b_{a_t}+R_w^tg^w+n_a\\
\hat{w}_t=w_t+b_{g_t}+n_w
$$
where
$$
n_a\sim(0,\sigma_a^2),n_a\sim(0,\sigma_w^2)\\
\dot{b}_{a_t}=n_{b_a}\sim N(0,\sigma_{b_a}^2), \dot{b}_{g_t}=n_{b_g}\sim N(0,\sigma_{b_g}^2)
$$

## Preintegration
### IMU kinematics
Integration on world frame:  
$$
p_{b_{k+1}}^w = p_{b_k}^w + v_{b_k}^w\Delta{t_k} + \iint_{t\in[t_k,t_{k+1}]}R_t^w(\hat{a}_t-b_{a_t}-g^w)dt^2\\
v_{b_{k+1}}^w = v_{b_k}^w + \int_{t\in[t_k,t_{k+1}]}R_t^w(\hat{a}_t-b_{a_t}-g^w)dt^2\\
q_{b_{k+1}}^w=q_{b_k}^w \otimes \int_{t\in[t_k,t_{k+1}]}\frac12\Omega(\hat{w}_t-b_{g_t})q_t^{b_k}dt
$$  
Integration on body frame:  
$$
R_w^{b_k}p_{b_{k+1}}^w = R_w^{b_k} (p^w_{b_k}+v^w_{b_k}\Delta{t_k}-\frac12g^w\Delta{t_k}^2) + \alpha^{b_k}_{b_{k+1}}\\
R_w^{b_k}v_{b_{k+1}}^w = R_w^{b_k} (v^w_{b_k}-g^w\Delta{t_k})+\beta^{b_k}_{b_{k+1}}\\
q_w^{b_k}q_{b_{k+1}}^w = \gamma^{b_k}_{b_{k+1}}
$$  
where $\alpha^{b_k}_{b_{k+1}},\beta^{b_k}_{b_{k+1}},\gamma^{b_k}_{b_{k+1}}$ only related to $R_t^w,b_{a_t},b_{g_t}$.

### Error-state kinematics
We introduce the error perturbation analysis as following:  
$$ \hat{x}=x+\delta{x} $$  
where $x$ is the **main-part/ideal/nominal** value of measurements and $\hat{x}$ is the **calculated/true/orbserved** value which contains normal errors $\delta x$.  
<!-- For the nominal kinematics, we have
$$
\dot{\alpha}=\beta \\
\dot{\beta}=R(a_m-b_a) \\
\dot{}
$$ -->
Let $x$ denotes $[\alpha,\beta,\gamma,b_a,b_g]$, finally we will get the jacobian and covariance of $\delta{x}^{b_k}_{b_{k+1}}$ w.r.t $\delta{x}^{b_k}_{b_{k}}$.  

Construct residuals from [IMU kinetmatics](#imu-kinematics) by minus equal sides, then you can update $x$ easily with high performance.  
