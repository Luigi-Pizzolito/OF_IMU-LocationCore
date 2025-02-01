# EKF Planning

## 1st Order IIR LP Filtering

*(done for all sensor measurements to remove high-frequency noise)*

$$
y_n = \alpha{\cdot}y_{n-1}+(1-\alpha){\cdot}x_{n} \\
f_c = \frac{1}{2\pi{\cdot}\alpha{\cdot}T_s}
$$

*(a good value for $\alpha$ is $0.01 \lessapprox \alpha \lessapprox 0.1$, with lower $\alpha$ for higher $f$ measurement)*

## Variables
State:
$$
\underline{\hat{x}}_n=
\begin{bmatrix}
x\\
y\\
z\\
v_x\\
v_y\\
v_z\\
\end{bmatrix}
$$
*(estimated values of the system's state, at a given time.)*

Error Covariance:

*(uncertainty in the current state prediction)*
$$
P = 
\begin{bmatrix}
\sigma_{x}^{2} & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{y}^{2} & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma_{z}^{2} & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma_{v_x}^{2} & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma_{v_y}^{2} & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{v_z}^{2} \\
\end{bmatrix}
\Rightarrow
\begin{bmatrix}
\sigma_{x}^{2}\\
\sigma_{y}^{2}\\
\sigma_{z}^{2}\\
\sigma_{v_x}^{2}\\
\sigma_{v_y}^{2}\\
\sigma_{v_z}^{2}\\
\end{bmatrix}
$$
*(assuming that there is no noise-correlation, all diagonals becomes a 1x6 vector)*


Process Noise Covariance:

*(uncertainty in process model, aka. due to IMU noise)*
$$
Q =
\begin{bmatrix}
\sigma_{a_x}^{2} & 0 & 0 \\
0 & \sigma_{a_y}^{2} & 0 \\
0 & 0 & \sigma_{a_z}^{2} \\
\end{bmatrix}
\Rightarrow
\begin{bmatrix}
\sigma_{a_x}^{2} \\
\sigma_{a_y}^{2} \\
\sigma_{a_z}^{2} \\
\end{bmatrix}
$$
*(assuming that there is no noise-correlation, all diagonals becomes a 1x3 vector)*


Measurement Noise Covariance:

*(uncertainty in measurements, aka. optical flow noise)*
$$
R =
\begin{bmatrix}
\sigma_{{\Delta}_x}^{2} & 0 & 0 \\
0 & \sigma_{{\Delta}_y}^{2} & 0 \\
0 & 0 & \sigma_{{\Delta}_z}^{2} \\
\end{bmatrix}
\Rightarrow
\begin{bmatrix}
\sigma_{{\Delta}_x}^{2} \\
\sigma_{{\Delta}_y}^{2} \\
\sigma_{{\Delta}_z}^{2} \\
\end{bmatrix}
$$
*(assuming that there is no noise-correlation, all diagonals becomes a 1x3 vector)*




## Functions





## Steps

1. **Initialize** $\underline{\hat{x}}_0, P, Q, R$ and $T$.

    1.1. $\underline{\hat{x}}_0 = \begin{bmatrix}x&y&z&v_x&v_y&v_z\end{bmatrix}^T = \begin{bmatrix}0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}^T$

    2.2. $P = \begin{bmatrix}\sigma_{x}^{2}&\sigma_{y}^{2}&\sigma_{z}^{2}&\sigma_{v_x}^{2}&\sigma_{v_y}^{2}&\sigma_{v_z}^{2}\end{bmatrix}^T = \begin{bmatrix}? & ? & ? & ? & ? & ? \end{bmatrix}^T$

    2.3. $Q = \begin{bmatrix}\sigma_{a_x}^{2}&\sigma_{a_y}^{2}&\sigma_{a_z}^{2} \end{bmatrix}^T = \begin{bmatrix}? & ? & ? \end{bmatrix}^T$

    2.4. $R = \begin{bmatrix}\sigma_{{\Delta}_x}^{2}&\sigma_{{\Delta}_y}^{2}&\sigma_{{\Delta}_z}^{2}\end{bmatrix}^T = \begin{bmatrix}? & ? & ? \end{bmatrix}^T$

2. **Predict** from IMU lin. accel. at 100Hz, $\Delta_t=0.01\text{s}$.

    > Inputs: 
    > - Body-frame linear acceleration $\textbf{a}_{\textbf{imu}}$ (gravity-compensated)
    > - Orientation quaternion $\textbf{q}$

    2.1. Rotate acceleration to global frame
    $$\textbf{a}_{\textbf{g}}=R(\textbf{q})\cdot\textbf{a}_{\textbf{imu}} = \underline{u} $$

    2.2. Update the current state prediction
    $$
    \underline{\hat{x}}_{n+1} = \underline{\hat{x}}_{n} + \Delta_t{\cdot}f({\underline{\hat{x}}_{n}},{\underline{u}},{\Delta_t}) \\
    \because f = \begin{bmatrix}
        v_x + \frac{1}{2}\underline{u}_x\Delta_t \\
        v_y + \frac{1}{2}\underline{u}_y\Delta_t \\
        v_z + \frac{1}{2}\underline{u}_z\Delta_t \\
        \underline{u}_x \\
        \underline{u}_y \\
        \underline{u}_z \\
    \end{bmatrix} \\
    \therefore \underline{\hat{x}}_{n+1} = \begin{bmatrix}
        x + v_x\cdot{\Delta_t} + \frac{1}{2}\underline{u}_x\cdot\Delta_t^{2} \\
        y + v_y\cdot{\Delta_t} + \frac{1}{2}\underline{u}_y\cdot\Delta_t^{2} \\
        z + v_z\cdot{\Delta_t} + \frac{1}{2}\underline{u}_z\cdot\Delta_t^{2} \\
        v_x + \underline{u}_x \\
        v_y + \underline{u}_y \\
        v_z + \underline{u}_z \\
    \end{bmatrix} \\
    $$

    *(updating based on Euler integration of acceleration, aka. dead-reckoning)*

    2.3. Update the error covariance
    $$
    P_{n+1} = P_{n} + \Delta_t\cdot(A{\cdot}P_{n}+P_{n}{\cdot}A^{T}+Q)
    $$

    *(where $A$ is the ...)*

    $$
    A = 
    $$

3. **Update** from OF disp. at 10Hz, $\Delta_t=0.1\text{s}$.

    > Inputs: 
    > - OF 2D delta-position $\Delta_{x_{OF}}, \Delta_{y_{OF}}$
    > - Position estimate 10 states ago $\underline{\hat{x}}_{n-9}$
    > - Orientation quaternion $\textbf{q}$

    3.1. Compute the Kalman gain
    $$
    K = P{\cdot}C^{T}{\cdot}(C{\cdot}P{\cdot}C^{T}+R)^{-1}
    $$

    *(where $C$ is the ...)*

    3.2. Update the current state
    $$
    \underline{\hat{x}}_{n+1} = \underline{\hat{x}}_{n} + K{\cdot}(\underline{y}-h({\underline{\hat{x}}_{n}},{\underline{\hat{x}}_{n-9}}))
    $$

    *(where $\underline{y}$ is the current position delta measurement (from OF), and $h$ is the prediction of the current position delta (from $\underline{\hat{x}}_{n}$, $\underline{\hat{x}}_{n-9}$); where $(\underline{y}-h)$ forms the innovation.)*

    $$
    \underline{y} = R(\textbf{q}){\cdot}\begin{bmatrix} \Delta_{x_{OF}} \\ \Delta_{y_{OF}} \\ 0 \end{bmatrix} = \begin{bmatrix} \Delta_{x_{OF}} \\ \Delta_{y_{OF}} \\ \Delta_{z_{OF}} \end{bmatrix}
    $$

    *(the optical flow measurements are projected into 3D using the current orientation (camera frame to global frame))*

    $$
    h({\underline{\hat{x}}_{n}},{\underline{\hat{x}}_{n-9}}) = ({\underline{\hat{x}}_{n}}-{\underline{\hat{x}}_{n-9}}){\cdot}\begin{bmatrix}I_3 \\ 0_{3{\times}3}\end{bmatrix} = \begin{bmatrix}x_n \\ y_n \\ z_n \end{bmatrix} - \begin{bmatrix}x_{n-9} \\ y_{n-9} \\ z_{n-9} \end{bmatrix} = \begin{bmatrix}\Delta_{x_{p}} \\ \Delta_{y_{p}} \\ \Delta_{z_{p}} \end{bmatrix}
    $$

    *(this delta prediction is taken by keeping a buffer of the last 10 position values, so that we can easily take a delta from the states' history)*

    *(here we need to check the dimensions of $K$ and be sure that in this step, we only update the position in the state ${\underline{\hat{x}}_{n+1}}$ and not the velocities; since OF gives us calculated average velocity, but not instantaneous velocity like the IMU, which is the only thing that makes sense to put into the current state vector)*

    3.3. Update the error covariance
    $$
    P_{n+1} = (I - K{\cdot}C){\cdot}P_{n}
    $$

    *(where $I$ is the ...)*

    *(and $C$ is the ..., as computed in the predict step)*
