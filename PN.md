目标就是：**你没有目标距离、没有目标速度、没有目标位置，也能先把一个近似 PN 跑起来。**

------

# 1. 输入量

你至少需要这些量：

- 图像检测到的目标像点：$(u,v)$
- 相机内参：$(f_x,f_y,c_x,c_y)$
- 云台/导引头到机体的安装矩阵：$C_c^b$
- 机体到世界系/NED 的姿态矩阵：$C_b^n$
- 载机在世界系的速度：$\mathbf v_m^n$
- 控制周期：$\Delta t$

------

# 2. 从图像点重建 LOS 单位向量

先在相机系下做归一化成像平面坐标：
$$
x=\frac{u-c_x}{f_x},\qquad y=\frac{v-c_y}{f_y}
$$
相机系 LOS 单位向量：
$$
\mathbf u_c
=
\frac{1}{\sqrt{x^2+y^2+1}}
\begin{bmatrix}
x\\
y\\
1
\end{bmatrix}
$$
再变到世界系/NED：
$$
\mathbf u^n = C_b^n C_c^b \mathbf u_c
$$
这就是你后面要用的 LOS 单位向量。

------

# 3. 估计 LOS 角速度

离散差分：
$$
\dot{\mathbf u}^n \approx \frac{\mathbf u_k^n-\mathbf u_{k-1}^n}{\Delta t}
$$
然后：
$$
\boldsymbol{\omega}_{LOS}^n = \mathbf u^n \times \dot{\mathbf u}^n
$$
这是最小实现。

为了减小噪声，实际建议先对 $\mathbf u$ 做低通滤波，再差分。

例如：
$$
\mathbf u_f(k)=\alpha \mathbf u_f(k-1)+(1-\alpha)\mathbf u(k)
$$
然后用 $\mathbf u_f$ 代替 $\mathbf u$。

------

# 4. 构造“伪闭合速度” $\hat V_c$

因为你没有测距，所以没法严格算真实 $V_c$。
 最简单实用的近似是：
$$
\hat V_c = \max\left(V_{min},\ \mathbf v_m^n \cdot \mathbf u^n\right)
$$
这里：

- $\mathbf v_m^n \cdot \mathbf u^n$：载机速度在 LOS 方向上的投影
- $V_{min}$：给个最小值，避免几何不利时 PN 太弱

这个 $\hat V_c$ 不是严格物理意义上的闭合速度，但足够作为近似 PN 的增益尺度。

------

# 5. 最小可用 PN 项

$$
\mathbf a_{PN}^n
=
N \hat V_c
\left(
\boldsymbol{\omega}_{LOS}^n \times \mathbf u^n
\right)
$$

其中：

- $N$：导航比，先从 2 到 4 开始调

这就是最核心的近似 PN 项。

------

# 6. 推荐加一个沿 LOS 推进项

因为 PN 项本质主要是法向机动，不直接负责径向推进，所以建议加：
$$
\mathbf a_{adv}^n = k_a \mathbf u^n
$$
这样可以明确增加“朝目标方向冲”的趋势。

------

# 7. 推荐再加一个速度阻尼项

定义一个参考速度方向仍沿 LOS：
$$
\mathbf v_{ref}^n = v_{cmd}\mathbf u^n
$$
再加阻尼：
$$
\mathbf a_{damp}^n = k_v(\mathbf v_{ref}^n-\mathbf v_m^n)
$$
这个项的作用是：

- 避免速度乱飞
- 改善可实现性
- 提高收敛性

------

# 8. 最终最小可用控制律

我最推荐你起步直接用这个：
$$
\mathbf a_{cmd}^n
=
N \hat V_c
\left(
\boldsymbol{\omega}_{LOS}^n \times \mathbf u^n
\right)
+
k_a \mathbf u^n
+
k_v\left(v_{cmd}\mathbf u^n-\mathbf v_m^n\right)
$$
这就是一版**只依赖图像角度 + 自身速度**的最小可用近似 PN。