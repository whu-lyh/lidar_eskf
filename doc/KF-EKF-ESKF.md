[TOC]

# ❤写在前面的话❤

+ 写本文的目的是想自己整理一个对我来说比较易懂的思路，来正确理解-推导KF以及其升级版的原理和公式，在去掉一些网上过长的前导知识铺垫的同时增加对简化的公式罗列的补充。
+ 虽然之前也学过Kalman Filter，但是年代就远了之后很多公式就忘记了，也许当时总结的不够到位，理解的不够深刻。现在重新总结一下。
+ 在测绘背景下，从状态估计的角度分析KF，这里引用高博知乎的回答：

> *任何传感器，激光也好，视觉也好，整个SLAM系统也好，要解决的问题只有一个：**如何通过数据来估计自身状态。**每种传感器的测量模型不一样，它们的精度也不一样。换句话说，状态估计问题，也就是“**如何最好地使用传感器数据**”。可以说，SLAM是状态估计的一个特例。*

+ 这里介绍的所有基于滤波的方法都是状态估计中的一种。任何一个需要估计的系统状态都可以写成[运动方程](https://www.zhihu.com/search?q=运动方程&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A1267117107})和测量方程。其中**运动方程**即系统前后状态之间的转换关系，而**测量方程**是系统在某个状态下传感器的观测值。通常来说，运动方程和测量方程在实际过程中都会有**噪声**，因此如何能够从噪声这么多的观测数据中估算出正确的状态是个难事儿，毕竟世界很复杂。（这里的状态指得是位姿，并不是控制等领域中的那种状态）
+ TODO
+ [x] 问题描述中的举例子
+ [ ] KF中另外两种角度解释
+ [ ] EKF
+ [ ] ESKF

# Kalman Filter

 		如果将状态变量看作是一个服从[高斯分布](https://www.zhihu.com/search?q=高斯分布&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A1267117107})的随机变量。就可以用基于KF的方法来完成状态估计这个工作。在**卡尔曼滤波器（KF）**的通用算法中，主要分为两个部分：预测和更新。其中**预测**过程是说整个状态随时间变化，依赖前一次的**状态**与**协方差**，根据**运动模型**预测下一次的**状态**和**协方差**（[只依赖两个状态的原因](#filter & smoothing & 图优化)）；**更新**过程则是根据当前测量和预测的状态和协方差根据**测量方程**校正本次的**状态**和**协方差**。两者迭代往前走，从而完成系统整个运动过程中的状态估计。

​        下面这个图很好的说明了KF的流程，即状态在不断迭代预测-更新的过程。

![whole_pipeline_of_kf](E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\Basic_concept_of_Kalman_filtering.svg)

<center style="color:#FF0000">图1.Kalman Filter流程</center>

### 问题描述

​		这里使用KF是为了解决状态估计问题，那么这里将状态估计问题描述为：

​		机器人（或某运动载体）在各个时刻的状态为$x_1,...,x_i,...,x_k$,$x_k \in \mathbb{R}^N$,k为离散时间下标,最大值为K,状态向量为一个N维的向量，那么按照状态估计的基本条件(有初值，运动方程，观测方程)，其对应的运动方程和观测方程可以**抽象**地用下面式子来表示：
$$
\begin{align}
\label{motion_function}
x_k &= f(x_{k-1},u_k,w_k) \\
\label{observation_function}
y_k &= g(x_k,n_k)
\end{align}
$$

+ $f(\cdot)$为**运动方程**，表示从前一刻状态$x_{k-1}$到当前时刻状态$x_k$之间的变换，描述两个状态之间是如何转换的，因此会涉及到状态转移方程（状态转移矩阵）
+ $u_k \in \mathbb{R}^N$为当前时刻的输入（可以理解为运动向量这一中间态，运动向量和运动状态向量之间需要控制矩阵来转换，不是最终要估计的状态向量）
+ $w_k \in \mathbb{R}^N$为输入数据的噪声
+ $g(\cdot)$为**观测方程**，表示当前状态$x_{k}$与此时刻系统传感器观测的数据$y_{k} \in \mathbb{R}^M$之间的转换，描述当前状态$x_{k}$与传感器观测到的$y_{k}$之间的关系，即观测方程（观测矩阵）是如何构建的
+ $n_k \in \mathbb{R}^M$为系统传感器的观测噪声，该误差由硬件决定,维度为M，不一定和状态向量的维度相同
+ 除了$u_k$，其他变量都是随机变量，噪声和初始状态一般假设是无关的，且各个时刻也都互不相关

​       带入具象化的栗子再描述一遍状态估计问题，方便理解其概念：

1. 当状态只关心位置时（或着说只能观测到坐标），状态$x$表示的就是坐标，一维空间：x，二维空间：xy，三维空间：xyz，没有再高维了。

   

   + 对应的运动方程可能是如下表示，$\Delta x_{k}$就是运动向量，它是相邻两个状态中间过渡的量，

   + $$
     \begin{align}
     x_{k}=x_{k-1}+\Delta x_{k}+w_{k}
     \end{align}
     $$
     
   + 对应的观测方程可能是如下，e.g.可能是一个LiDAR,观测到的量时角度$\theta$和距离r,那么$y_k$就是这里的$\theta$和r。$L_k$是它观测到的对象的位置，不是他的观测量。

   + $$
     \begin{align}
     \left[ \begin{array}{l} r\\ \theta \end{array} \right]_k = 
     \left[ \begin{array}{l} \sqrt {{{\left\| {{x_k} - {L_k}} \right\|}_2}} \\
     {\tan ^{ - 1}}\frac{{{L_{k,y}} - {x_{k,y}}}}{{{L_{k,x}} - {x_{k,x}}}}
     \end{array} \right] + n_k, \quad where \quad L_k=[L_{k,x},L_{k,y}]
     \end{align}
     $$

2. 如果系统状态更丰富一些，$x$表示的就除了位置xyz还有可能有速度$v$，加速度$a$，角速度$\omega$，重力加速度$g$等. <a href="#imu">here</a> 。因此传感器除了LiDAR还要有能测量旋转的[IMU](#IMU✅)。

   + 对应的运动方程可能是：用$\ref{const_velocity}$表示匀速运动，或者用$\ref{accel_velocity}$表示一般的动力学方程，符合牛顿第二定律。注意位置的一阶导数和二阶导数分别为速度和加速度$\dot{x}=v,\ddot{x}=a$。$u_k$就是速度和加速度。

   + $$
     \begin{align}
     \label{const_velocity}
     x_k &= x_{k-1}+v \cdot \Delta t \\
     \label{accel_velocity}
     x_k &= x_{k-1}+v \cdot \Delta t + \frac{1}{2} \cdot a \cdot \Delta t ^2
     \end{align}
     $$
     
   + 对应的观测方程也可能是如下表示，如果是相机，就是像素的位置或者逆深度，如果是激光雷达，就还和上述的一样，观测方程和对应传感器对应。
   
   + $$
     \begin{align}
     p(u,v)=\frac{1}{Z_w}K(R \cdot P_w + t),三维投影到二维的成像方程
     \end{align}
     $$

3. 不同的系统会有不同的运动模式，也会搭载不同的传感器，因此运动方程和观测方程会随之改动，因此原理推导时还得从抽象的概念开始。

​		这就是最一般的状态估计问题。根据$f,g$是否线性，分为**线性/非线性系统**。同时，对于噪声$\omega,n$，根据它们是否服从高斯分布，分为**高斯/非高斯噪声**系统。最一般的，也是最困难的问题，是非线性-非高斯(NLNG, Nonlinear-Non Gaussian)的状态估计。因此KF-EKF-PF分别对应线性高斯系统，非线性高斯系统和非线性非高斯系统。

### 线性高斯系统

​		因为**高斯分布经过线性变换之后仍为高斯分布**。而对于一个高斯分布，只要计算出它的一阶(期望)和二阶矩(平方的期望)，就可以描述它(高斯分布只有两个参数$\mu,\Sigma=\delta^2$)，假设线性系统形式如下：
$$
\begin{align}
{x_k} &= {A_{k-1}}{x_{k-1}} + {v_k} + {w_k},  k>1,A_k \in \mathbb{R}^{N \times N}为状态转移矩阵\\
{y_k} &= {C_k}{x_k} + {n_k}，k>0,C_k \in \mathbb{R}^{M \times N}为观测矩阵\\
{w_k} &\sim N ({0,{Q_k}})，{Q_k}为运动模型噪声的协方差矩阵\\
{n_k} &\sim N(0,{R_k})，{R_k}为观测模型噪声的协方差矩阵\\
\end{align}
$$

+ 动力学模型是线性的，观测模型也是线性的（矩阵线性变换），状态噪声和观测噪声均为零均值的白噪声，这两种噪声，以及噪声与状态之间均互不相关。
+ 必须要有一个初始状态为$x_0 \sim \mathcal{N}(\check{x}_0,\check{P}_0)$。
+ 其实应该将${x_k} = {A_{k-1}}{x_{k-1}} + {v_k} + {w_k}$写成${x_k} = {A_{k-1}}{x_{k-1}} + B_k{u_k} + {w_k}$,$B_k \in \mathbb{R}^{M \times N}$这种更一般的形式，不过前者对于推导KF已经够用了，因此这里就简化一下更方便。

### 卡尔曼滤波公式

​       直接上来就给出通用的KF的公式，明白整个文章最终要得到的是什么东西。🧿有的放矢🧿，KF的核心其实就是下面所示的5个公式，实现k时刻系统状态的估计。
$$
\begin{align}
{{\check x}_k} &= {A_{k - 1}}{{\hat x}_{k - 1}} + {v_k}，状态预测\\
\label{variance_predict}
{{\check P}_k} &= {A_{k - 1}}{{\hat P}_{k - 1}}A_{k - 1}^T + {Q_k}，方差预测\\
{K_k} &= {{\check P}_k}C_k^T{\left( {{C_k}{{\check P}_k}C_k^T + {R_k}} \right)^{ - 1}}，卡尔曼增益\\
\label{state_update}
{{\hat x}_k} &= {{\check x}_k} + {K_k}\left( {{y_k} - {C_k}{{\check x}_k}} \right)，状态更新\\
{{\hat P}_k} &= \left( {I - {K_k}{C_k}} \right){{\check P}_k}，方差更新
\end{align}
$$

+ 这里给出的公式尽量统一，主要是向STATE ESTIMATION FOR ROBOTICS这本书中做统一。其他参考资料的都差不多。
+ $\check{x}_k$表示k时刻状态的预测（又可理解为先验），$\hat{x}_k$表示k时刻状态的更新（又可理解为后验）,$\check{P}_k,\hat{P}_k$同理。
+ 从预测模型中可以看出，在没有额外观测时，系统误差是在不断增大的，因为每一个运动状态都有不确定性，不断累加就会增大其误差。为什么$\ref{variance_predict}$是相加，可以从运动模型的方差计算公式入手理解，按照高斯分布的线性变换法则，方差就是相加的。
+ 注意$\ref{state_update}$中${y_k} - {C_k}{{\check x}_k}$就是整个更新过程中的更新量，是有实际物理意义的，表达的是观测值和期望预测值之间的差，$K_k$可以看成这部分更新量的权重，当观测模型误差大时，$K_k$会变小，使得系统更偏向运动模型预测得到的状态估值，如果观测模型误差小，$K_k$会变大，系统更偏向于观测得到的值，会对原有的运动模型得到的状态估值给一个较大的修正量。

### 从极大验后估计MAP的角度解释

#### 从极大后验概率的角度解释状态估计

​		既然要从MAP的角度来解释KF，那就要先从[贝叶斯公式](#Bayesian Estimation or Rules)来描述一下状态估计问题（状态估计的通用形式之前已经介绍过了，这里更多从概率角度进行解释）。不考虑是什么系统，不考虑服从什么分布，状态估计就是要从给定的先验信息和所有时刻的运动v和观测y中推测出所有时刻的最优状态$\hat{x}$，那么状态估计就变成如下公式$\ref{map_init}$所示：
$$
\begin{align}
\label{map_init}
\hat{\mathbf{x}} &=\arg \max _{\mathbf{x}} p(\mathbf{x} \mid \mathbf{v}, \mathbf{y})，MAP的目标 \\
&=\arg \max _{\mathbf{x}} \frac{p(\mathbf{y} \mid \mathbf{x}, \mathbf{v}) p(\mathbf{x}, \mathbf{v})}{p(\mathbf{y}, \mathbf{v})}，贝叶斯公式展开 \\ 
&=\arg \max _{\mathbf{x}} \frac{p(\mathbf{y} \mid \mathbf{x}, \mathbf{v}) p(\mathbf{x} \mid \mathbf{v})p(\mathbf{v})}{p(\mathbf{y} \mid \mathbf{v})p(\mathbf{v})}，进一步展开然后约掉 \\ 
&=\arg \max _{\mathbf{x}} \frac{p(\mathbf{y} \mid \mathbf{x}, \mathbf{v}) p(\mathbf{x} \mid \mathbf{v})}{p(\mathbf{y} \mid \mathbf{v})}，分母和x无关可视为常数，当x已知时，y与v无关 \\ 
&=\arg \max _{\mathbf{x}} p(\mathbf{y} \mid \mathbf{x}) p(\mathbf{x} \mid \mathbf{v})， 最终得到此式
\label{map_func}
\end{align}
$$

​		其中：
$$
\begin{align}
\mathbf{x}&=\mathrm{x}_{0: K}=\left(\mathrm{x}_{0}, \ldots, \mathrm{x}_{K}\right)\\ 
\mathbf{v}&=\left(\check{\mathrm{x}}_{0}, \mathrm{v}_{1: K}\right)= (\check{\mathrm{x}}_{0}, \mathrm{v}_{1}, \ldots, \mathrm{v}_{K})\\
\mathbf{y}&=\mathbf{y}_{0: K}=\left(\mathrm{y}_{0}, \ldots, \mathrm{y}_{K}\right)
\end{align}
$$

+ 粗体和细体有时候分的不是那么严谨，容易造成理解困难。

​		根据[线性高斯系统](#线性高斯系统)中的**假设**，对于所有时刻$k=0,...,K$,所有的噪声$\omega_k,n_k$之间是无关的，因此可以对式$\ref{map_func}$中的$p(\mathbf{y} \mid \mathbf{x})$进行因子分解转换为所有条件概率相乘的形式，即变成:
$$
\begin{align}
p(\mathbf{y} \mid \mathbf{x})=\prod_{k=0}^{K} p({y}_{k} \mid {x}_{k})
\end{align}
$$
​		同时将式$\ref{map_func}$中的$p(\mathbf{x} \mid \mathbf{v})$分解为：
$$
\begin{align}
p(\mathbf{x} \mid \mathbf{v})=p\left(\mathrm{x}_{0} \mid \check{\mathrm{x}}_{0}\right) \prod_{k=1}^{K} p\left(\mathrm{x}_{k} \mid \mathrm{x}_{k-1}, {v}_{k}\right)，初始状态提出来方便后面连乘
\end{align}
$$
​		根据[多元高斯向量的条件PDF以及条件高斯PDF](https://blog.csdn.net/tanghonghanhaoli/article/details/107908154)的公式，可以将$p(\mathrm{x}_{0} \mid \check{\mathrm{x}}_{0}),p\left(\mathrm{x}_{k} \mid \mathrm{x}_{k-1}, {v}_{k}\right),p({y}_{k} \mid {x}_{k})$具体展开为如下所示：
$$
\begin{align}
p\left({x}_{0} \mid \check{{x}}_{0}\right) &= \frac{1}{\sqrt{(2 \pi)^{N} \operatorname{det} \check{\mathbf{P}}_{0}}} \times \exp \left(-\frac{1}{2}\left({x}_{0}-\check{{x}}_{0}\right)^{T} \check{\mathbf{P}}_{0}^{-1}\left({x}_{0}-\check{{x}}_{0}\right)\right) \\
p\left({x}_{k} \mid {x}_{k-1}, {v}_{k}\right) &= \frac{1}{\sqrt{(2 \pi)^{N} \operatorname{det} \mathbf{Q}_{k}}} \\ &\times \exp \left(-\frac{1}{2}\left({x}_{k}-\mathbf{A}_{k-1}{x}_{k-1}-v_k \right)^{T} \mathbf{Q}_{k}^{-1} \left(x_k-\mathbf{A}_{k-1}{x}_{k-1}-{v}_{k}\right)\right) \\
p\left({y}_{k} \mid {x}_{k}\right) &= \frac{1}{\sqrt{(2 \pi)^{M} \operatorname{det} \mathbf{R}_{k}}} \exp \left(-\frac{1}{2}\left({y}_{k}-\mathbf{C}_{k} {x}_{k}\right)^{T} \mathbf{R}_{k}^{-1}\left({y}_{k}-\mathbf{C}_{k} {x}_{k}\right)\right)
\end{align}
$$
​		把上面这三个代入到$\arg \max _{\mathbf{x}} p(\mathbf{y} \mid \mathbf{x}) p(\mathbf{x} \mid \mathbf{v})$ $\ref{map_func}$,然后两边取对数，将主要关注点从指数位置下沉到正常位置（即理解为求解概率最大变成求解目标函数最小），然后去掉前面的一些和状态无关的系数因子（机器人中的状态估计P35-36），就可以得到如下所示的最小化的目标函数。
$$
\begin{align}
\ln (p({y} \mid {x}) p({x} \mid {v}))=\ln p\left({x}_{0} \mid \check{{x}}_{0}\right)+\sum_{k=1}^{K} \ln p\left({x}_{k} \mid {x}_{k-1}, {v}_{k}\right)+\sum_{k=0}^{K} \ln p\left({y}_{k} \mid {x}_{k}\right)
\end{align}
$$
​		把具体的高斯展开结果带入上述等式，同时定义几个变量来简化目标函数，得到$\ref{object_final}$：
$$
\begin{align}
J_{v, k}({x}) &= \begin{cases}\frac{1}{2}\left({x}_{0}-\check{{x}}_{0}\right)^{T} \check{\mathbf{P}}_{0}^{-1}\left({x}_{0}-\check{{x}}_{0}\right), & k=0 \\ \frac{1}{2}\left({x}_{k}-\mathbf{A}_{k-1} {x}_{k-1}-{v}_{k}\right)^{T} \mathbf{Q}_{k}^{-1}\left({x}_{k}-\mathbf{A}_{k-1} {x}_{k-1}-{v}_{k}\right), & k=1 \ldots K\end{cases} \\
J_{y, k}({x}) &=\frac{1}{2}\left({y}_{k}-\mathbf{C}_{k} {x}_{k}\right)^{T} \mathbf{R}_{k}^{-1}\left({y}_{k}-\mathbf{C}_{k} {x}_{k}\right), \quad k=0 \ldots K\\
\label{object_final}
J(\mathbf{x}) &= \sum_{k=0}^{K}\left(J_{v, k}({x})+J_{y, k}({x})\right),最大化后验概率就等价于最小化该目标函数
\end{align}
$$
​		即MAP的问题就转换为了目标函数极值求解问题：
$$
\begin{align}
\hat{\mathbf{x}} =\arg \max _{\mathbf{x}} p(\mathbf{x} \mid \mathbf{v}, \mathbf{y})
\rightarrow
\hat{\mathbf{x}} =\arg \min _{\mathbf{x}} J(\mathbf{x})
\end{align}
$$
​		把初始时刻状态$\check{x}_0$,输入运动变量v，和观测值y写成列向量，状态量x同理，将其余系数写成**矩阵**形式如下：

+ 本节后面开始就不分粗细了，是在扣不动了...
+ 空着的部分就是0。

$$
\begin{align}
\label{z}
z=\left[\begin{array}{l}
\check{x}_{0} \\
v_{1} \\
\vdots \\
v_{K} \\
\hline
y_{0} \\
\vdots \\
y_{K}
\end{array}\right], 
x=\left[\begin{array}{l}
x_{0} \\
\vdots \\
x_{K}
\end{array}\right]，MAP即有了观测z来估算状态x
\end{align}
$$

$$
\begin{align}
\label{H}
\mathbf{H}=\left[\begin{array}{rrrr}1 & & & \\ -\mathbf{A}_{0} & 1 & & \\ & \ddots & \ddots & \\ & & -\mathbf{A}_{K-1} & 1 \\ \hline \mathbf{C}_{0} & & & \\ & \mathbf{C}_{1} & & \\ & & \ddots & \\ & & & \mathbf{C}_{K}\end{array}\right]，H矩阵由运动方程和观测方程组合而成
\end{align}
$$

$$
\begin{align}
\label{W}
\mathbf{W}=\left[\begin{array}{llll|llll}\check{\mathbf{P}}_{0} & & & & & & & \\ & \mathrm{Q}_{1} & & & & & & \\ & & \ddots & & & & & \\ & & & \mathbf{Q}_{K} & & & & \\ \hline & & & & \mathbf{R}_{0} & & & \\ & & & & & \mathbf{R}_{1} & & \\ & & & & & & \ddots & \\ & & & & & & &\mathbf{R}_{K}\end{array}\right]，W为对应的随机变量的方差
\end{align}
$$

​		整理一下，矩阵形式的目标函数就是：
$$
\begin{align}
J(\mathrm{x})=\frac{1}{2}(\mathrm{z}-\mathbf{H x})^{T} \mathbf{W}^{-1}(\mathrm{z}-\mathbf{H x})
\label{J_func}
\end{align}
$$
​		对应的概率密度函数为：式$\ref{object_pdf}$，$\eta$为归一化因子，即之前省略掉的系统因子：
$$
\begin{align}
\label{object_pdf}
p(\mathbf{z} \mid \mathbf{x})=\eta \exp \left(-\frac{1}{2}(\mathbf{z}-\mathbf{H x})^{T} \mathbf{W}^{-1}(\mathbf{z}-\mathbf{H x})\right)
\end{align}
$$
​		**显然**这个关于x的**二次型**问题可以用求导的方式得到其解析解（也是最优解）：
$$
\begin{align}
\left.\frac{\partial J(\mathbf{x})}{\partial \mathbf{x}^{T}}\right|_{\hat{\mathbf{x}}} &=-\mathbf{H}^{T} \mathbf{W}^{-1}(\mathbf{z}-\mathbf{H} \hat{\mathbf{x}})=\mathbf{0} \\
& \Rightarrow\left(\mathbf{H}^{T} \mathbf{W}^{-1} \mathbf{H}\right) \hat{\mathbf{x}}=\mathbf{H}^{T} \mathbf{W}^{-1} \mathbf{z}
\label{optimal_solution}
\end{align}
$$

#### 用Kalman Filter解决状态估计

​		接下来步入正题Kalman Filter的角度解释，首先看一张图，以图解的方式从MAP的角度阐述KF过程,如下：

<img src="E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\Kalman Filter pipeline.png" style="zoom:150%;" />

<center style="color:#FF0000">图2.从map解释KF</center>

​        **首先做一些假设**。假设目前已经拿到k-1时刻系统的状态估计值和对应的方差${\hat{x}_{k-1},\hat{P}_{k-1}}$，这两个值是从系统一开始的0时刻不断推导得到的，同时也有了k时刻的运动向量$v_k$和观测$y_k$，状态估计的目的是为了估计得到k时刻的系统状态估值${\hat{x}_{k},\hat{P}_{k}}$。这里需要知道的是实际上状态是一直累加变化的，在线性高斯系统中完全符合马尔科夫性，因此基于马尔科夫假设，只用前一时刻的状态和方差，在当前观测和运动的基础上，估计当前时刻的状态（**这就是KF的思想**）。于是状态估计问题就可以看成是如下式子所示：(这段话的意思可以用式子之后的图来表示)
$$
\begin{align}
\{{\hat{x}_{k-1},\hat{P}_{k-1}},v_k,y_k \} \mapsto \{{\hat{x}_{k},\hat{P}_{k}}\}
\end{align}
$$
![](E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\Recursive filter replaces past data with an estimate.png)

<center style="color:#FF0000">图3.递归滤波将过去所有的数据融合成一次估计</center>

​		那么KF只用前一时刻的状态$\hat{x}_{k-1}^{'}$(它的意思是前一时刻的状态其实是由0-k时刻的全部状态所估计出来的，如图3所示。只是KF都不看k-1时刻之前的状态了，同时KF无法预知未来，因此带有未来信息的状态$\hat{x}_{k-1}^{'}$将会被消掉，作为一个递推算法，这是不合理的，因此为了和$\hat{x}_{k-1}$作区分多加了一个变量，可不用计较它的形式)，估计当前时刻的状态$\hat{x}_k$，则状态向量$\hat{x} = [\hat{x}_{k-1}^{'},\hat{x}_k]^T$只有两维了，同时对应的矩阵形式中$\ref{z}$, $\ref{H}$,$\ref{W}$也只剩相关的系数和权重了，如下所示$z=\mathbf{H}\hat{x}$：
$$
\begin{align}
\mathbf{z}=\left[\begin{array}{c}
\hat{\mathbf{x}}_{k-1} \\
\mathbf{v}_{k} \\
\mathbf{y}_{k}
\end{array}\right], \quad \mathbf{H}=\left[\begin{array}{cc}
1 & \\
-\mathbf{A}_{k-1} & 1 \\
& \mathbf{C}_{k}
\end{array}\right], \quad \mathbf{W}=\left[\begin{array}{lll}
\hat{\mathbf{P}}_{k-1} & & \\
& \mathbf{Q}_{k} & \\
& & \mathbf{R}_{k}
\end{array}\right]
\end{align}
$$
​		带入$\ref{optimal_solution}$那么此时MAP下状态估计问题的最优解就变成了:
$$
\begin{align}
\label{LS}
\left[\begin{array}{cc}
\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1} & -\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \\
-\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1} & \mathbf{Q}_{k}^{-1}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}
\end{array}\right]\left[\begin{array}{c}
\hat{\mathbf{x}}_{k-1}^{\prime} \\
\hat{\mathbf{x}}_{k}
\end{array}\right]
=\left[\begin{array}{c}
\hat{\mathbf{P}}_{k-1}^{-1} \hat{\mathbf{x}}_{k-1}-\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{v}_{k} \\
\mathbf{Q}_{k}^{-1} \mathbf{v}_{k}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}
\end{array}\right]
\end{align}
$$
​		重点关注的是$\hat{x}_k$，因此可以采用边缘化（舒尔补）的方式将$\hat{x}_{k-1}^{'}$从矩阵中消掉，进而只保留下包含$\hat{x}_k$的等式，消除的方式就是想方设法让要消除的变量对应的系数变成0，即$\ref{LS}$左乘如下矩阵：
$$
\begin{align}
\left[\begin{array}{cc}
\mathbf{1} & 0 \\
\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1} & 1
\end{array}\right]
\end{align}
$$
​		 $\ref{LS}$ 就变成了如下的形式，从中可以得到$\hat{x}_k$的最优解：
$$
\begin{gathered}
{\left[\begin{array}{cc}
\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1} & -\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \\
\mathbf{0} & \mathbf{Q}_{k}^{-1}-\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1} \\
& \times \mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}
\end{array}\right]\left[\begin{array}{c}
\hat{\mathbf{x}}_{k-1}^{\prime} \\
\hat{\mathbf{x}}_{k}
\end{array}\right]} \\
=\left[\begin{array}{c}
\hat{\mathbf{P}}_{k-1}^{-1} \hat{\mathbf{x}}_{k-1}-\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{v}_{k} \\
\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1}\left(\hat{\mathbf{P}}_{k-1}^{-1} \hat{\mathbf{x}}_{k-1}-\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{v}_{k}\right) \\
+\mathbf{Q}_{k}^{-1} \mathbf{v}_{k}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}
\end{array}\right] .
\end{gathered}
$$
​		进一步化简（不看$\hat{x}_{k-1}^{'}$的项）得：
$$
\begin{align}
\label{x_hat}
&(\underbrace{\mathbf{Q}_{k}^{-1}-\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1} \mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1}}_{\left(\mathbf{Q}_{k}+\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T}\right)^{-1} \text { by }(\quad SMW \quad formula)}
+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}) \hat{\mathbf{x}}_{k}\\
&=\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1}
\left(\hat{\mathbf{P}}_{k-1}^{-1} \hat{\mathbf{x}}_{k-1}-\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{v}_{k}\right) \\
&+\mathbf{Q}_{k}^{-1} \mathbf{v}_{k}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}
\end{align}
$$
​		新定义一些变量$\check{\mathbf{P}}_{k}$(其实就是随机变量线性变化后方差的变换公式),$\hat{\mathbf{P}}_{k}$简化推导过程：
$$
\begin{align}
\label{check_p_k_def}
&\check{\mathbf{P}}_{k}=\mathbf{Q}_{k}+\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T} \\
\label{hat_p_k_def}
&\hat{\mathbf{P}}_{k}=\left(\check{\mathbf{P}}_{k}^{-1}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}\right)^{-1}
\end{align}
$$
​		将$\check{\mathbf{P}}_{k}$,$\hat{\mathbf{P}}_{k}$带入$\ref{x_hat}$进而可以得出（同时令$\check{x}_k$为状态的预测值）：
$$
\begin{align}
\hat{\mathbf{P}}_{k}^{-1} \hat{\mathbf{x}}_{k}&=\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1}\\
&\times\left(\hat{\mathbf{P}}_{k-1}^{-1} \hat{\mathbf{x}}_{k-1}-\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{v}_{k}\right)+\mathbf{Q}_{k}^{-1} \mathbf{v}_{k}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}\\
\label{pa}
&=\underbrace{\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1} \hat{\mathbf{P}}_{k-1}^{-1}}_{\check{\mathbf{P}}_{k}^{-1} \mathbf{A}_{k-1} \text 由下文推得} \hat{\mathbf{x}}_{k-1}\\
&+\underbrace{\left(\mathbf{Q}_{k}^{-1}-\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1} \mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1}\right)}_{\check{\mathbf{P}}_{k}^{-1}  \text { by }(\quad SMW \quad formula)} \mathbf{v}_{k}\\
&+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}\\
\label{hat_p_k_inv_hat_x_k}
&=\check{\mathbf{P}}_{k}^{-1} \underbrace{\left(\mathbf{A}_{k-1} \hat{\mathbf{x}}_{k-1}+\mathbf{v}_{k}\right)}_{\check{\mathbf{x}}_{k}}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k},
\end{align}
$$
​		式$\ref{pa}$的推导如下：
$$
\begin{align}
&\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1} \underbrace{\left(\hat{\mathbf{P}}_{k-1}^{-1}+\mathbf{A}_{k-1}^{T} \mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\right)^{-1}}_{\text {apply SMW formula again }} \hat{\mathbf{P}}_{k-1}^{-1} \\
&=\mathbf{Q}_{k}^{-1} \mathbf{A}_{k-1}\left(\hat{\mathbf{P}}_{k-1}-\hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T} \underbrace{\left(\mathbf{Q}_{k}+\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T}\right)^{-1}}_{\check{\mathbf{P}}_{k}^{-1}} \mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \right) \hat{\mathbf{P}}_{k-1}^{-1} \\
&=(\mathbf{Q}_{k}^{-1}-\mathbf{Q}_{k}^{-1} \underbrace{\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T}}_{\check{\mathbf{P}}_{k}-\mathbf{Q}_{k}} \check{\mathbf{P}}_{k}^{-1}) \mathbf{A}_{k-1} \\
&=\left(\mathbf{Q}_{k}^{-1}-\mathbf{Q}_{k}^{-1}+\check{\mathbf{P}}_{k}^{-1}\right) \mathbf{A}_{k-1} \\
&=\check{\mathbf{P}}_{k}^{-1} \mathbf{A}_{k-1}
\end{align}
$$
​		这样便可得到**递归滤波器**的形式，从k-1时刻递推出k时刻的系统状态。
$$
\begin{align}
\label{check_p_k}
\check{\mathbf{P}}_{k} &=\mathbf{A}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{A}_{k-1}^{T}+\mathbf{Q}_{k},\ref{check_p_k_def} \\
\label{check_x_k}
\check{\mathbf{x}}_{k} &=\mathbf{A}_{k-1} \hat{\mathbf{x}}_{k-1}+\mathbf{v}_{k}，其实就是运动方程 \\
\label{inv}
\hat{\mathbf{P}}_{k}^{-1} &=\check{\mathbf{P}}_{k}^{-1}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}, \ref{hat_p_k_def} \\
\label{hat_x_k}
\hat{\mathbf{P}}_{k}^{-1} \hat{\mathbf{x}}_{k} &=\check{\mathbf{P}}_{k}^{-1} \check{\mathbf{X}}_{k}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{y}_{k}, \ref{hat_p_k_inv_hat_x_k}
\end{align}
$$
​		而KF的经典形式只要稍加变换就可得到：

​		令卡尔曼增益$K_k$为：
$$
\begin{align}
\mathbf{K}_{k}=\hat{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1}
\end{align}
$$
​		在式$\ref{inv}$式左右各左乘$\hat{\mathbf{P}_k}$得：
$$
\begin{align}
1 &=\hat{\mathbf{P}}_{k}\left(\check{\mathbf{P}}_{k}^{-1}+\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \mathbf{C}_{k}\right) \\
&=\hat{\mathbf{P}}_{k} \check{\mathbf{P}}_{k}^{-1}+\mathbf{K}_{k} \mathbf{C}_{k},上式展开并把K_k带入 \\
\label{hat_p_k}
\hat{\mathbf{P}}_{k} &= \left(\mathbf{1}-\mathbf{K}_{k} \mathbf{C}_{k}\right) \check{\mathbf{P}}_{k},等式两侧右乘\mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1} \\
\underbrace{\hat{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1}}_{K_t} &= \left(\mathbf{1}-\mathbf{K}_{k} \mathbf{C}_{k}\right) \check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1}\\
\mathbf{K}_{k}\left(\mathbf{1}+\mathbf{C}_{k} \check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1}\right) &=\check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \mathbf{R}_{k}^{-1},等式两侧右乘R_k \\
\label{k_k}
K_k &= \check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T} \left( \mathbf{C}_{k} \check{\mathbf{P}}_{k} \mathbf{C}_{k}^{T}+R_k \right)^{-1},即为卡尔曼增益
\end{align}
$$
​		综上所述，最终的卡尔曼滤波的核心公式就是：
$$
\begin{align}
{{\check x}_k} &= {A_{k - 1}}{{\hat x}_{k - 1}} + {v_k}，\ref{check_x_k}\\
{{\check P}_k} &= {A_{k - 1}}{{\hat P}_{k - 1}}A_{k - 1}^T + {Q_k}，\ref{check_p_k}\\
{K_k} &= {{\check P}_k}C_k^T{\left( {{C_k}{{\check P}_k}C_k^T + {R_k}} \right)^{ - 1}}，\ref{k_k} \\
{{\hat x}_k} &= {{\check x}_k} + {K_k}\left( {{y_k} - {C_k}{{\check x}_k}} \right)，\ref{hat_x_k}等式两侧左乘\hat{\mathbf{P}}_k即得 \\
{{\hat P}_k} &= \left( {I - {K_k}{C_k}} \right){{\check P}_k}，\ref{hat_p_k}
\end{align}
$$
​		**从MAP角度解释KF完结撒花**。


### 从贝叶斯估计的角度解释

​       卡尔曼是线性系统的递推形式（recursive，也就是从$x_{k-1}$估计到$x_k$)的无偏最优估计（无偏最优指得是期望等于其真实值，具体内容鸽）。

​       由[观测方程可得](#线性高斯系统)，用来估计当前的先验：

TODO

### 从增益最优的角度来解释

TODO

# Extend Kalman Filter

​		接下来几乎全文照抄高博知乎(高博YYDS)

​		然而实际情况是状态并不是服从高斯分布的，噪声也不是，而且运动方程和观测方程也不是线性的，高斯分布经过非线性变换是什么变换也估摸不清楚了，于是有了EKF，提出把一个非线性的东西进行**线性化**表示，取$\hat{x}_{k-1}，\check{x}$处的近似值，那么公式$\ref{motion_function},\ref{observation_function}$中表示的运动方程和观测方程就线性化成如下形式：
$$
\begin{equation}
\begin{aligned}
f\left(x_{k-1}, v_{k}, w_{k}\right) &\approx f\left(\hat{x}_{k-1}, v_{k}, 0\right)+\frac{\partial f}{\partial x_{k-1}}\left(x_{k-1}-\hat{x}_{k-1}\right)+\frac{\partial f}{\partial w_{k}} w_{k} \\
g\left(x_{k}, n_{k}\right) &\approx g\left(\check{x}_{k}, 0\right)+\frac{\partial g}{\partial x_{k}} n_{k}
\end{aligned}
\end{equation}
$$
​	同时EKF认为状态和噪声**都服从高斯分布**（强假设），然后用线性化之后的矩阵去代替卡尔曼滤波器里的转移矩阵和观测矩阵即可。
$$
\begin{equation}
\begin{aligned}
&\check{P}_{k}=F_{k-1} \hat{P}_{k-1} F_{k-1}^{T}+Q_{k}^{\prime} \\
&\check{x}_{k}=f\left(\hat{x}_{k-1}, v_{k}, 0\right) \\
&K_{k}=\check{P}_{k} G_{k}^{T}\left(G_{k} \check{P}_{k} G_{k}^{T}+R_{k}^{\prime}\right)^{-1} \text { 其中 } F_{k-1}=\left.\frac{\partial f}{\partial x_{k-1}}\right|_{\hat{x}_{k-1}}, G_{k}=\left.\frac{\partial g}{\partial x_{k}}\right|_{\tilde{x}_{k}} \\
&\hat{P}_{k}=\left(I-K_{k} G_{k}\right) \tilde{P}_{k} \\
&\hat{x}_{k}=\check{x}_{k}+K_{k}\left(y_{k}-g\left(\check{x}_{k}, 0\right)\right)
\end{aligned}
\end{equation}
$$
​		所以EKF面临的一个重要问题是，当一个高斯分布经过非线性变换后，如何用另一个高斯分布近似它？按照它现在的做法，存在以下的局限性：（注意是滤波器自己的局限性，还没谈在SLAM问题里的局限性）。

1. 即使是高斯分布，**经过一个非线性变换后也不是高斯分布**。EKF只计算均值与协方差，是在用高斯近似这个非线性变换后的结果。（实际中这个近似可能很差）。
2. 系统本身线性化过程中，丢掉了高阶项，在非高斯分布时，误差就很大了。
3. 线性化的工作点往往不是输入状态真实的均值，而是一个估计的均值。于是，在这个工作点下计算的$F,G$，也不是最好的。
4. 在估计非线性输出的均值时，EKF算的是$\mu_y=f(\mu_x)$的形式。这个结果几乎不会是输出分布的真正期望值。协方差也是同理。

​		那么，怎么克服以上的缺点呢？途径很多:

+ 为了克服第3条工作点的问题，以EKF估计的结果为工作点，重新计算一遍EKF，直到这个工作点变化够小。是为**迭代EKF（Iterated EKF, IEKF）**。

+ 为了克服第4条，除了计算$\mu_y=f(\mu_x)$，再计算其他几个精心挑选的采样点，然后用这几个点估计输出的高斯分布。是为**Sigma Point KF（SPKF，或UKF)**。
+ 如果说不要高斯分布假设，凭什么要用高斯去近似一个长得根本不高斯的分布呢？于是问题变为，丢掉高斯假设后，怎么描述输出函数的分布就成了一个问题。一种比较暴力的方式是：用足够多的采样点，来表达输出的分布。这种[蒙特卡洛](https://www.zhihu.com/search?q=蒙特卡洛&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A103411007})的方式，也就是粒子滤波的思路。
+ 如果再进一步，可以丢弃滤波器思路，说：**为什么要用前一个时刻的值来估计下一个时刻呢**？**可以把所有状态看成变量，把运动方程和观测方程看成变量间的约束，构造误差函数，然后最小化这个误差的二次型。**这样就会得到非线性优化的方法，在SLAM里就走向图优化那条路上去了。不过，非线性优化也需要对误差函数不断地求梯度，并根据梯度方向迭代，因而局部线性化是不可避免的。

# Error State Kalman Filter

​		KF假设系统是线性高斯，EKF为了满足线性线性化假设，强行线性化的操作很多时候不符合实际情况，就有人想到如果将状态的误差看做是“状态”呢，每次对状态的误差按照KF的流程走一遍，然后对真实状态进行更新，然后再计算误差，一直迭代，于是有了ESKF。

​		无论是Joan大佬采用的基于四元数表示旋转的ESKF，还是weikun zhen、高博等采用的SO3表示的ESKF，其核心的基于KF思想的预测predict和更新update/correct都是一样的，主要的关注点是如何利用状态误差构建关于状态误差的状态方程以及状态和状态误差之间的更新。区别仅在于表示动力学方程中和旋转相关的$\delta \theta$不同。

​		使用状态误差做KF的优点：

+ 在旋转的处理上，ESKF的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。而传统KF需要用到四元数（4维）或者更高维的表达（旋转矩阵，9维），要不就得采用带有奇异性的表达方式（欧拉角）。

+ ESKF总是在原点附近，离奇异点较远，并且也不会由于离工作点太远而导致线性化近似不够的问题。

+ ESKF的状态量为小量，其二阶变量相对来说可以忽略。同时大多数雅可比矩阵在小量情况下变得非常简单，甚至可以用单位阵代替。

+ 误差状态的运动学也相比原状态变量要来得更小，因为可以把大量更新部分放到原状态变量中。

​		考虑到IMU的特性，用什么类型的KF都会导致飘移，因此多传感器融合成为必要性，除了IMU之外的传感器的观测数据可以作为KF中更新模块的观测，纠正IMU的bias。

​		假设观测方程为：$v \sim \mathcal{N}(0,V)$为高斯分布的观测噪声
$$
\begin{align}
y=h(x_t)+v
\end{align}
$$
​		按照KF的思想，其对应的预测和更新为：戴帽子的符号和KF表示的意义相同，$\leftarrow$表示的就是更新，赋值的意思，数学上表示的意义不严谨，权可认为与"="意义相同。
$$
\begin{align}
\check{\delta x} &\leftarrow F_x(x,u_m) \cdot \delta x, 误差预测\\
\check{P} &\leftarrow F_xPF_x^T + F_iQF_i^T, 协方差更新\\
K &= \check{P}H^T(H\check{P}H^T+V)^{-1}, 卡尔曼增益\\
\hat{\delta x} &\leftarrow K(y-h(\hat{x}_t))，误差更新\\
\hat{P} &= \check{P}+KH\check{P} = (I+KH)\check{P}，协方差更新
\end{align}
$$

+ 具体的公式这里就不贴了，在Joan大神的论文中写的很清楚。
+ 其中$\delta x \sim \mathcal{N}(\hat{\delta x},P)$,$F_x,F_i$分别是状态方程对误差状态和噪声的一阶导。
+ $H \equiv \left.\frac{\partial h}{\partial \delta x}\right|_{x} = \left.\frac{\partial h}{\partial x_t}\right|_{x} \left.\frac{\partial x_t}{\partial \delta x}\right|_{x}=H_x X_{\delta x}$，是根据链式法则求得的观测方程对状态误差的偏导数，在KF中这里的H就是线性化的观测矩阵，仿照EKF的思想实现线性化的假设，这里也对状态误差做了线性化处理。
+ 需要注意的是ESKF中是对误差状态建模的，因此需要将误差状态更新inject到实际的状态向量中，然后归零继续下一次线性化展开和KF过程。**需要明白，ESKF还是按照KF的假设来的，但处理的对象是状态误差，虽然也有非线性化的操作，但是和EKF直接对状态做KF是不同的。在图优化中也用了线性化的操作。**

疑问：

+ 将旋转表示在local系和global系之间的区别是？分别在什么场景下使用呢？

# Particle Filter

​       当然有一些系统不能有用高斯分布近似模拟噪声，而且即使一开始的状态是高斯分布的，但是经过非线性变换之后的分布不明，因此需要使用一些像粒子滤波这样更复杂滤波器，直接通过暴力的方式不断采样-算权重-重采样。越符合观测的粒子拥有越大的权重，而权重越大就越容易在重采样时被采到。核心是采样数量，以及权重如何设置。当然还有很多问题，没做过也不知道深浅。

![](E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\pf.jpg)

<center style="color:#FF0000">图4. Particle Filter示意图</center>

+ FastSLAM是基于PF的Vslam

# 番外👀

## Filter到底是不是直观意义上的过滤？✅

​        **[滤波器](https://www.zhihu.com/search?q=滤波器&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A1267117107})**的本质是根据传感器的信号估计真实状态。静态的、离线的估计可以使用最小二乘或者批估计的方法，但是**动态的、在线的估计**（如机器人的位姿）就需要使用像卡尔曼这样复杂一点的滤波方法。如果把KF想成是一个能从带有噪声的观测数据/信号中得到真实的状态，那么可以认为是过滤掉了噪声，但是远比过滤要复杂的多，这里已经不是字面上的过滤这一个操作了，而是广义的过滤。

### filter & smoothing & 图优化

+ 在状态估计问题中，假设某物体的运动状态为$x_1,x_2,...,x_{k-1},x_k$，他们随时间不断变化的，如果认为当前状态$x_k$只和前面的$x_{k-1}$有关，那么就用KF,EKF,ESKF等基于filter的方法，也就是状态估计满足马尔科夫模型假设；如果认为当前状态$x_k$和前面的都有关，则用图优化的方式，采用batch的形式迭代进行最优状态的估计。

+ 区别只在于使用的信息量，如果只用前一个或者前几个，那么就是KF-based的或者sliding window KF based的，当然马尔科夫模型假设太强了，实际不一一定只和前面的几个有关，那么就会采用batch的形式一起图优化，理论上拿到的信息更多，结果应该更好才对。而实际使用中考虑到效率和实时性的要求，sliding window的方式更实用。

+ smoothing则是说根据k时刻之后的状态，e.g. $x_{k+1}$反向更新k时刻的状态$x_{k}$，反向的这个过程提升了当前状态估计结果的精度，毕竟前后都考虑到了，但是因为用了未来时刻的状态估计现在的状态，说明它做不到实时，还可能更耗时，只能是一种off-line线下操作。而这么做的目的就是为了得到更好的状态估计结果。

### 不同滤波器方法之间的简单比较

+ **卡尔曼滤波是递归的线性高斯系统最优估计。**

+ **EKF将NLNG系统在工作点附近近似为LG进行处理。**

+ **PF去掉高斯假设，以粒子作为采样点来描述分布。**

![](E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\compare between filters.jpg)

<center style="color:#FF0000">图5. 不同种类的滤波器之间的简单比较示意图</center>

### 其他非高斯分布的解决方案

​		在视觉slam中存在大量非线性非高斯分布的情况，e.g.图像的点像素值不是高斯分布的，而其逆深度是符合高斯分布的，因此转换为逆深度进行求解。

## IMU✅

​		本文介绍的状态估计方法中，几乎每种都使用到了IMU这种传感器作为观测数据的输入，因此，有必要了解IMU到底是个啥，其工作原理十分有利于理解上述公式推导。这里只简单介绍和状态估计相关的，IMU本身是一个大的方向，我也没能力总结出来。<span name = "imu">imu</span>

### 1. IMU类型

- 3轴IMU即只有3轴陀螺仪的IMU，其因为只有一个3轴陀螺仪**gyroscopes**，所以只能感知载体roll、pitch、yawl共3个自由度的姿态信息。
- 6轴IMU在3轴IMU的基础上加装了3轴加速度计**accelerometers**，因此在感知载体姿态的基础上，还能感知载体3个自由度上的加速度信息。
- 9轴IMU在6轴IMu的基础上加装了3轴磁强计**magnetometer**，由于3轴陀螺仪只能估计载体自身的相对位姿变化（通过加速度计也可获得载体的绝对roll和pitch），单凭3轴陀螺仪无法获取载体的全部姿态信息，而通过3轴磁强计就可以，本质上磁强计的感知原理类似于指南针。

​		有学者说，磁强计在小范围内还行，大范围容易受到地磁不一样导致的误差，所以目前6轴的在机器人领域用的更多一些；由于工业制造的问题，IMU安装到板子上之后，由于散热器震动，高温等原因会导致IMU给出的数值不稳定，这也是加装了IMU的状态估计方法在使用时尽量在线online标定的一个原因。

### 2. IMU状态估计原理

​		其实是符合牛顿第二定律`f=ma`，利用动力学原理，从IMU测量的载体自身的旋转和加速度来得到当前的位置。其中就使用到了IMU积分的相关内容，加速度积分得到速度，速度积分得到位置，虽然IMU本身会输出一个直接积分得到的速度和位置，但是几乎无法使用，因此还是要用加速度和角速度自己积分。而为了提升积分的效率，有了**预计分**的思想。其中的数学也很硬核。

​		6轴IMU由两部分组成，加速度计和陀螺仪，这两个一起都是有偏置误差的，也就是说初始位置0位不准确，需要进行标定，同时测量量还是有噪声的。也就是说IMU输出的值是包含了偏置bias和噪声的，同时加速度计可以感应到地球重力，因此真实的载体的加速度需要减去重力加速度。

### 3. IMU预计分

IMU预计分涉及到的数学有点多，目前还没有来得及学习和整理。

+ 百度百科的欧拉积分中有错误，但是一直不修改。

## 概率论✅

### 随机变量

​		有几个概念上的生疏导致理解困难。

1. 状态服从高斯分布是什么意思？是随机变量吗？噪声可以理解。

> 随机变量 is a variable whose possible values are the outcomes of a random phenomenon
>
> 随机变量的意思是将样本所在的**样本空间到实数域的映射**，状态是一个在某一空间（e.g. SE3,欧式空间）真值周围随机变化的，为了描述其处于哪里，以及其对应的概率，需要映射到实数域来表示，因此完全可称状态为是随机变量。
>
> ![img](https://pic2.zhimg.com/80/v2-7e9bdd0176835735cbd9cb5c51d2d4e7_720w.jpg?source=1940ef5c)

2. 随机变量方差线性变换公式

> $$
> \begin{align}
> D(x_1+x_2) &=D(x_1)+D(x_2) \\ 
> D(A x_1 + b) &= A^2D(x_1) + D(b), if \ d\ is \ a\  random \ variable
> \end{align}
> $$
>
> Emmmm...

​		简单罗列一些概率论中的常见术语，学的时候没用起来，用的时候早忘了...

1. 某一服从某一分布随机变量的概率密度函数**probability density function,PDF**:连续型如此，离散型就求和

$$
\begin{align}
F_{X}(x)=\int^x_{-\infty}p(\tau)d\tau
\end{align}
$$

2. 多个类似随机变量的概率密度函数称为**联合概率密度函数**：

$$
\begin{align}
f_{X}(x)=\frac{\partial^n{F}}{\partial{x_1}...\partial{x_n}}\rvert_{x} \\  
{F}_{X}(x_1,x_2,...,x_n)=\int_{D}f_{X}(x_1,x_2,...,x_n)dx_1...dx_n
\end{align}
$$

3. 随机变量的两个统计意义上的度量$\mu,\Sigma=\delta^2$:

$$
\begin{align}
E(X) &= \int^\infty_{-\infty}xp(x)dx \\
Var(X)&= E([X-E(X)]^2) \\
&=\int^\infty_{-\infty}[x-E(x)]^2p(x)dx=E(X^2)-E^2(X)
\end{align}
$$

4. Sherman-Morrison-Woodbury

   TODO

### 极大似然估计&极大后验估计

​        是解决同一个任务的两种不同方法，问题是利用已知的数据，推测模型的参数。极大似然估计MLE属于频率派，极大后验估计MAP属于贝叶斯派。

+ MLE是为了最大化这个事情发生的概率。进一步可以推导向最小二乘。
+ MAP是为了最大化在给定数据样本的情况下模型参数的后验概率，需要给定一个先验概率，然后不断修正。

+ 为什么KF可以同时从MAP和贝叶斯推估的角度同时推导出来？是因为其假设为线性高斯的系统，两者推导结果一致，如果不是线性的，结果全然不同。

### Bayesian Estimation or Rules

​       下图真的非常不错

<img src="E:\codefiles\catkin_ws_lidar\src\lidar_eskf\doc\imgs\bayesian estimation.png" style="zoom:150%;" />

<center style="color:#FF0000">图4.贝叶斯公式</center>

$$
\begin{align}
P(B|A) &= \frac {P(A|B)P(B)}{P(A)} \\
P(AB) &= P(A|B)P(B) = P(B|A)P(A)
\end{align}
$$

​       上面就是最基础的**贝叶斯公式**以及变体。如果把$P(A)$用全概率公式写出来，就是将所有B事件发生的概率$P(B)$下，对应A事件发生的条件概率$P(A|B)$的总和，即贝叶斯公式就会变为：

+ 连续型(积分)，$p_{Y}(y),p_{X}(x)$就是边缘概率密度，x,y分别对应B,A

$$
\begin{align}
p(x|y)=\frac{p(x,y)}{p_{Y}(y)}=\frac{p(y|x)p_{X}(x)}{p_{Y}(y)}=\frac{p(y|x)p_{X}(x)}{\int{p(y|x)p_{X}(x)dx}}
\end{align}
$$

+ 离散型(求和)

$$
\begin{align}
P(B_i|A)=\frac{P(A|B_i)P(B_i)}{{\sum_{i=1}^{n}P(A|Bi)P(B_i)}}
\end{align}
$$

# Reference

以下排名不分先后：

+ [知乎上关于各种滤波器的讨论](https://www.zhihu.com/question/46916554)
+ [高博关于ESKF的简明推导](https://zhuanlan.zhihu.com/p/441182819)
+ Quaternion based 的ESKF, Quaternion kinematics for the error-state Kalman filter
+ weikun zhen CMU 2021年博士论文SO3 based的ESKF，Robust State Estimation and Mapping for Autonomous Inspection
+ [LiDAR_ESKF](https://github.com/whu-lyh/lidar_eskf) 坚信talk is cheap, show me the code 的真理。
+ 最硬核的应该直接看原版的STATE ESTIMATION FOR ROBOTICS，高博翻译了中文版。
+ [Kalman Filter's wikipedia](https://en.wikipedia.org/wiki/Kalman_filter#Details).
+ [IMU集合](https://github.com/whu-lyh/IMU-study)
+ [知乎推文1](https://zhuanlan.zhihu.com/p/36745755)