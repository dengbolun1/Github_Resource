现代控制理论

一 状态空间与状态方程

状态与状态空间

在经典控制理论中，采用传递函数来描述系统

y = G(s)u

这里有一个假设为系统的初始条件为零

当系统的初始条件非零时，需要外部输入 u(t) (t \in [t_0,\infty]) 和初始条件的信息，才能确定系统的输出 y(t)(t \in [t_0,\infty])

- 在输入已知的情况下，能唯一确定系统运动在某个时刻的初始信息，称为系统在该时刻的状态
- 状态：在系统未来外部输入已知时，为完全描述系统行为所需的最小一组变量。而系统的行为由某一时刻的状态以及该时刻之后的外部输入唯一地确定

状态向量 

X(t)=\left[ \begin{array}{c}{x_{1}(t)} \\ {x_{2}(t)} \\ {\vdots} \\ {x_{n}(t)}\end{array}\right]

- 对于给定的系统，状态变量的选取不是唯一的
- 状态向量全体构成的空间为状态空间
- 可量测的量（其为状态和输入的函数）所构成的向量称为输出向量

状态方程和输出方程

状态变量的一阶微分方程称为状态方程，而线性状态方程的标准形式为

\dot{X}(t)=A X(t)+B u(t)

其中 X(t) \in \mathrm{R}^{n}, u(t) \in \mathrm{R}^{\prime}， 状态矩阵 A 为 n \times n矩阵，输入矩阵 B 是 n \times l矩阵

当矩阵 A 和 B 为常数矩阵时，称状态方程为定常的；而当其中含有时间的变元时，称为时变的

线性输出方程的标准形式为

Y(t)=C X(t)+D u(t)

其中 Y \in R^{m}, C \in R^{m \times n}, D \in R^{m \times l}

将状态方程和输出方程合称为运动方程

线性系统的运动方程可以写成

\left[ \begin{array}{l}{\dot{X}} \\ {Y}\end{array}\right]=\left[ \begin{array}{ll}{A} & {B} \\ {C} & {D}\end{array}\right] \left[ \begin{array}{l}{X} \\ {U}\end{array}\right]

简记为 \sum)(A,B,C,D)，当 D = 0时，记为\sum(A,B,C)

状态方程的导出

二 状态方程的解

齐次定常状态方程的解

1. 矩阵指数法
   考虑方程
   \dot{X}(t)=A X(t), \quad X\left(t_{0}\right)=X_{0}
   定义矩阵指数
   e^{A t}=I+A t+\frac{1}{2} A^{2} t^{2}+\frac{1}{3 !} A^{3} t^{3}+\ldots \ldots+\frac{1}{k !} A^{k} t^{k}+\ldots \ldots
   有
   X(t)=e^{A\left(t-t_{o}\right)} X\left(t_{0}\right)
2. Laplace法
   设 t_0 = 0，对方程取拉氏变换并考虑初始条件有
   X(t)=L^{-1}\left[(s I-A)^{-1}\right] X(0)
   所以可以求得矩阵指数
   e^{A t}=L^{-1}\left[(s I-A)^{-1}\right]
   其中称 (sI - A)^{-1} 为预解矩阵

矩阵指数的特性

- A e^{A t}=e^{A t} A
- \frac{\mathrm{d}}{\mathrm{d} t} e^{A t}=A e^{A t}
- e^{A t_{1}} \cdot e^{A t_{2}}=e^{A\left(t_{1}+t_{2}\right)}
- 如果 AB = BA 有 e^{A t} \cdot e^{B t}=e^{(A+B) t}
- \left(e^{A t}\right)^{-1}=e^{-A t}
- 如果 A=\operatorname{diag}\left[\lambda_{1}, \quad \lambda_{2}, \quad \lambda_{3}\right]有 
  e^{AT} = diag[e^{\lambda_1t},...,e^{\lambda_3t}]
- T e^{A t} T^{-1}=e^{\left(T A T^{-1}\right) t}

线性定常系统状态转移矩阵

定义

\begin{array}{c}{\dot{\Phi}(t)=A \Phi(t)} \\ {\Phi(0)=I_{n \times n}}\end{array}

如果存在那么

X(t)=\Phi(t) X(0)

注意有：

1. \Phi(t) 可逆，其列向量线性独立
2. \Phi_i(t) 构成一个基本解，从而也称为基本解矩阵

其性质和 e^{At} 类似

\begin{array}{l}{\Phi\left(t_{1}+t_{2}\right)=\Phi\left(t_{1}\right) \Phi\left(t_{2}\right)} \\ {\Phi\left(t_{1}\right) \Phi\left(t_{2}\right)=\Phi\left(t_{2}\right) \Phi\left(t_{1}\right)} \\ {[\Phi(t)]^{n}=\Phi(n t)} \\ {\Phi\left(t_{2}-t_{1}\right) \Phi\left(t_{1}-t_{0}\right)=\Phi\left(t_{2}-t_{0}\right)} \\ {\left[\Phi\left(t_{2}-t_{1}\right)\right]^{-1}=\Phi\left(t_{1}-t_{2}\right)}\end{array}

线性时变系统状态转移矩阵

\dot{X}(t)=A(t) X\left(t_{0}\right), \quad X\left(t_{0}\right)=X_{0}

对应的状态转移矩阵为下面矩阵方程的解

\begin{aligned} \frac{d}{d t} \Phi\left(t, t_{0}\right) &=A(t) \Phi\left(t, t_{0}\right) \\ \Phi\left(t_{0}, t_{0}\right) &=I_{\operatorname{n x} n} \end{aligned}

非齐次状态方程的解

X(t)=e^{A\left(t-t_{0}\right)} X\left(t_{0}\right)+\int_{t_{0}}^{t} e^{A(t-\tau)} B u(\tau) \mathrm{d} \tau

X(t)=L^{-1}\left[(s I-A)^{-1} X(0)\right]+L^{-1}\left[(s I-A)^{-1} B u(s)\right]

X(t)=\Phi\left(t, t_{0}\right) X\left(t_{0}\right)+\int_{t_{0}}^{t} \Phi(t, \tau) B(\tau) u(\tau) \mathrm{d} \tau

三 特征值规范型

假设 X 和 \widetilde{X} 为一线性定常系统的两个状态向量，如果存在非奇异矩阵 P 使得

X  = P \widetilde{X}

而

\begin{array}{l}{\widetilde{A}=P^{-1} A P, \quad \widetilde{B}=P^{-1} B} \\ {\widetilde{C}=C P, \quad \widetilde{D}=D}\end{array}

同时成立

\begin{array}{l}{\operatorname{det}(s I-A)=\operatorname{det}(s I-\widetilde{A})} \\ {C(s I-A)^{-1} B+D=\widetilde{C}(s I-\widetilde{A})^{-1} \widetilde{B}+\widetilde{D}}\end{array}

特征值规范型

在状态的恰当选取下，系统矩阵具有某种特殊的解构，使得某种不变性质一目了然，称具有这种结构的系统描述为规范型

对角线规范型

显然，存在对角线规范型的充要条件为 A 具有 n 个线性独立的特征向量，而相似变换矩阵就由此 n 个特征向量构成

- 如果 A 具有 n 个互不相同的特征值，此时必有 n 个线性独立的特征向量
- 如果 \lambda_i 为 A 的 \sigma_i 重特征值，成立
  \operatorname{rank}\left(\lambda_{i} I-A\right)=n-\sigma_{i}
  即 (\lambda_i I -A)的零空间是\sigma_i维的

特征向量和运动模态

当 A 有 n 个独立特征向量 w_i 时， W=\left[w_{1} w_{2} \dots w_{n}\right]使得

X(t)=\sum_{i=1}^{n} \widetilde{x}(0) e^{\lambda_{t} t} w_{i}

称 e^{\lambda_i t}w_i为系统的模态

模态规范型

假设 A 为 2 \times 2 矩阵，其特征值为

\lambda_{1}=\sigma+j \omega, \quad \lambda_{2}=\sigma-j \omega

对应 \lambda_1 的特征向量为 w_1 = \alpha + j \beta

令 W = [\alpha, \beta]则

W^{-1} A W=\left[ \begin{array}{cc}{\sigma} & {\omega} \\ {-\omega} & {\sigma}\end{array}\right]=\Pi

称为模态规范型

同时

e^{I I t}=\left[ \begin{array}{cc}{e^{\sigma t} \cos \omega t} & {e^{\alpha t} \sin \omega t} \\ {-e^{\sigma t} \sin \omega t} & {e^{\sigma t} \cos \omega t}\end{array}\right]

Fedeeva算法和最小多项式

对

(s I-A)^{-1}=\frac{P(s)}{\psi(s)}

\begin{aligned} \psi(s) &=\operatorname{det}(s I-A)=s^{n}+a_{1} s^{n-1}+\cdots+a_{n-1} s+a_{n} \\ P(s) &=\operatorname{adj}(s I-A)=P_{0} s^{n-1}+P_{1} s^{n-2}+\cdots+P_{n-2} s+P_{n-1} \end{aligned}

则

\begin{aligned} P_{0} &=I \\ P_{1} &=A P_{0}+a_{1} I \\ P_{2} &=A P_{1}+a_{2} I \\ P_{n-1} &=A P_{n-2}+a_{n-1} I \\ 0 &=A P_{n-1}+a_{n} I \end{aligned}

\begin{aligned} a_{1} &=-\operatorname{tr}(A) \\ a_{2} &=-2^{-1} \operatorname{tr}\left(A P_{1}\right) \\ a_{3} &=-3^{-1} \operatorname{tr}\left(A P_{2}\right) \\ \cdots & \cdots \\ a_{n} &=-n^{-1} \operatorname{tr}\left(A P_{n-1}\right) \end{aligned}

同时预解矩阵可以表示为

(s I-A)^{-1}=\frac{\operatorname{adj}(s I-A)}{\operatorname{det}(s I-A)}

在消除了 adj(sI - A)的所有公因子后

(s I-A)^{-1}=\frac{M(s)}{\phi(s)}

其中 \phi(s) 为 A 的最小多项式

约当规范型与可控、可观规范型

略

状态可控性

X\left(t_{0}\right) \stackrel{u(t)}{\Rightarrow} X\left(t_{\alpha}\right)

显然，输入对于状态变量可以进行控制的必要条件是其输入和状态变量有联系

状态可控性定义

\dot{X}(t)=A(t) X(t)+B(t) u(t)

如果对于非零的 \overline{x} \in \mathrm{R}^{n} 存在 u(t) \in [t_0,t_{\alpha}] 使得当x\left(t_{0}\right)=\overline{x} 时，有 x\left(t_{\alpha}\right)=0 ，则称 \bar{x} 为该系统在 [t_0,t_{\alpha}] 上的可控态

如果 \bar{x} 可以是状态空间中的任意点，则称该系统在 [t_0,t_{\alpha}] 上是状态完全可控的

对于时变系统而言，其可控性和时间区间有关，而定常系统与之无关，其在一区间上可控，则在任意时间区间上都是完全可控 

可控态的表达式

x_{+}=-\int_{t_{0}}^{t_{a}} \Phi\left(t_{0}, \tau\right) B(\tau) u(\tau) \mathrm{d} \tau

可以看出，状态是否可控，完全由 \Phi(t,t_0) 和 B(t) 决定，即 A(t),B(t) 决定。所以可控性是系统的结构性质

当系统不是完全可控时，其可控态全部构成的几何是状态空间的线性子空间，称为可控子空间

非奇异变换不改变可控性

外界干扰不影响系统的可控性

可控子空间和其正交子空间

记可控子空间 \mathbf{X}_{c}^{+}\left[t_{0}, t_{\alpha}\right] 的正交补空间为 \mathbf{X}_{c}^{-}\left[t_{0}, t_{\alpha}\right]， 状态空间可以表示为可控子空间和正交补空间的直和

\mathbf{X}=\mathbf{X}_{c}^{+}\left[t_{0}, t_{\alpha}\right] \oplus \mathbf{X}_{c}^{-}\left[t_{0}, t_{\alpha}\right]

基本判据

1. 如果 \Phi\left(t_{0}, t\right) B(t) 的行向量在 \left[t_{0}, t_{\alpha}\right] 上线性独立，则系统完全可控
2. 不可控态与 \Phi\left(t_{0}, t\right) B(t) 的所有列向量正交

可控性判据

对于线性定常系统 \Sigma(A, B)

\dot{X}(t)=A X(t)+B u(t)

在任意的有限的时间区间上其状态完全可控当且仅当下述其中一个成立：

1. 下述矩阵时非奇异的：
   W_{c}=\int_{t_{0}}^{t_{a}} e^{A\left(t_{o}-\tau\right)} B B^{T} e^{A^{\tau}\left(t_{0}-\tau\right)} \mathrm{d} \tau
2. e^{At} B 的行线性独立
3. 如下定义的可控性矩阵为行满秩
   Q_{c}=\left[ \begin{array}{lllll}{B} & {A B} & {A^{2} B} & {\cdots} & {A^{n-1} B}\end{array}\right]
4. 对A的所有特征值\lambda_i，成立
   \operatorname{rank}\left[\lambda_{i} I-A \quad B\right]=n
5. 假设非奇异矩阵将\sum(A,B)变换为\sum(J,\widetilde{B})，则
   1. A的相异特征值只有一个约当子块时，每个约当子块末行对应的\widetilde{B}的行不完全为零
   2. A的相同特征值对应的约当子块的末行所对应的\widetilde{B}是线性独立的
6. (sI - A)^{-1}B 的行是线性独立的

3称为基本判据

其中4常用于判定，称为代数判据，其在后续的结构分解中十分重要

5称为模态判据，对于一些有固定结构的矩阵进行判定很方便

性质

- 可控性矩阵Q_c的列向量张成可控子空间
  X_{c}^{+}=\left\{x_{+} | x_{+}=Q_{c} \eta, \eta \in \mathrm{R}^{n \cdot l}\right\}
- Q_c的左零空间是可控子空间的正交补空间
  X_{c}^{-}=\left\{x_{-} | x_{-}^{T} Q_{c}=0, x_{-} \in \mathrm{R}^{n}\right\}
- X_c^{+}是对A不变子空间，X_c^{-}是对A^T的不变子空间
- 可控性指标：使下式成立的最小指数 k 称为可控性指标，记为 \mu_k
  rank\left[ \begin{array}{llllll}{B} & {A B} & {A^{2} B} & {\cdots} & {A^{k-1} B}\end{array}\right] = r

状态可控性和子系统可控性

所有子系统可控，系统未必可控

对于约当标准型描述的情形：

- 每个特征根仅有一个约当子块时，所有子系统可控，系统必可控
- 一个特征根有多个约当子块时，所有子系统可控，系统未必可控

状态可观性和对偶原理

状态可观性定义

\begin{aligned} \dot{x}(t) &=A(t) x(t) \\ y(t) &=C(t) x(t) \end{aligned}

在有限时间区间内，如果对应初态 x(t_0) = \bar{x} ，有

y(t) \equiv 0, \quad t \in\left[t_{0}, t_{\alpha}\right]

则称\bar{x}为系统在该时间区间内的不可观测状态

如果不存在上述不可观测状态，则称系统状态完全可观测

\boldsymbol{X}=\boldsymbol{X}_{o}^{+}\left[t_{0}, t_{a}\right] \oplus \boldsymbol{X}_{o}^{-}\left[t_{0}, t_{\alpha}\right]

基本判据

状态完全可观测的充要条件为

C(t) \Phi\left(t, t_{0}\right) 的列向量在 [t_0,t_{\alpha}] 上线性独立

可观性判据

系统完全可观当且仅当下述一个成立：

1. 下述矩阵非奇异
   W_{o}=\int_{t_{0}}^{t_{a}} e^{A^{\tau}\left(t_{o}-\tau\right)} C^{T} C e^{A\left(t_{0}-\tau\right)} \mathrm{d} \tau
2. Ce^{At}列线性独立
3. 可观性矩阵Q_o列满秩
   Q_{o}=\left[ \begin{array}{c}{C} \\ {C A} \\ {C A^{2}} \\ {\vdots} \\ {C A^{n-1}}\end{array}\right]
4. 对于 A 的特征值 \lambda_i 成立
   \operatorname{rank} \left[ \begin{array}{c}{\lambda_{i} I-A} \\ {C}\end{array}\right]=n
5. 假设非奇异矩阵 T 将\sum(A,C) 变换为 \sum(J,\widetilde{C})，则
   1. A 的相异特征值只有一个约当子块时，每个约当子块首列对应的\widetilde{C}的列不全为零
   2. A 的相同特征值对应的约当子块的首列对应的\widetilde{C} 的列是线性独立的
6. C(s I-A)^{-1} 的列是线性独立的

讨论

1. 不可观测子空间是对 A 不变的子空间，其正交补空间是对 A^T 的不变子空间

对偶原理

系统 \sum(A(t), B(t), C(t)) 的状态完全可控性（可观性）和其对偶系统 \sum^{*}\left(-A^{T}(t), C^{T}(t), B^{T}(t)\right) 的状态完全可观性（可控性）是等价的

线性系统结构分解

可控性和可观性的频域判据

可控规范型和可观规范型




