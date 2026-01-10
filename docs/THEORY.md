# F-IMOP 理论详解 | Theory Deep Dive

[← 返回 README](../README.md)

---

<div align="center">

## 🎯 一句话理解 F-IMOP

**"给任何控制器套上一层'安全气囊'，让机器人的能量只降不升。"**

</div>

---

## 📖 目录

- [直觉解释](#直觉解释-intuitive-explanation)
- [问题背景](#问题背景)
- [数学推导](#数学推导-mathematical-derivation)
  - [动力学基础](#1-动力学基础)
  - [滤波误差](#2-滤波误差与参考动力学)
  - [Lyapunov 函数构造](#3-lyapunov-函数构造)
  - [闭式解推导](#4-闭式解推导)
- [为什么选择惯性度量](#为什么选择惯性度量)
- [参考文献](#参考文献)

---

## 直觉解释 (Intuitive Explanation)

### 🤔 问题：机器人控制为什么不安全？

想象你在开车。你的大脑（控制器）发出指令："向右打方向盘 30°"。但如果路面有冰，这个指令可能让你失控打滑。

机器人控制也面临类似问题：
- 神经网络策略可能输出"疯狂"的力矩指令
- 模型误差导致计算出的力矩不准确
- 外部干扰（如碰撞）打断原有计划

**传统解决方案**：调低增益、加各种限幅……但这些都是"事后补救"。

### 💡 F-IMOP 的思路：能量守恒

物理系统有一个美妙的性质：**能量不会凭空产生**。

F-IMOP 的核心思想是：
1. 定义一个"系统能量"函数 $V$（Lyapunov 函数）
2. 强制要求：**每一时刻，能量只能减少，不能增加**
3. 如果控制指令会导致能量增加，就"修正"它到刚好使能量减少的边界

这就像给机器人装了一个**物理层面的刹车系统**——不管上层控制器多疯狂，底层永远保证系统往稳定方向走。

### 🎨 几何直觉

在控制力矩空间中，满足"能量下降"的力矩构成一个**半空间**（由一个超平面分割）。

如果名义控制器的输出落在"安全半空间"内 → 直接放行 ✅
如果落在"危险半空间" → 投影到边界上，选择**最近的安全点** ✅

关键问题：用什么"距离"来定义"最近"？

**欧几里得距离**：$\|\tau_1 - \tau_2\|^2$ — 简单，但忽略了物理意义
**惯性度量**：$(\tau_1 - \tau_2)^T M^{-1} (\tau_1 - \tau_2)$ — 考虑了机器人的质量分布！

使用惯性度量，**大质量关节允许更大的修正**（因为它们对加速度的影响更小），这更符合物理直觉。

---

## 问题背景

考虑一个 $n$ 自由度的刚体机械臂，其动力学方程为：

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

其中：
- $q \in \mathbb{R}^n$：关节角度
- $M(q) \in \mathbb{R}^{n \times n}$：惯性矩阵（对称正定）
- $C(q, \dot{q})$：科氏力/离心力矩阵
- $g(q)$：重力向量
- $\tau$：控制力矩

**控制目标**：使 $q(t) \to q_d(t)$（轨迹跟踪）

**安全目标**：无论名义控制器 $\tau_{nom}$ 是什么，保证闭环系统全局指数稳定

---

## 数学推导 (Mathematical Derivation)

### 1. 动力学基础

刚体系统有一个关键的结构性质——**斜对称性**：

$$
x^T (\dot{M} - 2C) x = 0, \quad \forall x \in \mathbb{R}^n
$$

> 📝 **物理意义**：这反映了系统动能 $\frac{1}{2}\dot{q}^T M \dot{q}$ 的变化只来自外部做功，而非内部"凭空产生"。

### 2. 滤波误差与参考动力学

定义位置误差和滤波误差：

$$
e = q - q_d, \quad s = \dot{e} + \Lambda e
$$

其中 $\Lambda \succ 0$ 是用户设定的正定增益矩阵。

> 💡 **直觉**：$s$ 是一种"加权误差"，它把位置误差和速度误差融合在一起。如果 $s \to 0$，则 $e \to 0$（因为 $\dot{e} + \Lambda e = 0$ 是一个指数稳定的线性 ODE）。

定义参考轨迹：

$$
\dot{q}_r = \dot{q}_d - \Lambda e, \quad \ddot{q}_r = \ddot{q}_d - \Lambda \dot{e}
$$

则 $s = \dot{q} - \dot{q}_r$。定义"参考力矩"为使系统完美跟踪参考轨迹所需的力矩：

$$
\tau_r = M \ddot{q}_r + C \dot{q}_r + g
$$

### 3. Lyapunov 函数构造

选择包含惯性信息的候选函数：

$$
V = \frac{1}{2} s^T M s
$$

> 📝 这是一种"动能形式"的 Lyapunov 函数，$s$ 可以理解为"误差速度"。

对 $V$ 求时间导数：

$$
\dot{V} = s^T M \dot{s} + \frac{1}{2} s^T \dot{M} s
$$

将动力学方程代入 $M\dot{s}$：

$$
\begin{aligned}
M\dot{s} &= M(\ddot{q} - \ddot{q}_r) \\
&= \tau - C\dot{q} - g - M\ddot{q}_r \\
&= \tau - C(s + \dot{q}_r) - g - M\ddot{q}_r \\
&= \tau - Cs - \underbrace{(M\ddot{q}_r + C\dot{q}_r + g)}_{\tau_r} \\
&= \tau - Cs - \tau_r
\end{aligned}
$$

代入 $\dot{V}$：

$$
\dot{V} = s^T(\tau - Cs - \tau_r) + \frac{1}{2}s^T \dot{M} s
$$

$$
= s^T(\tau - \tau_r) - s^T C s + \frac{1}{2}s^T \dot{M} s
$$

$$
= s^T(\tau - \tau_r) + \frac{1}{2}s^T(\dot{M} - 2C)s
$$

利用**斜对称性** $s^T(\dot{M} - 2C)s = 0$：

$$
\boxed{\dot{V} = s^T \tau - s^T \tau_r}
$$

> 🎉 **重大简化**！非线性的 $C$ 项完全消失了！$\dot{V}$ 只与 $\tau$ 线性相关。

### 4. 稳定性约束

为保证指数稳定性 $V(t) \le V(0) e^{-\lambda t}$，我们强制：

$$
\dot{V} \le -\lambda V
$$

代入 $\dot{V}$ 的表达式：

$$
s^T \tau - s^T \tau_r \le -\frac{\lambda}{2} s^T M s
$$

移项得到**线性约束**：

$$
\boxed{s^T \tau \le \underbrace{s^T \tau_r - \frac{\lambda}{2} s^T M s}_{b}}
$$

这就是 F-IMOP 的安全约束！

### 5. 闭式解推导

现在我们要解决：

$$
\min_\tau \frac{1}{2}(\tau - \tau_{nom})^T M^{-1} (\tau - \tau_{nom})
$$

$$
\text{s.t.} \quad s^T \tau \le b
$$

构造拉格朗日函数：

$$
L(\tau, \mu) = \frac{1}{2}(\tau - \tau_{nom})^T M^{-1} (\tau - \tau_{nom}) + \mu(s^T \tau - b)
$$

KKT 条件：

$$
\nabla_\tau L = M^{-1}(\tau - \tau_{nom}) + \mu s = 0
$$

$$
\Rightarrow \tau = \tau_{nom} - \mu M s
$$

若约束不紧（$s^T \tau_{nom} \le b$），则 $\mu = 0$，$\tau^\star = \tau_{nom}$。

若约束紧（$s^T \tau_{nom} > b$），代入约束取等号：

$$
s^T(\tau_{nom} - \mu M s) = b
$$

$$
\mu = \frac{s^T \tau_{nom} - b}{s^T M s}
$$

最终**闭式解**：

$$
\boxed{
\tau^\star = \begin{cases}
\tau_{nom} & \text{if } s^T \tau_{nom} \le b \\
\tau_{nom} - \frac{s^T \tau_{nom} - b}{s^T M s} M s & \text{otherwise}
\end{cases}
}
$$

---

## 为什么选择惯性度量？

| 度量方式 | 优点 | 缺点 |
|---------|------|------|
| 欧几里得 $I$ | 简单 | 忽略物理结构，大惯量关节修正不足 |
| 惯性度量 $M^{-1}$ | 物理一致，消除非线性项 | 需要计算 $M$ |

使用 $M^{-1}$ 作为度量的好处：
1. **消除非线性**：$s^T(\dot{M} - 2C)s = 0$ 利用了刚体动力学的固有性质
2. **物理可解释**：修正量对应的是"加速度层面"的最小扰动
3. **数值稳定**：闭式解避免了迭代求解 QP 的不确定性

---

## 参考文献

1. Slotine, J. J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall.
2. Ames, A. D., et al. (2017). Control Barrier Functions: Theory and Applications. *IEEE CDC*.
3. 本项目论文（待发表）

---

<div align="center">

[← 返回 README](../README.md) | [查看代码实现](../fimop/controller.py)

</div>
