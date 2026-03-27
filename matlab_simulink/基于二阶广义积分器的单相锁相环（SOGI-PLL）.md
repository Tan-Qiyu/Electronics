# 基于二阶广义积分器的单相锁相环（SOGI-PLL）

## 引言

在电力电子技术飞速发展的今天，并网逆变器、有源电力滤波器、不间断电源（UPS）等设备的普及，对电网同步控制的精准性、稳定性提出了极高要求。单相电网系统因缺乏天然正交信号，传统锁相环（PLL）方案难以满足复杂工况下的同步需求。而**二阶广义积分器锁相环（SOGI-PLL）** 凭借其独特的正交信号生成能力、优异的抗干扰特性，成为单相系统锁相控制的主流方案。本文将从原理、结构、实现到工程应用，全面拆解 SOGI-PLL 的核心技术。

## 一、单相锁相的核心痛点与 SOGI-PLL 的诞生

### 1.1 传统锁相方案的局限性

锁相环（PLL）的核心使命是实时检测电网电压的**相位、频率和幅值**，为电力电子设备提供精准的同步基准。但单相电网存在一个关键缺陷：**仅有单一电压信号，无法直接构建同步旋转坐标系所需的正交信号对**（α 轴、β 轴信号）。

传统单相锁相方案各有硬伤：

- **延时正交法**：通过 1/4 周期延时生成正交信号，存在固定相位延时，动态响应慢，电网频率波动时相位误差显著；
- **希尔伯特变换法**：虽能生成正交信号，但计算量大，且对谐波敏感，难以满足嵌入式系统实时性要求；
- **低通滤波法**：仅能简单滤除谐波，无法解决直流偏移问题，正交信号纯度差，锁相精度低。

### 1.2 SOGI-PLL 的核心优势

SOGI-PLL 的核心创新在于**将二阶广义积分器（SOGI）与传统同步旋转坐标系锁相环（SRF-PLL）深度融合**，完美解决单相锁相难题：

✅ 无延时生成严格 90° 正交信号，动态响应快；

✅ 自带带通滤波特性，高效抑制谐波、直流偏移；

✅ 结构简洁，计算量小，易在 DSP、MCU 等嵌入式平台实现；

✅ 电网畸变、频率波动工况下，锁相稳定性与精准度拉满。

## 二、SOGI-PLL 的核心组成与工作原理

SOGI-PLL 整体架构由**二阶广义积分器（SOGI）**、**Park 坐标变换**、**PI 调节器**和**积分器**四大模块构成，各模块协同工作，实现单相电网信号的精准锁相。

### 2.1 核心模块 1：二阶广义积分器（SOGI）—— 正交信号生成器

SOGI 是 SOGI-PLL 的 “心脏”，其本质是一种二阶带通滤波器，核心功能是从单相正弦输入信号中，同时提取同相分量（$v_α$）和严格正交的 $90°$ 滞后分量（$v_β$），为后续坐标变换提供纯净的正交信号对。（[二阶广义积分器（SOGI）从原理到实践](https://mp.weixin.qq.com/s/L4yKH-NzlJrsXpKgWKL-4w)）

#### 2.1.1 SOGI 的数学模型

连续域状态方程：
$$
\left\{\begin{array}{l}
\dot{v}_{\alpha}=k \omega_0\left(v_{i n}-v_{\alpha}\right)-\omega_0 v_{\beta} \\
\dot{v}_{\beta}=\omega_0 v_{\alpha}
\end{array}\right.
\notag
$$
其中：

- $v_{in}$：单相电网输入电压信号；
- $v_α$：SOGI 输出同相信号（$α$ 轴分量）；
- $v_β$：SOGI 输出正交信号（$β$ 轴分量，滞后$v_α$ $90°$）；
- $ω_0=2πf_0$：电网额定角频率（50Hz 对应$ω_0=314 rad/s$）；
- $k$：阻尼系数，决定系统带宽与稳定性，工程通用最优值$k=\sqrt2≈1.414$。

#### 2.1.2 SOGI 的传递函数与频率特性

对状态方程进行拉普拉斯变换，可推导得到 SOGI 的传递函数：

- 同相信号通道（带通滤波器）：
	$$
	H_{\alpha}(s)=\frac{V_{\alpha}(s)}{V_{i n}(s)}=\frac{k \omega_0 s}{s^{2}+k \omega_0 s+\omega_0^{2}}
	\notag
	$$

- 正交信号通道（低通滤波器）：
	$$
	H_{\beta}(s)=\frac{V_{\beta}(s)}{V_{i n}(s)}=\frac{k \omega_0^{2}}{s^{2}+k \omega_0 s+\omega_0^{2}}
	\notag
	$$

在电网额定频率（$s=jω_0$）处，传递函数简化为：
$$
H_α(jω_0)=1,H_β(jω_0)=−j
\notag
$$
这意味着**额定频率下，$v_β$ 与 $v_α$幅值相等、相位严格相差 90°**，完美满足单相系统正交信号需求；同时，带通滤波特性可有效滤除 50Hz 基波以外的谐波干扰，确保信号纯度。

### 2.2 核心模块 2：Park 坐标变换 —— 坐标系迁移

Park 坐标变换是连接静止坐标系（α-β 轴）与旋转坐标系（d-q 轴）的桥梁，其核心作用是将 SOGI 生成的正交信号对，转换为便于相位误差检测的 d-q 轴信号。（[帕克变换（Park Transformation）](https://mp.weixin.qq.com/s/SQvLC_iJ0EZHF6KxXmTRbQ)）

变换公式（基于 α-β 轴到 d-q 轴的变换）：
$$
\left[\begin{array}{l}
v_{d} \\
v_{q}
\end{array}\right]=\left[\begin{array}{cc}
\cos \theta & \sin \theta \\
-\sin \theta & \cos \theta
\end{array}\right]\left[\begin{array}{l}
v_{\alpha} \\
v_{\beta}
\end{array}\right]
\notag
$$
其中：

- $v_d$：d 轴分量，对应电网电压的有功分量，反映幅值信息；
- $v_q$：q 轴分量，**相位误差核心指标**，锁相的核心目标是使$v_q→0$；
- $θ$：PI 调节器输出的角频率积分值，即实时估计的电网相位。

### 2.3 核心模块 3：PI 调节器 —— 相位误差校正

PI 调节器是 SOGI-PLL 的 “调节器”，核心功能是根据 q 轴相位误差信号$v_q$，实时调整输出角频率$ω$，实现相位快速跟踪与稳定。

调节规律：
$$
\omega=\omega_{n}+\left(k_{p}+\frac{k_{i}}{s}\right) v_{q}
\notag
$$
其中：

- $ω_n$：电网额定角频率（$2π×50=314 rad/s$）；系统框图中$w_n$的作用是为了加快PI调节器的速度，当不包含$w_n$前馈输入时，PI调节器的期望理想输出为$w_n$，调节速度较有$w_n$前馈时稍慢，但最终都不会影响PI调节器输出稳定，实现锁相。
- $k_p$：PI 比例系数，决定系统动态响应速度；
- $k_i$：PI 积分系数，决定系统稳态精度，消除静差。

### 2.4 核心模块 4：积分器 —— 相位累积

积分器是 SOGI-PLL 的 “相位计算器”，核心作用是对 PI 调节器输出的角频率$ω$ 进行积分，得到实时电网相位$θ$，反馈至 Park 坐标变换模块，形成闭环控制。

后文实现中的**压控振荡器（Voltage-Controlled Oscillator, VCO）**即对应此处的积分器。

积分公式：
$$
θ=∫ωdt
\notag
$$

### 2.5 SOGI-PLL 完整闭环工作流程

1. 单相电网电压信号$v_{grid}$ 输入 SOGI 模块，SOGI 输出严格正交的$v_α$、$v_β$；
2. 正交信号$v_α$、$v_β$ 与估计相位$θ$ 经 Park 变换，得到 d 轴分量$v_d$ 和 q 轴分量$v_q$；
3. 相位误差信号$v_q$ 输入 PI 调节器，输出校正后的角频率$ω$；
4. 角频率$ω$ 经积分器积分，得到实时相位$θ$，反馈至 Park 变换模块；
5. 闭环调节下，$v_q$ 逐渐趋近于 0，相位$θ$ 精准跟踪电网实际相位，完成锁相。

## 三、SOGI-PLL 仿真（matlab/simulink）

SOGI-PLL系统框图如下所示：

![image-20260310211059369](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260310211059369.png)

根据系统框图，搭建matlab/simulink模型如下图所示：

![image-20260311015710111](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311015710111.png)

**注：**Park变换模块也可以不使用simulink库实现，可根据帕克变换公式自行搭建，如下图所示，两者效果完全一致。

![image-20260315234529721](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260315234529721.png)

**仿真分析**

### 1、二阶广义积分器（SOGI）仿真分析

$v_{in}$：单相电网输入电压信号，$v_{in}=311\cos (w_0t)，w_0=2πf_0，f_0=50$，电网频率为50HZ，电网周期为20ms。

$v_{disturb}$：谐波干扰，此处分析为0。

$v_α$：SOGI 输出同相信号（$α$ 轴分量）。

$v_β$：SOGI 输出正交信号（$β$ 轴分量，滞后$v_α$ $90°$）。

![image-20260310230402511](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260310230402511.png)

![image-20260310230751123](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260310230751123.png)

可以看出，$v_α≈v_{in}=311\cos (w_0t)$，$v_β≈311\cos (w_0t-\frac{π}{2})=311\sin (w_0t)$，满足正交性。

### 2、帕克变换仿真分析

将$v_α、v_β$代入帕克变换公式
$$
\left[\begin{array}{l}
v_{d} \\
v_{q}
\end{array}\right]=\left[\begin{array}{cc}
\cos \theta & \sin \theta \\
-\sin \theta & \cos \theta
\end{array}\right]\left[\begin{array}{l}
v_{\alpha} \\
v_{\beta}
\end{array}\right]
\notag
$$
得
$$
\begin{aligned}
\left[\begin{array}{l}
v_{d} \\
v_{q}
\end{array}\right]&=\left[\begin{array}{cc}
\cos \theta & \sin \theta \\
-\sin \theta & \cos \theta
\end{array}\right]\left[\begin{array}{l}
311\cos (w_0t) \\
311\sin (w_0t)
\end{array}\right] \\
&=\left[\begin{array}{l}
311[\cos (w_0t)\cos \theta + sin (w_0t)\sin \theta] \\
311[\sin (w_0t)\cos \theta - cos (w_0t)\sin \theta]
\end{array}\right]
\end{aligned}
\notag
$$
可以看出，当$\theta=w_0t$时，有
$$
\left[\begin{array}{l}
v_{d} \\
v_{q}
\end{array}\right]=\left[\begin{array}{c}
311 \\
0
\end{array}\right] 
\notag
$$
这也是SOGI-PLL能实现**相位锁定的核心**，通过PI调节器和积分器控制$\theta$，使$v_q=0$（此处$v_q$反映了$\theta$与$w_0t$之间的误差，将误差作为PI输入，目的是为了使误差为0），即等效于$\theta=w_0t$，实现相位锁定。

![image-20260310235203490](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260310235203490.png)

从仿真结果中可以看出，在经过1.5个电网周期（0.03s）调节后，$v_d=311$，$v_q=0$，即实现了电网相位的跟随锁定。

### 3、PI调节器仿真分析

PI控制器公式为
$$
\omega=\omega_{n}+\left(k_{p}+\frac{k_{i}}{s}\right) v_{q}
\notag
$$
仿真中$k_p=20，k_i=0.1$，当PI控制输出稳定时，$v_q=0$，$v_f=\left(k_{p}+\frac{k_{i}}{s}\right) v_{q}=0$，$\omega=\omega_{n}=2πf_0$，$f=f_0=50HZ$如下图所示。

![image-20260311000101677](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311000101677.png)

![image-20260311000350033](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311000350033.png)

![image-20260311015958910](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311015958910.png)

### 4、积分器仿真分析（VCO）

PI控制器输出的角频率$ω$经积分器后输出为$∫ωdt=ωt$，即为斜率为$ω$，随时间线性增加的斜坡函数，对应$\cos (ωt)$的相位信息，当$\omega=\omega_{n}$时，其反应了电网输入电压信号的相位信息。

由于$\cos (ωt)$为周期为$2π$的周期函数，故$\cos (ωt)=\cos (ωt+2kπ)，k=0,±1,±2...$。当$ωt∈[0,2π]$时，取$k=0$，则$ωt+2kπ∈[0,2π]$；当$ωt∈[2π,4π]$时，取$k=-1$，则$ωt+2kπ∈[0,2π]$；当$ωt∈[4π,6π]$时，取$k=-2$，则$ωt+2kπ∈[0,2π]$；以此类推。可以通过$ωt$对$2π$取余实现。

仿真结果如下图所示：

![image-20260311001101966](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311001101966.png)

![image-20260311001244308](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311001244308.png)

将积分器输出的角度信息$\theta = \omega t$代入与电网输入电压相同的$\cos$函数中，即$\cos (\omega t)$，并与电网输入电压$v_{in}=311\cos (\omega _0 t)$对比，结果如下图所示。从图中可以看出，$\cos (\omega t)$与$v_{in}$保持同相，即$\omega = \omega_0$，$\theta = \omega _0 t$，锁相环输出角度与电网输入电压角度保持一致，锁相成功。

![image-20260311001400116](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311001400116.png)

### 5、含谐波输入仿真分析

上述仿真中输入信号为50HZ的无谐波交流信号，为验证SOGI-PLL的滤波效果，分别加入幅值为30V，频率为100HZ、200HZ的谐波干扰信号，进行验证，仿真结果如下图所示。

![image-20260311012057153](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311012057153.png)

![image-20260311012236460](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311012236460.png)

从仿真结果可以看出，当加入干扰之后，输入信号幅值与相位信息均发生偏移，但SOGI-PLL锁相环仍然能滤除干扰信号，实时跟随锁定频率为50HZ的电网输入电压。主要有以下原因：1、二阶广义积分器（SOGI）本身就具有带通滤波和低通滤波的效果；2、PI调节器会根据$v_q$实时调节$\omega $，实现相位跟踪与稳定。

此时PI控制器的输出波形为：

![image-20260311013923855](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311013923855.png)

![image-20260311020550849](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311020550849.png)

经过积分器（本质也是低通滤波器，积分<->面积<->求和<->平均）后的波形为：

![image-20260311014516628](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311014516628.png)

![image-20260311014547923](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311014547923.png)

## 四、DSP软件实现

软件中的SOGI与PI调节器均使用**双线性变换离散法**进行实现，详见（[离散化方法——前向差分、后向差分、双线性变换、零阶保持ZOH](https://mp.weixin.qq.com/s/mIY8yDYozClJ46yw6NH4MA)）。

```c
//定义结构体
typedef struct{
    float32_t   u[3];       //!< AC input data buffer
    float32_t   osg_u[3];   //!< Orthogonal signal generator data buffer
    float32_t   osg_qu[3];  //!< Orthogonal signal generator quadrature data buffer
    float32_t   u_Q[2];     //!< Q-axis component
    float32_t   u_D[2];     //!< D-axis component
    float32_t   ylf[2];     //!< Loop filter data storage
    float32_t   fo;         //!< Output frequency of PLL(Hz)
    float32_t   fn;         //!< Nominal frequency (Hz)
    float32_t   wc;         //!< Center (Nominal) frequency in radians
    float32_t   theta;      //!< Angle output (0-2*pi)
    float32_t   cosine;     //!< Cosine value of the PLL angle
    float32_t   sine;       //!< Sine value of the PLL angle
    float32_t   delta_t;    //!< Inverse of the ISR rate at which module is called
    float32_t   ef2;        //!< FLL parameter
    float32_t   x3[2];      //!< FLL data storage
    float32_t   w_dash;     //!< Output frequency of PLL(radians)
    float32_t   gamma;      //!< Gamma parameter for FLL
    float32_t   k;          //!< K parameter for FLL
    SPLL_1PH_SOGI_FLL_OSG_COEFF osg_coeff; //!< Orthogonal signal generator coefficient
    SPLL_1PH_SOGI_FLL_LPF_COEFF lpf_coeff; //!< Loop filter coeffcient structure
} SPLL_1PH_SOGI_FLL;


//! \param  *spll_obj The SPLL_1PH_SOGI_FLL structure pointer
//! \param  acValue AC grid voltage in per unit (pu)
static inline void SPLL_1PH_SOGI_FLL_run(SPLL_1PH_SOGI_FLL *spll_obj, 
                                         float32_t acValue)
{
    float32_t osgx,osgy,temp;

    // Update the spll_obj->u[0] with the grid value
    spll_obj->u[0]=acValue;

    // Orthogonal Signal Generator
    spll_obj->osg_u[0]=(spll_obj->osg_coeff.osg_b0*
                       (spll_obj->u[0]-spll_obj->u[2])) +
                       (spll_obj->osg_coeff.osg_a1*spll_obj->osg_u[1]) +
                       (spll_obj->osg_coeff.osg_a2*spll_obj->osg_u[2]);

    spll_obj->osg_u[2]=spll_obj->osg_u[1];
    spll_obj->osg_u[1]=spll_obj->osg_u[0];

    spll_obj->osg_qu[0]=(spll_obj->osg_coeff.osg_qb0*spll_obj->u[0]) +
                        (spll_obj->osg_coeff.osg_qb1*spll_obj->u[1]) +
                        (spll_obj->osg_coeff.osg_qb2*spll_obj->u[2]) +
                        (spll_obj->osg_coeff.osg_a1*spll_obj->osg_qu[1]) +
                        (spll_obj->osg_coeff.osg_a2*spll_obj->osg_qu[2]);

    spll_obj->osg_qu[2]=spll_obj->osg_qu[1];
    spll_obj->osg_qu[1]=spll_obj->osg_qu[0];

    spll_obj->u[2]=spll_obj->u[1];
    spll_obj->u[1]=spll_obj->u[0];

    // Park Transform from alpha beta to d-q axis
    spll_obj->u_Q[0]=(spll_obj->cosine*spll_obj->osg_u[0]) +
                     (spll_obj->sine*spll_obj->osg_qu[0]);
    spll_obj->u_D[0]=(spll_obj->cosine*spll_obj->osg_qu[0]) -
                     (spll_obj->sine*spll_obj->osg_u[0]);

    // Loop Filter
    spll_obj->ylf[0]=spll_obj->ylf[1] +
                     (spll_obj->lpf_coeff.b0*spll_obj->u_Q[0]) +
                     (spll_obj->lpf_coeff.b1*spll_obj->u_Q[1]);
    spll_obj->ylf[1]=spll_obj->ylf[0];

    //spll_obj->ylf[0] = (spll_obj->ylf[0]>0.5)?0.5:spll_obj->ylf[0];
    //spll_obj->ylf[0] = (spll_obj->ylf[0]<-0.5)?-0.5:spll_obj->ylf[0];

    spll_obj->u_Q[1]=spll_obj->u_Q[0];

    // VCO
    spll_obj->fo=spll_obj->fn+spll_obj->ylf[0];

    spll_obj->theta=spll_obj->theta + (spll_obj->fo*spll_obj->delta_t)*
                    (float32_t)(2.0*3.1415926f);

    if(spll_obj->theta>(float32_t)(2.0*3.1415926f))
    {
        spll_obj->theta=spll_obj->theta-(float32_t)(2.0*3.1415926f);
    }

    spll_obj->sine=(float32_t)sinf(spll_obj->theta);
    spll_obj->cosine=(float32_t)cosf(spll_obj->theta);
}
```

## 五、SOGI-PLL 的工程应用与参数整定

### 4.1 典型应用场景

SOGI-PLL 凭借其优异性能，广泛应用于各类单相电力电子设备：

- **并网逆变器**：精准锁定电网相位，实现电流与电压同相，保证并网功率质量；
- **有源电力滤波器（APF）**：快速检测电网谐波，实现谐波补偿；
- **UPS 系统**：电网异常时快速切换至逆变模式，确保负载供电连续性；
- **充电桩**：精准同步电网信号，保障充电功率稳定输出。

### 4.2 关键参数整定方法

参数整定是 SOGI-PLL 稳定运行的核心，需兼顾动态响应与稳态精度，以下是工程通用整定原则：

![image-20260311021107570](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images/image-20260311021107570.png)

### 4.3 抗干扰性能验证

SOGI-PLL 的抗干扰特性是其核心优势，以下两种典型工况下的表现：

- **电网谐波干扰**：SOGI 自带带通滤波特性，可有效滤除 3、5、7 次谐波，$v_α$、$v_β$ 纯度高，锁相相位误差＜0.1°；
- **直流偏移干扰**：SOGI 对直流信号无增益，可完全抑制直流偏移，避免锁相相位缓慢漂移。

## 六、总结与展望

### 5.1 核心总结

SOGI-PLL 作为单相系统锁相控制的 “黄金方案”，核心价值在于：

1. 以简洁的 SOGI 模块解决单相正交信号缺失难题，无延时、高精度；
2. 融合 SRF-PLL 成熟架构，闭环控制逻辑清晰，易工程落地；
3. 超强抗干扰能力，适配电网畸变、频率波动等复杂工况，稳定性拉满。

从数学模型到代码实现，从原理到应用，SOGI-PLL 凭借其 “高效、简洁、可靠” 的特性，已成为并网逆变器、充电桩、UPS 等设备的标配锁相方案。

### 5.2 技术展望

随着新能源电力系统的发展，SOGI-PLL 技术仍在持续演进：

- **改进型 SOGI-PLL**：如双 SOGI-PLL、自适应 SOGI-PLL，进一步提升电网不平衡、频率快速波动工况下的锁相性能；
- **数字化优化**：结合 AI 算法、模型预测控制（MPC），实现参数自整定，降低工程调试难度；
- **集成化实现**：依托高性能 MCU、SOC，将 SOGI-PLL 与功率控制算法集成，实现小型化、高集成度系统设计。