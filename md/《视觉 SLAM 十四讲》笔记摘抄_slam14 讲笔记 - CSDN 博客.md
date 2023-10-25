> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/ncepu_Chen/article/details/105322585?spm=1001.2014.3001.5502)

#### 《视觉 [SLAM](https://so.csdn.net/so/search?q=SLAM&spm=1001.2101.3001.7020) 十四讲》笔记摘抄

*   [ch02 初识 SLAM](#ch02_SLAM_3)
*   *   [经典视觉 SLAM 框架](#SLAM_5)
    *   [SLAM 问题的数学表述](#SLAM_44)
*   [ch03 三维空间刚体运动](#ch03__80)
*   *   [旋转矩阵](#_82)
    *   *   [点和向量, 坐标系](#_84)
        *   [坐标系间的欧氏变换](#_137)
        *   [变换矩阵与齐次坐标](#_202)
    *   [齐次坐标 (Homogeneous Coordinate) 的优势](#Homogeneous_Coordinate_252)
    *   *   *   [优势 1: 方便判断是否在直线或平面上](#1_254)
            *   [优势 2: 方便表示线线交点和点点共线](#2_266)
            *   [优势 3: 能够区分向量和点](#3_289)
            *   [优势 4: 能够表达无穷远点](#4_294)
            *   [优势 5: 能够简洁的表示变换](#5_298)
    *   [旋转向量和欧拉角](#_310)
    *   *   [旋转向量](#_312)
        *   [欧拉角](#_337)
    *   [四元数](#_354)
    *   *   [四元数的定义](#_359)
        *   [用单位四元数表示旋转](#_436)
*   [ch04 李群与李代数](#ch04__467)
*   *   [李群与李代数基础](#_469)
    *   *   [群的定义](#_492)
        *   [李代数的定义](#_502)
        *   [李代数 s o (3) \mathfrak{so}(3) so(3)](#mathfrakso3_533)
        *   [李代数 s e (3) \mathfrak{se}(3) se(3)](#mathfrakse3_566)
    *   [李群与李代数的转换关系: 指数映射和对数映射](#_601)
    *   *   [S O (3) SO(3) SO(3) 和 s o (3) \mathfrak{so}(3) so(3) 间的转换关系](#SO3mathfrakso3_603)
        *   [S E (3) SE(3) SE(3) 和 s e (3) \mathfrak{se}(3) se(3) 间的转换关系](#SE3mathfrakse3_629)
    *   [李代数求导: 引入李代数的一大动机就是方便求导优化](#__659)
    *   *   [李群乘法与李代数加法的关系](#_661)
        *   [S O (3) SO(3) SO(3) 上的李代数求导](#SO3_758)
        *   *   [李代数求导](#_772)
            *   [扰动模型 (左乘)](#_781)
        *   [S E (3) SE(3) SE(3) 上的李代数求导](#SE3_791)
*   [ch05 相机与图像](#ch05__806)
*   *   [针孔相机模型](#_808)
    *   [畸变模型](#_884)
    *   [单目相机的成像过程](#_907)
*   [ch06 非线性优化](#ch06__923)
*   *   [状态估计问题](#_925)
    *   *   [最大后验与最大似然](#_927)
        *   [最小二乘](#_990)
        *   *   [基于观测数据 z z z 的最小二乘](#z_992)
            *   [基于观测数据 z z z 和输入数据 u u u 的最小二乘](#zu_1020)
    *   [非线性最小二乘](#_1050)
    *   *   [一阶和二阶梯度法](#_1067)
        *   [高斯牛顿法](#_1096)
        *   [列文伯格 - 马夸尔特方法](#_1122)
*   [ch07 视觉里程计 01](#ch07_01_1183)
*   *   [特征点匹配](#_1185)
    *   *   [特征点](#_1187)
    *   [根据特征点匹配计算相机运动](#_1193)
    *   *   [2D-2D 匹配: 对极几何](#2D2D__1205)
        *   *   [对极约束](#_1207)
            *   [本质矩阵 E E E 的求解](#E_1238)
            *   [对极几何的讨论](#_1284)
        *   [3D-2D 匹配: PnP(Perspective-n-Point)](#3D2D_PnPPerspectivenPoint_1310)
        *   *   [直接线性变换 (DLT): 先求解相机位姿, 再求解空间点位置](#DLT__1322)
            *   [P3P: 先求解空间点位置, 再求解相机位姿](#P3P__1370)
            *   [Bundle Adjustment: 最小化重投影误差, 同时求解空间点位置和相机位姿](#Bundle_Adjustment__1401)
        *   [3D-3D 匹配: ICP](#3D3D_ICP_1485)
        *   *   [SVD 方法](#SVD_1505)
            *   [非线性优化方法](#_1542)

ch02 初识 SLAM
------------

### 经典视觉 SLAM 框架

![](https://img-blog.csdnimg.cn/20200405095018389.png)

视觉 SLAM 流程包括以下步骤:

1.  **传感器信息读取**: 在视觉 SLAM 中主要为相机图像信息的读取和预处理. 如果是在机器人中, 还可能有码盘、惯性传感器等信息的读取和同步.
    
2.  **视觉里程计** (Visual Odometry,VO): 视觉里程计的任务是估算相邻图像间相机的运动, 以及局部地图的样子. VO 又称为前端 (Front End).
    
    视觉里程计不可避免地会出现**累积漂移** (Accumulating Drift) 问题.
    
3.  **后端优化** (Optimization): 后端接受不同时刻视觉里程计测量的相机位姿, 以及回环检测的信息, 对它们进行优化, 得到全局一致的轨迹和地图. 由于接在 VO 之后, 又称为后端 (Back End).
    
    在视觉 SLAM 中, 前端和计算机视觉研究领域更为相关, 比如图像的特征提取与匹配等, 后端则主要是滤波与非线性优化算法.
    
4.  **回环检测** (Loop Closing): 回环检测判断机器人是否到达过先前的位置. 如果检测到回环, 它会把信息提供给后端进行处理.
    
5.  **建图** (Mapping): 它根据估计的轨迹, 建立与任务要求对应的地图.
    
    地图的形式包括**度量地图** (精确表示地图物体的位置关系) 与**拓扑地图** (更强调地图元素之间的关  
    系) 两种.
    

### SLAM 问题的数学表述

“小萝卜携带着传感器在环境中运动”, 由如下两件事情描述:

1.  什么是**运动** ? 我们要考虑从 k − 1 k-1 k−1 时刻到 k k k 时刻, 小萝卜的位置 x x x 是如何变化的.
    
    运动方程:
    
    x k = f ( x k − 1 , u k , w k ) x_k = f(x_{k-1}, u_k, w_k) xk​=f(xk−1​,uk​,wk​)
    
    *   x k , x k − 1 x_k, x_{k-1} xk​,xk−1​表示小萝卜在 k k k 和 k − 1 k-1 k−1 时刻的位置
    *   u k u_k uk​表示运动传感器的读数 (有时也叫**输入**)
    *   w k w_k wk​表示噪声
2.  什么是**观测** ? 假设小萝卜在 k k k 时刻于 x k x_k xk​处探测到了某一个路标 y j y_j yj​, 我们要考虑这件事情是如何用数学语言来描述的.
    
    观测方程:
    
    z k , j = h ( y j , x k , v k , j ) z_{k,j} = h(y_j, x_k, v_{k,j}) zk,j​=h(yj​,xk​,vk,j​)
    
    *   z k , j z_{k,j} zk,j​表示小萝卜在 x k x_k xk​位置上看到路标点 y j y_j yj​, 产生的观测数据
    *   y j y_j yj​表示第 j j j 个路标点
    *   v k , j v_{k,j} vk,j​表示噪声

这两个方程描述了最基本的 SLAM 问题: 当知道运动测量的读数 u u u , 以及传感器的读数 z z z 时, 如何求解定位问题 (估计 x x x ) 和建图问题 (估计 y y y)? 这时, 我们就把 SLAM 问题建模成了一个**状态估计问题**: 如何通过带有噪声的测量数据, 估计内部的、隐藏着的状态变量?

ch03 三维空间刚体运动
-------------

### 旋转矩阵

#### 点和向量, 坐标系

1.  向量 a a a 在线性空间的基 [ e 1 , e 2 , e 3 ] [e_1, e_2, e_3] [e1​,e2​,e3​] 下的坐标为 [ a 1 , a 2 , a 3 ] T [a_1, a_2, a_3]^T [a1​,a2​,a3​]T.
    
    a = [ e 1 , e 2 , e 3 ] [ a 1 a 2 a 3 ] = a 1 e 1 + a 2 e 2 + a 3 e 3 a = [e_1, e_2, e_3] \left[
    
    $$\begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array}$$
    
    \right] = a_1e_1 + a_2e_2 + a_3e_3 a=[e1​,e2​,e3​]⎣⎡​a1​a2​a3​​⎦⎤​=a1​e1​+a2​e2​+a3​e3​
    
2.  向量的内积与外积
    
    *   向量的内积: 描述向量间的投影关系  
        a ⋅ b = a T b = ∑ i = 1 3 a i b i = ∣ a ∣   ∣ b ∣ cos ⁡ ⟨ a , b ⟩ a \cdot b = a^T b = \sum_{i=1}^3 a_ib_i = |a|\,|b| \cos \langle a,b \rangle a⋅b=aTb=i=1∑3​ai​bi​=∣a∣∣b∣cos⟨a,b⟩
        
    *   向量的外积: 描述向量的旋转  
        a × b = [ i j k a 1 a 2 a 3 b 1 b 2 b 3 ] = [ a 2 b 3 − a 3 b 2 a 3 b 1 − a 1 b 3 a 1 b 2 − a 2 b 1 ] = [ 0 − a 3 a 2 a 3 0 − a 1 − a 2 a 1 0 ] b ≜ a ∧ b a \times b = \left[
        
        $$\begin{array}{ccc} i & j & k \\ a_1 & a_2 & a_3 \\ b_1 & b_2 & b_3 \\ \end{array}$$
        
        \right] = \left[
        
        $$\begin{array}{c} a_2b_3 - a_3b_2 \\ a_3b_1 - a_1b_3 \\ a_1b_2 - a_2b_1 \end{array}$$
        
        \right] = \left[
        
        $$\begin{array}{ccc} 0 & -a_3 & a_2\\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{array}$$
        
        \right] b \triangleq a ^\wedge b a×b=⎣⎡​ia1​b1​​ja2​b2​​ka3​b3​​⎦⎤​=⎣⎡​a2​b3​−a3​b2​a3​b1​−a1​b3​a1​b2​−a2​b1​​⎦⎤​=⎣⎡​0a3​−a2​​−a3​0a1​​a2​−a1​0​⎦⎤​b≜a∧b
        
        其中 a ∧ a^\wedge a∧表示 a a a 的反对称矩阵  
        a ∧ = [ 0 − a 3 a 2 a 3 0 − a 1 − a 2 a 1 0 ] a ^\wedge = \left[
        
        $$\begin{array}{ccc} 0 & -a_3 & a_2\\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{array}$$
        
        \right] a∧=⎣⎡​0a3​−a2​​−a3​0a1​​a2​−a1​0​⎦⎤​
        

#### 坐标系间的欧氏变换

1.  欧式变换:
    
    在欧式变换前后的两个坐标系下, 同一个向量的模长和方向不发生改变, 是为欧式变换.
    
    一个欧式变换由一个旋转和一个平移组成.
    
2.  旋转矩阵 R R R:
    
    *   旋转矩阵 R R R 的推导:
        
        设单位正交基 [ e 1 , e 2 , e 3 ] [e_1, e_2, e_3] [e1​,e2​,e3​] 经过一次旋转变成了 [ e 1 ′ , e 2 ′ , e 3 ′ ] [e_1', e_2', e_3'] [e1′​,e2′​,e3′​], 对于同一个向量 a a a, 在两个坐标系下的坐标分别为 [ a 1 , a 2 , a 3 ] T [a_1, a_2, a_3]^T [a1​,a2​,a3​]T 和 [ a 1 ′ , a 2 ′ , a 3 ′ ] T [a_1', a_2', a_3']^T [a1′​,a2′​,a3′​]T. 根据坐标的定义:  
        [ e 1 , e 2 , e 3 ] [ a 1 a 2 a 3 ] = [ e 1 ′ , e 2 ′ , e 3 ′ ] [ a 1 ′ a 2 ′ a 3 ′ ] [e_1, e_2, e_3] \left[
        
        $$\begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array}$$
        
        \right] = [e_1', e_2', e_3'] \left[
        
        $$\begin{array}{c} a_1' \\ a_2' \\ a_3' \end{array}$$
        
        \right] [e1​,e2​,e3​]⎣⎡​a1​a2​a3​​⎦⎤​=[e1′​,e2′​,e3′​]⎣⎡​a1′​a2′​a3′​​⎦⎤​
        
        等式左右两边同时左乘 [ e 1 T , e 2 T , e 3 T ] T [e_1^T, e_2^T, e_3^T]^T [e1T​,e2T​,e3T​]T, 得到  
        [ a 1 a 2 a 3 ] = [ e 1 T e 1 ′ e 1 T e 2 ′ e 1 T e 3 ′ e 2 T e 1 ′ e 2 T e 2 ′ e 2 T e 3 ′ e 3 T e 1 ′ e 3 T e 2 ′ e 3 T e 3 ′ ] [ a 1 ′ a 2 ′ a 3 ′ ] ≜ R a ′ \left[
        
        $$\begin{array}{c} a_1 \\ a_2 \\ a_3 \end{array}$$
        
        \right] = \left[
        
        $$\begin{array}{ccc} e_1^Te_1' & e_1^Te_2' & e_1^Te_3' \\ e_2^Te_1' & e_2^Te_2' & e_2^Te_3' \\ e_3^Te_1' & e_3^Te_2' & e_3^Te_3' \end{array}$$
        
        \right] \left[
        
        $$\begin{array}{c} a_1' \\ a_2' \\ a_3' \end{array}$$
        
        \right] \triangleq R a' ⎣⎡​a1​a2​a3​​⎦⎤​=⎣⎡​e1T​e1′​e2T​e1′​e3T​e1′​​e1T​e2′​e2T​e2′​e3T​e2′​​e1T​e3′​e2T​e3′​e3T​e3′​​⎦⎤​⎣⎡​a1′​a2′​a3′​​⎦⎤​≜Ra′
        
        矩阵 R R R 描述了旋转, 称为**旋转矩阵**.
        
    *   旋转矩阵 R R R 的性质
        
        1.  旋转矩阵是**行列式为 1 的正交矩阵**, 任何行列式为 1 的正交矩阵也是一个旋转矩阵. 所有旋转矩阵构成特殊正交群 S O SO SO:
        
        S O (n) = { R ∈ R n × n ∣ R R T = I , det ⁡ ( R ) = 1 } SO(n) = \{ R \in \mathbb{R}^{n \times n} | RR^T = I, \det(R)=1 \} SO(n)={R∈Rn×n∣RRT=I,det(R)=1}
        
        2.  旋转矩阵是正交矩阵 (其转置等于其逆), 旋转矩阵的逆 R − 1 R^{-1} R−1(即转置 R T R^T RT) 描述了一个相反的旋转.
3.  欧式变换的向量表示:
    
    世界坐标系中的向量 a a a, 经过一次旋转 (用旋转矩阵 R R R 描述) 和一次平移 (用平移向量 t t t 描述) 后, 得到了 a ′ a' a′:  
    a ′ = R a + t a' = Ra + t a′=Ra+t
    

#### 变换矩阵与齐次坐标

1.  变换矩阵 T T T:
    
    在三维向量的末尾添加 1, 构成的四维向量称为**齐次坐标**. 将旋转和平移写入**变换矩阵** T T T 中, 得到:
    
    [ a ′ 1 ] = [ R t 0 1 ] [ a 1 ] ≜ T [ a 1 ] \left[
    
    $$\begin{array}{c} a' \\ 1 \end{array}$$
    
    \right] = \left[
    
    $$\begin{array}{cc} R & t \\ 0 & 1 \end{array}$$
    
    \right] \left[
    
    $$\begin{array}{c} a \\ 1 \end{array}$$
    
    \right] \triangleq T \left[
    
    $$\begin{array}{c} a \\ 1 \end{array}$$
    
    \right] [a′1​]=[R0​t1​][a1​]≜T[a1​]  
    齐次坐标的意义在于**将欧式变换表示为线性关系**.
    
2.  变换矩阵 T T T 的性质:
    
    1.  变换矩阵 T T T 构成特殊欧式群 S E SE SE  
        S E (3) = { T = [ R t 0 1 ] ∈ R 4 × 4 ∣ R ∈ S O ( 3 ) , t ∈ R 3 } SE(3) = \left\{ T = \left[
        
        $$\begin{array}{cc} R & t \\ 0 & 1 \end{array}$$
        
        \right] \in \mathbb{R}^{4\times4} | R \in SO(3), t \in \mathbb{R}^3 \right\} SE(3)={T=[R0​t1​]∈R4×4∣R∈SO(3),t∈R3}
        
    2.  变换矩阵的逆表示一个反向的欧式变换  
        T − 1 = [ R T − R T t 0 1 ] T^{-1} = \left[
        
        $$\begin{array}{cc} R^T & -R^Tt \\ 0 & 1 \end{array}$$
        
        \right] T−1=[RT0​−RTt1​]
        

### 齐次坐标 (Homogeneous Coordinate) 的优势

##### 优势 1: 方便判断是否在直线或平面上

若点 p = ( x , y ) p=(x,y) p=(x,y) 在直线 l = ( a , b , c ) l=(a,b,c) l=(a,b,c) 上, 则有:  
a x + b y + c = [ a , b , c ] T ⋅ [ x , y , 1 ] = l T ⋅ p ′ = 0 ax+by+c = [a,b,c]^T \cdot [x,y,1] = l^T \cdot p' = 0 ax+by+c=[a,b,c]T⋅[x,y,1]=lT⋅p′=0

若点 p = ( x , y , z ) p=(x,y,z) p=(x,y,z) 在平面 A = ( a , b , c , d ) A=(a,b,c,d) A=(a,b,c,d) 上, 则有:  
a x + b y + c z + d = [ a , b , c , d ] T ⋅ [ x , y , z , 1 ] = A T ⋅ p ′ = 0 ax+by+cz+d = [a,b,c,d]^T \cdot [x,y,z,1] = A^T \cdot p' = 0 ax+by+cz+d=[a,b,c,d]T⋅[x,y,z,1]=AT⋅p′=0

##### 优势 2: 方便表示线线交点和点点共线

在齐次坐标下,

1.  可以用两个点 p p p, q q q 的齐次坐标叉乘结果表示它们的共线 l l l.
2.  可以用两条直线 l l l, m m m 的齐次坐标叉乘结果表示它们的交点 x x x.

![](https://img-blog.csdnimg.cn/20200509211458423.png)

这里利用叉乘的性质: 叉乘结果与两个运算向量都垂直:

*   性质 1 的证明:  
    l T ⋅ p = ( p × q ) ⋅ p = 0 l T ⋅ q = ( p × q ) ⋅ q = 0 l^T \cdot p = (p \times q) \cdot p = 0 \\ l^T \cdot q = (p \times q) \cdot q = 0 lT⋅p=(p×q)⋅p=0lT⋅q=(p×q)⋅q=0
    
*   性质 2 的证明:  
    l T ⋅ p = l T ⋅ ( l × m ) = 0 m T ⋅ p = m T ⋅ ( l × m ) = 0 l^T \cdot p = l^T \cdot (l \times m) = 0 \\ m^T \cdot p = m^T \cdot (l \times m) = 0 lT⋅p=lT⋅(l×m)=0mT⋅p=mT⋅(l×m)=0
    

##### 优势 3: 能够区分向量和点

*   点 ( x , y , z ) (x,y,z) (x,y,z) 的齐次坐标为 ( x , y , z , 1 ) (x,y,z,1) (x,y,z,1)
*   向量 ( x , y , z ) (x,y,z) (x,y,z) 的齐次坐标为 ( x , y , z , 0 ) (x,y,z,0) (x,y,z,0)

##### 优势 4: 能够表达无穷远点

对于平行直线 l = ( a , b , c ) l=(a,b,c) l=(a,b,c) 和 m = ( a , b , d ) m=(a,b,d) m=(a,b,d), 求取其交点的齐次坐标 x = l × m = ( k b , − k a , 0 ) x=l \times m=(kb, -ka, 0) x=l×m=(kb,−ka,0), 将其转为非齐次坐标, 得到 x = ( k b / 0 , − k a / 0 ) = ( inf ⁡ , − inf ⁡ ) x = (kb/0, -ka/0) = (\inf, -\inf) x=(kb/0,−ka/0)=(inf,−inf), 这表示无穷远点.

##### 优势 5: 能够简洁的表示变换

使用齐次坐标, 可以将加法运算转化为乘法运算.

<table><thead><tr><th>变换形式</th><th>图形示意</th><th>数学变换</th><th>MATLAB 函数</th></tr></thead><tbody><tr><td>位移 (Translation)</td><td><img class="" src="https://img-blog.csdnimg.cn/2019111813242824.png"></td><td>[ x ′ y ′ 1 ] = [ 1 0 t x 0 1 t y 0 0 1 ] ∗ [ x y 1 ] \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-17-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><msup><mi>x</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">x′y′1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><msup><mi>x</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-17">\begin{array}{c} x' \\ y' \\ 1 \end{array}</script>\right] =\left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-18-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mn>1</mn></mtd><mtd><mn>0</mn></mtd><mtd><msub><mi>t</mi><mi>x</mi></msub></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd><mtd><msub><mi>t</mi><mi>y</mi></msub></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">100010txty1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mn>1</mn></mtd><mtd><mn>0</mn></mtd><mtd><msub><mi>t</mi><mi>x</mi></msub></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd><mtd><msub><mi>t</mi><mi>y</mi></msub></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-18">\begin{array}{c} 1 & 0 & t_x \\ 0 & 1 & t_y \\ 0 & 0 & 1 \end{array}</script>\right] * \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-19-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">xy1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-19">\begin{array}{c} x \\ y \\ 1 \end{array}</script>\right] ⎣⎡​x′y′1​⎦⎤​=⎣⎡​100​010​tx​ty​1​⎦⎤​∗⎣⎡​xy1​⎦⎤​</span></span></span></td><td><code>imtranslate()</code></td></tr><tr><td>缩放 (Scale)</td><td><img class="" src="https://img-blog.csdnimg.cn/20191118132831770.png"></td><td>[ x ′ y ′ 1 ] = [ s x 0 0 0 s y 0 0 0 1 ] ∗ [ x y 1 ] \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-20-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><msup><mi>x</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">x′y′1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><msup><mi>x</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-20">\begin{array}{c} x' \\ y' \\ 1 \end{array}</script>\right] =\left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-21-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><msub><mi>s</mi><mi>x</mi></msub></mtd><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><msub><mi>s</mi><mi>y</mi></msub></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">sx000sy0001</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><msub><mi>s</mi><mi>x</mi></msub></mtd><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><msub><mi>s</mi><mi>y</mi></msub></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-21">\begin{array}{c} s_x & 0 & 0 \\ 0 & s_y & 0 \\ 0 & 0 & 1 \end{array}</script>\right] * \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-22-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">xy1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-22">\begin{array}{c} x \\ y \\ 1 \end{array}</script>\right] ⎣⎡​x′y′1​⎦⎤​=⎣⎡​sx​00​0sy​0​001​⎦⎤​∗⎣⎡​xy1​⎦⎤​</span></span></span></td><td><code>imresize()</code></td></tr><tr><td>错切 (Shear)</td><td><img class="" src="https://img-blog.csdnimg.cn/20191118133317854.png"></td><td>[ x ′ y ′ 1 ] = [ 1 h x 0 h y 1 0 0 0 1 ] ∗ [ x y 1 ] \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-23-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><msup><mi>x</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">x′y′1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><msup><mi>x</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-23">\begin{array}{c} x' \\ y' \\ 1 \end{array}</script>\right] =\left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-24-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mn>1</mn></mtd><mtd><msub><mi>h</mi><mi>x</mi></msub></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><msub><mi>h</mi><mi>y</mi></msub></mtd><mtd><mn>1</mn></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">1hy0hx10001</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mn>1</mn></mtd><mtd><msub><mi>h</mi><mi>x</mi></msub></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><msub><mi>h</mi><mi>y</mi></msub></mtd><mtd><mn>1</mn></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-24">\begin{array}{c} 1 & h_x & 0 \\ h_y & 1 & 0 \\ 0 & 0 & 1 \end{array}</script>\right] * \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-25-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">xy1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-25">\begin{array}{c} x \\ y \\ 1 \end{array}</script>\right] ⎣⎡​x′y′1​⎦⎤​=⎣⎡​1hy​0​hx​10​001​⎦⎤​∗⎣⎡​xy1​⎦⎤​</span></span></span></td><td></td></tr><tr><td>旋转 (Rotate)</td><td><img class="" src="https://img-blog.csdnimg.cn/20191118134626274.png"></td><td>[ x ′ y ′ 1 ] = [ cos ⁡ θ sin ⁡ θ 0 − sin ⁡ θ cos ⁡ θ 0 0 0 1 ] ∗ [ x y 1 ] \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-26-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><msup><mi>x</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>&amp;#x2032;</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">x′y′1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><msup><mi>x</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><msup><mi>y</mi><mo>′</mo></msup></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-26">\begin{array}{c} x' \\ y' \\ 1 \end{array}</script>\right] =\left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-27-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mi>cos</mi><mo>&amp;#x2061;</mo><mi>&amp;#x03B8;</mi></mtd><mtd><mi>sin</mi><mo>&amp;#x2061;</mo><mi>&amp;#x03B8;</mi></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mo>&amp;#x2212;</mo><mi>sin</mi><mo>&amp;#x2061;</mo><mi>&amp;#x03B8;</mi></mtd><mtd><mi>cos</mi><mo>&amp;#x2061;</mo><mi>&amp;#x03B8;</mi></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">cosθ−sinθ0sinθcosθ0001</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mi>cos</mi><mo>⁡</mo><mi>θ</mi></mtd><mtd><mi>sin</mi><mo>⁡</mo><mi>θ</mi></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mo>−</mo><mi>sin</mi><mo>⁡</mo><mi>θ</mi></mtd><mtd><mi>cos</mi><mo>⁡</mo><mi>θ</mi></mtd><mtd><mn>0</mn></mtd></mtr><mtr><mtd><mn>0</mn></mtd><mtd><mn>0</mn></mtd><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-27">\begin{array}{c} \cos\theta & \sin\theta & 0 \\ -\sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{array}</script>\right] * \left[<span class="MathJax MathJax_FullWidth" id="MathJax-Element-28-Frame" tabindex="0" data-mathml="<math xmlns=&quot;http://www.w3.org/1998/Math/MathML&quot; display=&quot;block&quot;><mtable rowspacing=&quot;4pt&quot; columnspacing=&quot;1em&quot;><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math>" role="presentation" style="position: relative;"><nobr aria-hidden="true">xy1</nobr><math xmlns="http://www.w3.org/1998/Math/MathML" display="block"><mtable rowspacing="4pt" columnspacing="1em"><mtr><mtd><mi>x</mi></mtd></mtr><mtr><mtd><mi>y</mi></mtd></mtr><mtr><mtd><mn>1</mn></mtd></mtr></mtable></math><script type="math/tex; mode=display" id="MathJax-Element-28">\begin{array}{c} x \\ y \\ 1 \end{array}</script>\right] ⎣⎡​x′y′1​⎦⎤​=⎣⎡​cosθ−sinθ0​sinθcosθ0​001​⎦⎤​∗⎣⎡​xy1​⎦⎤​</span></span></span></td><td><code>imrotate()</code></td></tr></tbody></table>

### 旋转向量和欧拉角

#### 旋转向量

*   旋转矩阵的缺点:
    
    1.  旋转矩阵有 9 个量, 但一次旋转只有 3 个自由度, 这种表达方式是冗余的.
    2.  旋转矩阵自带约束 (必须是行列式为 1 的正交矩阵), 这些约束会给估计和优化带来困难.
*   旋转向量: 任意旋转都可以用一个**旋转轴**和一个**旋转角** 来刻画. 于是, 我们可以使用一个向量, 其**方向表示旋转轴**而**长度表示旋转角**. 这种向量称为**旋转向量** (或**轴角**,Axis-Angle).
    
    假设有一个旋转轴为 n n n, 角度为 θ \theta θ的旋转, 其对应的旋转向量为 θ n \theta n θn.
    
*   旋转向量和旋转矩阵之间的转换:
    
    设旋转向量 R R R 表示一个绕单位向量 n n n, 角度为 θ θ θ的旋转.
    
    *   旋转向量到旋转矩阵:  
        R = cos ⁡ θ I + ( 1 − cos ⁡ θ ) n n T + sin ⁡ θ   n ∧ R = \cos\theta I + (1-\cos\theta) n n^T + \sin\theta \, n^\wedge R=cosθI+(1−cosθ)nnT+sinθn∧
        
    *   旋转矩阵到旋转向量:
        
        *   旋转角 θ = arccos ⁡ ( t r (R) − 1 2 ) \theta = \arccos \left( \frac{tr(R)-1}{2} \right) θ=arccos(2tr(R)−1​)
        *   旋转轴 n n n 是矩阵 R R R 特征值 1 对应的特征向量

#### 欧拉角

*   欧拉角将一次旋转分解成 **3 个分离的转角**. 常用的一种 ZYX 转角将任意旋转分解成以下 3 个轴上的转角:
    
    1.  绕物体的 Z Z Z 轴旋转, 得到偏航角 yaw
    2.  绕**旋转之后**的 Y Y Y 轴旋转, 得到俯仰角 pitch
    3.  绕**旋转之后**的 X X X 轴旋转, 得到滚转角 roll
*   欧拉角的一个重大缺点是**万向锁问题** (**奇异性问题**): 在俯仰角为 $\pm$90° 时, 第一次旋转与第三次旋转将使用同一个轴, 使得系统丢失了一个自由度 (由 3 次旋转变成了 2 次旋转).
    

### 四元数

为什么需要四元数: 对于三维旋转, 找不到**不带奇异性的三维向量描述方式**. 因此引入四元数.  
四元数是一种**扩展的复数**, **既是紧凑的, 也没有奇异性**.

#### 四元数的定义

1.  四元数的定义
    
    一个四元数 q q q 拥有一个实部和三个虚部  
    q = q 0 + q 1 i + q 2 j + q 3 k q = q_0 + q_1 i + q_2 j + q_3 k q=q0​+q1​i+q2​j+q3​k
    
    其中 i i i, j j j, k k k, 为四元数的 3 个虚部, 它们满足以下关系式 (**自己和自己的运算像复数, 自己和别人的运算像叉乘**):  
    { i 2 = j 2 = k 2 = − 1 i j = k , j i = − k j k = i , k j = − i k i = j , i k = − j \left\{
    
    $$\begin{aligned} & i^2 = j^2 = k^2 = -1 \\ & ij = k, ji=-k \\ & jk = i, kj=-i \\ & ki = j, ik=-j \end{aligned}$$
    
     \right. ⎩⎪⎪⎪⎪⎨⎪⎪⎪⎪⎧​​i2=j2=k2=−1ij=k,ji=−kjk=i,kj=−iki=j,ik=−j​
    
    也可以用一个标量和一个向量来表达四元数:  
    q = [ s , v ] , s = q 0 ∈ R v = [ q 1 , q 2 , q 3 ] T ∈ R 3 q = [s, v], \quad s=q_0\in\mathbb{R} \quad v=[q_1, q_2, q_3]^T \in \mathbb{R}^3 q=[s,v],s=q0​∈Rv=[q1​,q2​,q3​]T∈R3
    
    s s s 为四元数的实部, v v v 为四元数的虚部. 有**实四元数**和**虚四元数**的概念.
    
2.  四元数与旋转角度的关系:
    
    *   在二维情况下, 任意一个旋转都可以用**单位**复数来描述, 乘 i i i 就是绕 i i i 轴旋转 90°.
    *   在三维情况下, 任意一个旋转都可以用**单位**四元数来描述, 乘 i i i 就是绕 i i i 轴旋转 180°.
3.  单位四元数和旋转向量之间的转换:
    
    设单位四元数 q q q 表示一个绕单位向量 n = [ n x , n y , n z ] T n =[n_x,n_y,n_z]^T n=[nx​,ny​,nz​]T, 角度为 θ θ θ的旋转.
    
    *   从旋转向量到单位四元数:
    
    q = [ cos ⁡ ( θ 2 ) , n sin ⁡ ( θ 2 ) ] T = [ cos ⁡ ( θ 2 ) , n x sin ⁡ ( θ 2 ) , n y sin ⁡ ( θ 2 ) , n z sin ⁡ ( θ 2 ) ] T q = \left[\cos(\frac{\theta}{2}), n\sin(\frac{\theta}{2}) \right]^T= \left[ \cos(\frac{\theta}{2}), n_x\sin(\frac{\theta}{2}), n_y\sin(\frac{\theta}{2}), n_z\sin(\frac{\theta}{2}) \right]^T q=[cos(2θ​),nsin(2θ​)]T=[cos(2θ​),nx​sin(2θ​),ny​sin(2θ​),nz​sin(2θ​)]T
    
    *   从单位四元数到旋转向量:  
        { θ = 2 arccos ⁡ q 0 [ n x , n y , n z ] = [ q 1 , q 2 , q 3 ] T / sin ⁡ θ 2 \left\{ 
        
        $$\begin{aligned} & \theta = 2 \arccos{q_0}\\ & [n_x,n_y,n_z] = [q_1, q_2, q_3]^T / \sin{\frac{\theta}{2}} \end{aligned}$$
        
         \right. ⎩⎨⎧​​θ=2arccosq0​[nx​,ny​,nz​]=[q1​,q2​,q3​]T/sin2θ​​

#### 用单位四元数表示旋转

给定一个空间三维点 p = [ x , y , z ] ∈ R 3 p=[x,y,z]\in \R^3 p=[x,y,z]∈R3, 以及一个由轴角 n n n, θ θ θ指定的旋转, 三维点 p p p 经过旋转后变为 p ′ p′ p′. 如何使用单位四元数 q q q 表达旋转?

1.  把三维空间点用一个虚四元数 p p p 表示:  
    p = [ 0 , x , y , z ] = [ 0 , v ] p = [0, x, y, z] = [0, v] p=[0,x,y,z]=[0,v]
    
2.  把旋转用单位四元数 q q q 表示:  
    q = [ cos ⁡ θ 2 , n sin ⁡ θ 2 ] q = [\cos{\frac{\theta}{2}}, n\sin{\frac{\theta}{2}} ] q=[cos2θ​,nsin2θ​]
    
3.  旋转后的点 p ′ p' p′可表示为:  
    p ′ = q p q − 1 p' = qpq^{-1} p′=qpq−1
    

这样得到的点 p ′ p' p′仍为一个纯虚四元数, 其虚部的 3 个分量表示旋转后 3D 点的坐标.

> 只有**单位**四元数才能表示旋转, 因此在程序中创建四元数后, 要记得调用`normalize()`以将其单位化

ch04 李群与李代数
-----------

### 李群与李代数基础

旋转矩阵构成特殊正交群 S O (3) SO(3) SO(3), 变换矩阵构成了特殊欧氏群 S E (3) SE(3) SE(3).  
S O (3) = { R ∈ R 3 × 3 ∣ R R T = I , det ⁡ ( R ) = 1 } S E ( 3 ) = { T = [ R t 0 T 1 ] ∈ R 4 × 4 ∣ R ∈ S O ( 3 ) , t ∈ R 3 }

$$\begin{aligned} SO(3) &= \left\{ R \in \mathbb{R}^{3\times 3} | RR^T=I, \det(R)=1 \right\} \\ SE(3) &= \left\{ T = \left[\begin{array}{cc} R & t \\ 0^T & 1 \end{array}\right] \in \mathbb{R}^{4\times 4} | R \in SO(3), t \in \mathbb{R}^3 \right\} \end{aligned}$$

SO(3)SE(3)​={R∈R3×3∣RRT=I,det(R)=1}={T=[R0T​t1​]∈R4×4∣R∈SO(3),t∈R3}​

#### 群的定义

*   群 (Group) 是**一种集合**加上**一种运算**的代数结构. 把集合记作 A A A, 运算记作 ⋅ \cdot ⋅ , 那么群可以记作 G = ( A , ⋅ ) G =(A,\cdot) G=(A,⋅). 群要求这个运算满足如下条件 (**封结幺逆**):
    
    1.  封闭性: ∀ a 1 , a 2 ∈ A , a 1 ⋅ a 2 ∈ A \forall{a_1, a_2} \in A, \quad a_1 \cdot a_2 \in A ∀a1​,a2​∈A,a1​⋅a2​∈A.
    2.  结合律: ∀ a 1 , a 2 , a 3 ∈ A , ( a 1 ⋅ a 2 ) ⋅ a 3 = a 1 ⋅ ( a 2 ⋅ a 3 ) \forall{a_1, a_2, a_3} \in A, \quad (a_1 \cdot a_2) \cdot a_3 = a_1 \cdot (a_2 \cdot a_3 ) ∀a1​,a2​,a3​∈A,(a1​⋅a2​)⋅a3​=a1​⋅(a2​⋅a3​)
    3.  幺元: ∃ a 0 ∈ A , s . t . ∀ a ∈ A , a 0 ⋅ a = a ⋅ a 0 = a \exists{a_0} \in A, \quad \mathrm{s.t.} \quad \forall a \in A, \quad a_0\cdot a = a\cdot a_0 = a ∃a0​∈A,s.t.∀a∈A,a0​⋅a=a⋅a0​=a
    4.  逆: ∀ a ∈ A , ∃ a − 1 ∈ A , s . t . a ⋅ a − 1 = a 0 \forall a \in A, \quad \exists{a^{-1}} \in A, \quad \mathrm{s.t.} a\cdot a^{-1}=a_0 ∀a∈A,∃a−1∈A,s.t.a⋅a−1=a0​
*   **李群**是指具有连续 (光滑) 性质的群. S O (3) SO(3) SO(3) 和 S E (3) SE(3) SE(3) 都是李群
    

#### 李代数的定义

每个李群都有与之对应的李代数, 李代数描述了李群的局部性质.

通用的李代数的定义如下:  
李代数由一个集合 V V V, 一个数域 F F F 和一个二元运算 [,] [, ] [,] 组成. 如果它们满足以下几条性质, 则称 ( V , F , [,] ) (V, F, [, ]) (V,F,[,]) 为一个李代数, 记作 g \mathfrak{g} g.

1.  封闭性: ∀ X , Y ∈ V , [ X , Y ] ∈ V \forall{X, Y} \in V, [X,Y] \in V ∀X,Y∈V,[X,Y]∈V.
    
2.  双线性: $\forall X,Y,Z \in V, a,b \in F $ 有:  
    [ a X + b Y , Z ] = a [ X , Z ] + b [ Y , Z ] , [ Z , a X + b Y ] = a [ Z , X ] + b [ Z , Y ] [aX+bY,Z]=a[X,Z]+b[Y,Z], \quad [Z, aX+bY]=a[Z,X]+b[Z,Y] [aX+bY,Z]=a[X,Z]+b[Y,Z],[Z,aX+bY]=a[Z,X]+b[Z,Y]
    
3.  自反性: ∀ X , ∈ V , [ X , X ] = 0 \forall{X,} \in V, [X,X]=0 ∀X,∈V,[X,X]=0.
    
4.  雅可比等价 ∀ X , Y , Z ∈ V , [ X , [ Y , Z ] ] + [ Z , [ X , Y ] ] + [ Y , [ Z , X ] ] = 0 \forall X,Y,Z \in V, \quad [X, [Y,Z]]+[Z, [X,Y ]]+[Y, [Z,X ]]=0 ∀X,Y,Z∈V,[X,[Y,Z]]+[Z,[X,Y]]+[Y,[Z,X]]=0.
    

其中的二元运算 [,] [,] [,] 被称为**李括号**. 例如三维向量空间 R 3 \mathbb{R^3} R3 上定义的叉积 × \times × 是一种李括号.

#### 李代数 s o (3) \mathfrak{so}(3) so(3)

*   李群 S O (3) SO(3) SO(3) 对应的李代数 s o (3) \mathfrak{so}(3) so(3) 是定义在 R 3 \mathbb{R^3} R3 上的向量, 记作 ϕ \phi ϕ.
    
    s o (3) = { ϕ ∈ R 3 , Φ = ϕ ∧ = [ 0 − ϕ 3 ϕ 2 ϕ 3 0 − ϕ 1 − ϕ 2 ϕ 1 0 ] ∈ R 3 × 3 } \mathfrak{so}(3) = \left\{ \phi \in \mathbb{R^3}, \Phi=\phi^\wedge = \left[
    
    $$\begin{array}{ccc} 0 & -\phi_3 & \phi_2\\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{array}$$
    
    \right] \in \mathbb{R^{3 \times 3}} \right\} so(3)=⎩⎨⎧​ϕ∈R3,Φ=ϕ∧=⎣⎡​0ϕ3​−ϕ2​​−ϕ3​0ϕ1​​ϕ2​−ϕ1​0​⎦⎤​∈R3×3⎭⎬⎫​
    
*   李代数 s o (3) \mathfrak{so}(3) so(3) 的李括号为  
    [ ϕ 1 , ϕ 2 ] = ( Φ 1 Φ 2 − Φ 2 Φ 1 ) ∨ [\phi_1, \phi_2] = (\Phi_1 \Phi_2 - \Phi_2 \Phi_1) ^\vee [ϕ1​,ϕ2​]=(Φ1​Φ2​−Φ2​Φ1​)∨  
    其中 ∨ ^\vee ∨是 ∧ ^\wedge ∧的逆运算, 表示将反对称矩阵还原为向量
    
*   s o (3) \mathfrak{so}(3) so(3) 和 S O (3) SO(3) SO(3) 间的映射关系为
    
    李 群 R = exp ⁡ ( ϕ ∧ ) = exp ⁡ (Φ) 李 代 数 ϕ = ln ⁡ ( R ) ∨
    
    $$\begin{aligned} 李群R &= \exp(\phi ^\wedge) = \exp (\Phi) \\ 李代数\phi &= \ln (R) ^\vee \end{aligned}$$
    
     李群 R 李代数ϕ​=exp(ϕ∧)=exp(Φ)=ln(R)∨​
    

#### 李代数 s e (3) \mathfrak{se}(3) se(3)

*   类似地, 李群 S E (3) SE(3) SE(3) 的李代数 s e (3) \mathfrak{se}(3) se(3) 是定义在 R 6 \mathbb{R^6} R6 上上的向量. 记作 ξ \xi ξ:
    
    s e (3) = { ξ = [ ρ ϕ ] ∈ R 6 , ρ ∈ R 3 , ϕ ∈ s o ( 3 ) , ξ ∧ = [ ϕ ∧ ρ 0 T 0 ] ∈ R 4 × 4 } \mathfrak{se}(3) = \left\{ \xi = \left[
    
    $$\begin{array}{c} \rho \\ \phi\end{array}$$
    
    \right] \in \mathbb{R^6}, \rho \in \mathbb{R^3}, \phi \in \mathfrak{so}(3), \xi^\wedge = \left[
    
    $$\begin{array}{cc} \phi^\wedge & \rho \\ 0^T & 0\end{array}$$
    
    \right] \in \mathbb{R^{4\times 4}} \right\} se(3)={ξ=[ρϕ​]∈R6,ρ∈R3,ϕ∈so(3),ξ∧=[ϕ∧0T​ρ0​]∈R4×4}  
    s e (3) \mathfrak{se}(3) se(3) 中的每个元素 ξ \xi ξ, 是一个六维向量. 前三维 ρ \rho ρ表示平移; 后三维 ϕ \phi ϕ表示旋转, 本质上是 s o (3) \mathfrak{so}(3) so(3) 元素.
    
*   在这里同样使用 ∧ ^\wedge ∧符号将六维向量扩展成为四维矩阵, 但不再表示反对称
    
    ξ ∧ = [ ϕ ∧ ρ 0 T 0 ] ∈ R 4 × 4 \xi^\wedge = \left[
    
    $$\begin{array}{cc} \phi^\wedge & \rho \\ 0^T & 0\end{array}$$
    
    \right] \in \mathbb{R^{4 \times 4}} ξ∧=[ϕ∧0T​ρ0​]∈R4×4
    
*   李代数 s e (3) \mathfrak{se}(3) se(3) 的李括号和 s o (3) \mathfrak{so}(3) so(3) 类似:  
    [ ξ 1 , ξ 2 ] = ( ξ 1 ∧ ξ 2 ∧ − ξ 2 ∧ ξ 1 ∧ ) ∨ [\xi_1, \xi_2] = (\xi^\wedge_1 \xi^\wedge_2 - \xi^\wedge_2 \xi^\wedge_1) ^\vee [ξ1​,ξ2​]=(ξ1∧​ξ2∧​−ξ2∧​ξ1∧​)∨
    
*   s e (3) \mathfrak{se}(3) se(3) 和 S E (3) SE(3) SE(3) 间映射关系为  
    李 群 T = exp ⁡ ( ξ ∧ ) 李 代 数 ξ = ln ⁡ (T) ∨
    
    $$\begin{aligned} 李群T &= \exp(\xi ^\wedge) \\ 李代数\xi &= \ln (T) ^\vee \end{aligned}$$
    
     李群 T 李代数ξ​=exp(ξ∧)=ln(T)∨​
    

### 李群与李代数的转换关系: 指数映射和对数映射

#### S O (3) SO(3) SO(3) 和 s o (3) \mathfrak{so}(3) so(3) 间的转换关系

*   将三维向量 ϕ \phi ϕ分解为其模长 θ \theta θ和方向向量 α \alpha α, 即 ϕ = θ α \phi=\theta\alpha ϕ=θα. 则从 s o (3) \mathfrak{so}(3) so(3) 到 S O (3) SO(3) SO(3) 的**指数映射**可表示为:
    
    R = exp ⁡ (ϕ) = exp ⁡ ( θ α ∧ ) = cos ⁡ θ I + ( 1 − cos ⁡ θ ) α α T + sin ⁡ θ α ∧ R = \exp(\phi) = \exp(\theta \alpha ^\wedge) = \cos \theta I + (1-\cos\theta) \alpha \alpha^T + \sin \theta \alpha ^\wedge R=exp(ϕ)=exp(θα∧)=cosθI+(1−cosθ)ααT+sinθα∧
    
    上式即为旋转向量到旋转矩阵的罗德里格斯公式, 可见 ** s o (3) \mathfrak{so}(3) so(3) 本质上是旋转向量组成的空间 **.
    
*   从 S O (3) SO(3) SO(3) 到 s o (3) \mathfrak{so}(3) so(3) 的**对数映射**可表示为:  
    ϕ = ln ⁡ (R) ∨ \phi = \ln(R)^\vee ϕ=ln(R)∨
    
    实际计算时可以通过迹的性质分别求出转角 θ \theta θ和转轴 α \alpha α  
    θ = arccos ⁡ t r (R) − 1 2 , R α = α \theta = \arccos \frac{tr(R)-1}{2}, \qquad R \alpha = \alpha θ=arccos2tr(R)−1​,Rα=α
    

#### S E (3) SE(3) SE(3) 和 s e (3) \mathfrak{se}(3) se(3) 间的转换关系

*   从 s e (3) \mathfrak{se}(3) se(3) 到 S E (3) SE(3) SE(3) 的**指数映射**可表示为:
    
    T = exp ⁡ ( ξ ∧ ) = [ R J ρ 0 T 1 ] T = \exp(\xi ^\wedge) = \left[
    
    $$\begin{array}{cc} R & J\rho \\ 0^T & 1\end{array}$$
    
    \right] T=exp(ξ∧)=[R0T​Jρ1​]
    
    其中  
    J = sin ⁡ θ θ I + ( 1 − sin ⁡ θ θ ) α α T + 1 − cos ⁡ θ θ α ∧ J = \frac{\sin\theta}{\theta} I + (1-\frac{\sin\theta}{\theta}) \alpha \alpha^T + \frac{1- \cos\theta}{\theta} \alpha^\wedge J=θsinθ​I+(1−θsinθ​)ααT+θ1−cosθ​α∧
    
    可以看到, 平移部分经过指数映射之后, 发生了一次以 J J J 为系数矩阵的线性变换.
    
*   从 S E (3) SE(3) SE(3) 到 s e (3) \mathfrak{se}(3) se(3) 的**对数映射**可表示为:  
    ξ = ln ⁡ (T) ∨ \xi = \ln(T)^\vee ξ=ln(T)∨
    
    实际计算时 ϕ \phi ϕ可以由 S O (3) SO(3) SO(3) 到 s o (3) \mathfrak{so}(3) so(3) 的映射得到, ρ \rho ρ可以由 t = J ρ t=J\rho t=Jρ计算得到.
    

![](https://img-blog.csdnimg.cn/20200408115750546.png)

### 李代数求导: 引入李代数的一大动机就是方便求导优化

#### 李群乘法与李代数加法的关系

1.  BCH 公式及其近似形式
    
    *   很遗憾地, 李群乘积和李代数加法并不等价, 即:  
        R 1 R 2 = exp ⁡ ( ϕ 1 ∧ ) exp ⁡ ( ϕ 1 ∧ ) ≠ exp ⁡ ( ( ϕ 1 + ϕ 2 ) ∧ ) R_1 R_2 = \exp(\phi_1^\wedge) \exp(\phi_1^\wedge) \ne \exp((\phi_1 + \phi_2)^\wedge) R1​R2​=exp(ϕ1∧​)exp(ϕ1∧​)​=exp((ϕ1​+ϕ2​)∧)
        
        李群乘积与李代数运算的对应关系由 BCH 公式给出:
        
        ln ⁡ ( exp ⁡ (A) exp ⁡ ( B ) ) = A + B + 1 2 [ A , B ] + 1 12 [ A , [ A , B ] ] − 1 12 [ B , [ A , B ] ] + . . . \ln(\exp(A) \exp(B)) = A+B +\frac{1}{2} [A,B] +\frac{1}{12} [A, [A,B]] -\frac{1}{12} [B, [A,B]] + ... ln(exp(A)exp(B))=A+B+21​[A,B]+121​[A,[A,B]]−121​[B,[A,B]]+...
        
        上式中 [,] [,] [,] 表示李括号运算.
        
    *   当 ϕ 1 \phi_1 ϕ1​或 ϕ 2 \phi_2 ϕ2​为小量时, 可以对 BCH 公式进行线性近似, 得到**李群乘积对应的李代数**的表达式:  
        R 1 ⋅ R 2 对 应 的 李 代 数 = ln ⁡ ( exp ⁡ ( ϕ 1 ∧ ) exp ⁡ ( ϕ 1 ∧ ) ) ∨ ≈ { J l ( ϕ 2 ) − 1 ϕ 1 + ϕ 2 当 ϕ 1 为小量时 J r ( ϕ 1 ) − 1 ϕ 2 + ϕ 1 当 ϕ 2 为小量时 R_1 \cdot R_2 对应的李代数 = \ln (\exp(\phi_1^\wedge) \exp(\phi_1^\wedge))^\vee \approx \left\{
        
         
        
        $$\begin{aligned} J_l(\phi_2)^{-1} \phi_1 + \phi_2 \quad \text{当$\phi_1$为小量时} \\ J_r(\phi_1)^{-1} \phi_2 + \phi_1 \quad \text{当$\phi_2$为小量时} \end{aligned}$$
        
         \right. R1​⋅R2​对应的李代数 =ln(exp(ϕ1∧​)exp(ϕ1∧​))∨≈{Jl​(ϕ2​)−1ϕ1​+ϕ2​当ϕ1​为小量时 Jr​(ϕ1​)−1ϕ2​+ϕ1​当ϕ2​为小量时​
        
        其中左乘雅可比矩阵 J l J_l Jl​即为从 S E (3) SE(3) SE(3) 到 s e (3) \mathfrak{se}(3) se(3) 对数映射中的雅可比矩阵  
        J l = sin ⁡ θ θ I + ( 1 − sin ⁡ θ θ ) α α T + 1 − cos ⁡ θ θ α ∧ J_l = \frac{\sin\theta}{\theta} I + (1-\frac{\sin\theta}{\theta}) \alpha \alpha^T + \frac{1- \cos\theta}{\theta} \alpha^\wedge Jl​=θsinθ​I+(1−θsinθ​)ααT+θ1−cosθ​α∧
        
        其逆为  
        J l − 1 = θ 2 cot ⁡ θ 2 I + ( 1 − θ 2 cot ⁡ θ 2 ) α α T + θ 2 α ∧ J_l^{-1} = \frac{\theta}{2} \cot{\frac{\theta}{2}} I + (1-\frac{\theta}{2} \cot{\frac{\theta}{2}}) \alpha \alpha^T + \frac{\theta}{2} \alpha^\wedge Jl−1​=2θ​cot2θ​I+(1−2θ​cot2θ​)ααT+2θ​α∧
        
        右乘雅可比矩阵只需对自变量取负号即可  
        J r (ϕ) = J l ( − ϕ ) J_r(\phi) = J_l(-\phi) Jr​(ϕ)=Jl​(−ϕ)
        
2.  李群 S O (3) SO(3) SO(3) 乘法与李代数 s o (3) \mathfrak{so}(3) so(3) 加法的关系:
    
    *   对旋转 R R R(李代数为 ϕ \phi ϕ) 左乘一个微小旋转 Δ R \Delta R ΔR(李代数为 Δ ϕ \Delta \phi Δϕ), 得到的旋转李群 Δ R ⋅ R \Delta R\cdot R ΔR⋅R 对应的李代数为:  
        Δ R ⋅ R 对 应 的 李 代 数 = ln ⁡ ( exp ⁡ ( Δ ϕ ∧ ) exp ⁡ ( ϕ ∧ ) ) = ϕ + J l − 1 (ϕ) Δ ϕ \Delta R \cdot R 对应的李代数 = \ln \left( \exp(\Delta \phi^\wedge) \exp(\phi^\wedge) \right) = \phi + J_l^{-1}(\phi)\Delta \phi ΔR⋅R 对应的李代数 =ln(exp(Δϕ∧)exp(ϕ∧))=ϕ+Jl−1​(ϕ)Δϕ
        
    *   反之, 李代数加法 ( ϕ + Δ ϕ ) (\phi+\Delta \phi) (ϕ+Δϕ) 对应的李群元素可表示为:  
        ( ϕ + Δ ϕ ) 对 应 的 李 群 = exp ⁡ ( ( ϕ + Δ ϕ ) ∧ ) = exp ⁡ ( ( J l Δ ϕ ) ∧ ) exp ⁡ ( ϕ ∧ ) = exp ⁡ ( ϕ ∧ ) exp ⁡ ( ( J r Δ ϕ ) ∧ ) (\phi+\Delta \phi) 对应的李群 = \exp((\phi+\Delta \phi)^\wedge) = \exp((J_l \Delta \phi)^\wedge) \exp(\phi^\wedge)= \exp(\phi^\wedge) \exp((J_r \Delta \phi)^\wedge) (ϕ+Δϕ) 对应的李群 =exp((ϕ+Δϕ)∧)=exp((Jl​Δϕ)∧)exp(ϕ∧)=exp(ϕ∧)exp((Jr​Δϕ)∧)
        
3.  同理, 李群 S E (3) SE(3) SE(3) 乘法与李代数 s e (3) \mathfrak{se}(3) se(3) 加法的关系:  
    exp ⁡ ( Δ ξ ∧ ) exp ⁡ ( ξ ∧ ) ≈ exp ⁡ ( ( J l − 1 Δ ξ + ξ ) ∧ ) exp ⁡ ( ξ ∧ ) exp ⁡ ( Δ ξ ∧ ) ≈ exp ⁡ ( ( J r − 1 Δ ξ + ξ ) ∧ ) \exp(\Delta \xi^\wedge) \exp(\xi^\wedge) \approx \exp\left( (J_l^{-1}\Delta \xi + \xi)^\wedge \right) \\ \exp(\xi^\wedge) \exp(\Delta \xi^\wedge) \approx \exp\left( (J_r^{-1}\Delta \xi + \xi)^\wedge \right) exp(Δξ∧)exp(ξ∧)≈exp((Jl−1​Δξ+ξ)∧)exp(ξ∧)exp(Δξ∧)≈exp((Jr−1​Δξ+ξ)∧)
    

#### S O (3) SO(3) SO(3) 上的李代数求导

对空间点 p p p 进行旋转, 得到 R p Rp Rp, 旋转之后点的坐标对旋转的导数可表示为:  
∂ ( R p ) ∂ R \frac{\partial(Rp)}{\partial R} ∂R∂(Rp)​

对于上式的求导, 有两种方式:

1.  用李代数 ϕ \phi ϕ表示**姿态** R R R, 然后根据李代数加法对 ϕ \phi ϕ求导.
2.  用李代数 φ \varphi φ表示**微小扰动** ∂ R \partial R ∂R, 然后根据李群左乘对 φ \varphi φ求导.

其中扰动模型表达式简单, 更为实用.

##### 李代数求导

用李代数 ϕ \phi ϕ表示**姿态** R R R, 求导得到  
∂ ( R p ) ∂ R = ∂ ( exp ⁡ ( ϕ ∧ ) p ) ∂ ϕ = − ( R p ) ∧ J l \frac{\partial(Rp)}{\partial R} = \frac{\partial( \exp(\phi^\wedge) p)}{\partial \phi} = -(Rp) ^\wedge J_l ∂R∂(Rp)​=∂ϕ∂(exp(ϕ∧)p)​=−(Rp)∧Jl​

##### 扰动模型 (左乘)

另一种求导方式是对 R R R 进行一次左乘扰动 ∂ R \partial R ∂R, 设左乘扰动 ∂ R \partial R ∂R 对应的李代数为 φ \varphi φ, 对 φ \varphi φ求导, 得到  
∂ ( R p ) ∂ R = exp ⁡ ( ( ϕ + φ ) ∧ ) p − exp ⁡ ( ϕ ∧ ) p φ = − ( R p ) ∧ \frac{\partial(Rp)}{\partial R} = \frac{ \exp((\phi+\varphi)^\wedge)p - \exp(\phi^\wedge)p }{\varphi} =-(Rp) ^\wedge ∂R∂(Rp)​=φexp((ϕ+φ)∧)p−exp(ϕ∧)p​=−(Rp)∧

#### S E (3) SE(3) SE(3) 上的李代数求导

类似地, 空间点 p p p 经过变换 T T T 得到 T p Tp Tp, 给 T T T 左乘一个扰动 Δ T = exp ⁡ ( δ ξ ∧ ) \Delta T = \exp (\delta \xi ^\wedge) ΔT=exp(δξ∧), 则有  
∂ ( R p ) δ ξ = [ I − ( R p + t ) ∧ 0 T 0 T ] = ( T P ) ⊙ \frac{\partial(Rp)}{\delta \xi} = \left[

$$\begin{array}{cc} I & -(Rp+t)^\wedge \\ 0^T & 0^T\end{array}$$

\right]= (TP) ^ \odot

δξ∂(Rp)​=[I0T​−(Rp+t)∧0T​]=(TP)⊙

ch05 相机与图像
----------

### 针孔相机模型

![](https://img-blog.csdnimg.cn/20200418173129814.png)

O − x − y − z O-x-y-z O−x−y−z 为相机坐标系, 现实空间点 P P P 的**相机坐标**为 [ X , Y , Z ] T [X,Y,Z]^T [X,Y,Z]T, 投影到 O ′ − x ′ − y ′ O'-x'-y' O′−x′−y′平面上的点 P ′ P' P′, 坐标为 [ X ′ , Y ′ , Z ′ ] T [X',Y',Z']^T [X′,Y′,Z′]T.

*   将成像平面对称到相机前方, 根据几何相似关系 Z f = X X ′ = Y Y ′ \frac{Z}{f} = \frac{X}{X'} = \frac{Y}{Y'} fZ​=X′X​=Y′Y​, 整理得到投影点 P ′ P' P′在投影平面上的坐标 P ′ = [ X ′ , Y ′ ] P'=[X',Y'] P′=[X′,Y′]:
    
    { X ′ = f X Z Y ′ = f Y Z \left\{
    
    $$\begin{aligned} X' = f \frac{X}{Z} \\ Y' = f \frac{Y}{Z} \\ \end{aligned}$$
    
     \right. ⎩⎪⎪⎨⎪⎪⎧​X′=fZX​Y′=fZY​​
    
*   转换得到投影点 P ′ P' P′在像素平面上的**像素坐标** P u , v = [ u , v ] T P_{u,v} = [u, v]^T Pu,v​=[u,v]T  
    { u = α X ′ + c x = f x X Z + c x v = β Y ′ + c y = f x X Z + c x \left\{
    
    $$\begin{aligned} u = \alpha X' + c_x &= f_x \frac{X}{Z}+c_x \\ v = \beta Y' + c_y &= f_x \frac{X}{Z}+c_x \\ \end{aligned}$$
    
     \right. ⎩⎪⎪⎨⎪⎪⎧​u=αX′+cx​v=βY′+cy​​=fx​ZX​+cx​=fx​ZX​+cx​​
    
    上式中 u u u, v v v, c x c_x cx​, c y c_y cy​, f x f_x fx​, f y f_y fy​的单位为像素, α \alpha α, β \beta β的单位为像素 / 米.
    
*   将上式写成矩阵形式, 得到 ** 现实空间点相机坐标 P P P **和**投影点像素坐标 P u v P_{uv} Puv​** 之间的关系:  
    Z P u v = Z [ u v 1 ] = [ f x 0 c x 0 f y c y 0 0 1 ] [ X Y Z ] ≜ K P Z P_{uv} = Z \left[
    
    $$\begin{array}{c} u \\ v \\ 1 \end{array}$$
    
    \right] = \left[
    
    $$\begin{array}{ccc} f_x &0 &c_x \\ 0 &f_y &c_y \\ 0 &0 &1 \end{array}$$
    
    \right] \left[
    
    $$\begin{array}{c} X \\ Y \\ Z \end{array}$$
    
    \right] \triangleq KP ZPuv​=Z⎣⎡​uv1​⎦⎤​=⎣⎡​fx​00​0fy​0​cx​cy​1​⎦⎤​⎣⎡​XYZ​⎦⎤​≜KP
    
    其中矩阵 K K K 称为相机的**内参数矩阵**.
    
*   上式中的 P P P 为现实空间点在相机坐标系下的**相机坐标**, 将其转为**世界坐标** P W P_W PW​, 有
    
    Z P u v = K ( R P W + t ) = K T P W ZP_{uv} = K(RP_W+t)= KTP_W ZPuv​=K(RPW​+t)=KTPW​
    
    因此 R R R, t t t(或 T T T) 又称为相机的**外参数**.
    
*   将最后一维进行**归一化处理**, 得到点 P P P 在归一化平面的**归一化坐标** P c = [ X / Z , Y / Z , 1 ] T P_c=[X/Z, Y/Z, 1]^T Pc​=[X/Z,Y/Z,1]T
    
    P c = P Z = K − 1 P u v P_c = \frac{P}{Z}={K^{-1} P_{uv}} Pc​=ZP​=K−1Puv​
    

![](https://img-blog.csdnimg.cn/20200420000343237.png)

参数矩阵有内参数 K K K 和外参数 R R R, t t t, 其中:

1.  内参数矩阵 K K K 体现了**归一化相机坐标**到**像素坐标**的变换.
    
    之所以是**归一化**坐标, 这体现了投影性质: 在某一条直线上的**空间点**, 最终会投影到同一**像素点**上.
    
2.  外参数矩阵 R R R, t t t(或 T T T) 体现了**世界坐标**到**相机坐标**的变换.
    

### 畸变模型

畸变包含两种: **径向畸变**和**切向畸变**.

*   **径向畸变**: 由透镜形状引起, 主要包括**桶形畸变**和**枕形畸变**.
    
    可以看成坐标点沿着长度方向发生了变化, 也就是其距离原点的长度发生了变化.  
    x d i s t o r t e d = x ( 1 + k 1 r 2 + k 2 r 4 + k 3 r 6 ) y d i s t o r t e d = y ( 1 + k 1 r 2 + k 2 r 4 + k 3 r 6 ) x_{distorted} = x(1+ k_1r^2 + k_2r^4 + k_3r6) \\ y_{distorted} = y(1+ k_1r^2 + k_2r^4 + k_3r6) xdistorted​=x(1+k1​r2+k2​r4+k3​r6)ydistorted​=y(1+k1​r2+k2​r4+k3​r6)
    
*   **切向畸变**: 由透镜和成像平面不严格平行引起.
    
    可以看成坐标点沿着切线方向发生了变化，也就是水平夹角发生了变化.  
    x d i s t o r t e d = x + 2 p 1 x y + p 2 ( r 2 + 2 x 2 ) y d i s t o r t e d = y + p 1 ( r 2 + 2 y 2 ) + 2 p 2 x y x_{distorted} = x + 2p_1xy + p_2(r^2+2x^2) \\ y_{distorted} = y + p_1(r^2+2y^2) + 2p_2xy xdistorted​=x+2p1​xy+p2​(r2+2x2)ydistorted​=y+p1​(r2+2y2)+2p2​xy
    

### 单目相机的成像过程

单目相机的成像过程：

1.  世界坐标系下有一个固定的原点 P P P, 其**世界坐标** P W P_W PW​
2.  由于相机在运动, 它的运动由 R R R, t t t 或变换矩阵 T ∈ S E (3) T\in SE(3) T∈SE(3) 描述. 原点 P P P 的**相机坐标** P c ~ = R P W + t \tilde{P_c}=RP_W+t Pc​~​=RPW​+t
3.  这时 P c ~ \tilde{P_c} Pc​~​的分量为 X X X, Y Y Y, Z Z Z, 把它们投影到归一化平面 Z = 1 Z =1 Z=1 上, 得到 P P P 的**归一化相机坐标** P c = P c ~ Z = [ X Z , Y Z , 1 ] T P_c =\frac{\tilde{P_c}}{Z}=[\frac{X}{Z},\frac{Y}{Z}, 1] ^T Pc​=ZPc​~​​=[ZX​,ZY​,1]T
4.  有畸变时, 根据畸变参数计算 P c P_c Pc​发生畸变后的归一化相机坐标
5.  P P P 的**归一化相机坐标** P c P_c Pc​经过内参 K K K 后, 对应到它的**像素坐标** P u v = K P c P_{uv}=KP_c Puv​=KPc​

在讨论相机成像模型时, 我们一共谈到了四种坐标: **世界坐标**、**相机坐标**、**归一化相机坐标**和**像素坐标**. 请读者厘清它们的关系, 它反映了整个成像的过程.

ch06 非线性优化
----------

### 状态估计问题

#### 最大后验与最大似然

SLAM 模型由状态方程和运动方程构成:  
{ x k = f ( x k − 1 , u k , w k ) z k , j = h ( y j , x k , v k , j ) \left\{

$$\begin{aligned} x_k &= f(x_{k-1}, u_k, w_k) \\ z_{k,j} &= h(y_j, x_k, v_{k,j}) \end{aligned}$$

\right.

{xk​zk,j​​=f(xk−1​,uk​,wk​)=h(yj​,xk​,vk,j​)​

通常假设两个噪声项 w k w_k wk​, v k , j v_{k,j} vk,j​满足零均值的高斯分布:  
w k ∼ N ( 0 , R k ) ,    v k , j ∼ N ( 0 , Q k , j ) w_k \sim \mathcal{N}(0, R_k) ,\; v_{k,j} \sim \mathcal{N}(0, Q_{k,j}) wk​∼N(0,Rk​),vk,j​∼N(0,Qk,j​)

对机器人的估计, 本质上就是已知**输入数据** u u u 和**观测数据** z z z 的条件下, 求机器人位姿 x x x 和路标点 y y y 的条件概率分布:  
P ( x , y ∣ z , u ) P(x,y|z,u) P(x,y∣z,u)

利用贝叶斯法则, 有:  
P ( x , y ∣ z , u ) = P ( z , u ∣ x , y ) P ( x , y ) P ( z , u ) ∝ P ( z , u ∣ x , y ) P ( x , y ) P(x,y|z,u) = \frac{P(z,u|x,y) P(x,y)}{P(z,u)} \propto P(z,u|x,y) P(x,y) P(x,y∣z,u)=P(z,u)P(z,u∣x,y)P(x,y)​∝P(z,u∣x,y)P(x,y)

其中 P ( x , y ∣ z , u ) P(x,y|z,u) P(x,y∣z,u) 为**后验概率**, P ( z , u ∣ x , y ) P(z,u|x,y) P(z,u∣x,y) 为**似然**, P ( x , y ) P(x,y) P(x,y) 为**先验**, 上式可表述为 后 验 概 率 ∝ 似 然 ⋅ 先 验 后验概率 \propto 似然 \cdot 先验 后验概率∝似然⋅先验. **直接求后验分布是困难的, 但是求一个状态最优估计, 使得在该状态下后验概率最大化则是可行的**:  
( x , y ) M A P ∗ = arg ⁡ max ⁡ P ( x , y ∣ z , u ) = arg ⁡ max ⁡ P ( z , u ∣ x , y ) P ( x , y ) (x,y)^*_{MAP} = \arg \max P(x,y | z,u) = \arg \max P(z,u|x,y) P(x,y) (x,y)MAP∗​=argmaxP(x,y∣z,u)=argmaxP(z,u∣x,y)P(x,y)

求解**最大后验概率相当于最大化似然和先验的乘积**. 因为 x x x, y y y 未知, 即不知道先验, 则可以求最大似然估计:  
( x , y ) M L E ∗ = arg ⁡ max ⁡ P ( z , u ∣ x , y ) (x,y)^*_{MLE} = \arg \max P(z,u|x,y) (x,y)MLE∗​=argmaxP(z,u∣x,y)

最大似然估计的直观意义为: **在什么样的状态下，最可能产生现在观测到的数据**.

#### 最小二乘

##### 基于观测数据 z z z 的最小二乘

对于某一次观测  
z k , j = h ( y j , x k ) + v k , j z_{k,j} = h(y_j, x_k) + v_{k,j} zk,j​=h(yj​,xk​)+vk,j​

由于假设噪声 v k , j ∼ N ( 0 , Q k , j ) v_{k,j} \sim \mathcal{N}(0, Q_{k,j}) vk,j​∼N(0,Qk,j​), 则观测数据 z j , k z_{j,k} zj,k​的似然为  
P ( z j , k ∣ x k , y j ) = N ( h ( y j , x k ) , Q k , j ) P(z_{j,k}|x_k,y_j) = \mathcal{N} (h(y_j, x_k), Q_{k,j}) P(zj,k​∣xk​,yj​)=N(h(yj​,xk​),Qk,j​)

将上式代入高斯分布表达式中, 并取负对数, 得到  
( x k , y j ) ∗ = arg ⁡ max ⁡ N ( h ( y j , x k ) , Q k , j ) = arg ⁡ min ⁡ ( ( z k , j − h ( x k , y j ) ) T Q k , j − 1 ( z k , j − h ( x k , y j ) ) )

$$\begin{aligned} (x_k,y_j)^* &= \arg \max \mathcal{N} (h(y_j, x_k), Q_{k,j}) \\ &= \arg \min \left( (z_{k,j} - h(x_k, y_j))^T Q_{k,j}^{-1} (z_{k,j} - h(x_k, y_j)) \right) \end{aligned}$$

(xk​,yj​)∗​=argmaxN(h(yj​,xk​),Qk,j​)=argmin((zk,j​−h(xk​,yj​))TQk,j−1​(zk,j​−h(xk​,yj​)))​

上式等价于最小化噪声项 (即误差) 的一个二次型, 其中 Q k , j − 1 Q_{k,j}^{-1} Qk,j−1​称为**信息矩阵**, 即高斯分布协方差矩阵的逆.

##### 基于观测数据 z z z 和输入数据 u u u 的最小二乘

因为观测 z z z 和输入 u u u 是独立的, 因此可对 z z z 和 u u u 的联合似然进行因式分解:  
P ( x , y ∣ z , u ) = ∏ k P ( u k ∣ x k − 1 , x k ) ∏ k , j P ( z j , k ∣ x k , y j ) P(x,y|z,u) = \prod_k P(u_k|x_{k-1},x_k) \prod_{k,j} P(z_{j,k}|x_k,y_j) P(x,y∣z,u)=k∏​P(uk​∣xk−1​,xk​)k,j∏​P(zj,k​∣xk​,yj​)

定义输入和观测数据与模型之间的误差:  
e u , k = x k − f ( x k − 1 , u k ) e z , j , k = z k , j − h ( x k , y j )

$$\begin{aligned} e_{u,k} &= x_{k} - f(x_{k-1}, u_k) \\ e_{z,j,k} &= z_{k,j} - h(x_k,y_j) \end{aligned}$$

eu,k​ez,j,k​​=xk​−f(xk−1​,uk​)=zk,j​−h(xk​,yj​)​

定义  
J ( x , y ) = ∑ k e u , k T R k − 1 e u , k + ∑ k ∑ j e z , k , j T Q k , j − 1 e z , k , j J(x,y) = \sum_k e_{u,k}^T R_k^{-1}e_{u,k} + \sum_k \sum_j e_{z,k,j}^T Q_{k,j}^{-1}e_{z,k,j} J(x,y)=k∑​eu,kT​Rk−1​eu,k​+k∑​j∑​ez,k,jT​Qk,j−1​ez,k,j​

则有  
( x k , y j ) ∗ = arg ⁡ min ⁡ J ( x , y ) (x_k,y_j)^* = \arg \min J(x,y) (xk​,yj​)∗=argminJ(x,y)

### 非线性最小二乘

对于非线性最小二乘问题:  
min ⁡ x F (x) = 1 2 ∣ ∣ f ( x ) ∣ ∣ 2 2 \min_{x} F(x) = \frac{1}{2} ||f(x)||_2^2 xmin​F(x)=21​∣∣f(x)∣∣22​

求解该问题的具体步骤如下:

1.  给定某个初始值 x 0 x_0 x0​
2.  对于第 k k k 次迭代, 寻找一个增量 Δ x k \Delta x_k Δxk​, 使得 ∣ ∣ F ( x k + Δ x k ) ∣ ∣ 2 2 ||F(x_k +\Delta x_k)||_2^2 ∣∣F(xk​+Δxk​)∣∣22​达到极小值
3.  若 Δ x k \Delta x_k Δxk​足够小, 则停止
4.  否则, 令 x k + 1 = x k + Δ x k x_{k +1} =x_k +\Delta x_k xk+1​=xk​+Δxk​, 返回第 2 步

这样, 最小二乘问题被转化为一个不断寻找下降增量 Δ x k \Delta x_k Δxk​的问题., 具体有以下方法

#### 一阶和二阶梯度法

将目标函数 F (x) F(x) F(x) 在 x k x_k xk​附近进行泰勒展开  
F ( x k + Δ x k ) ≈ F ( x k ) + J ( x k ) T Δ x k + 1 2 Δ x k T H ( x k ) x k F(x_k +\Delta x_k) \approx F(x_k) + J(x_k)^T \Delta x_k + \frac{1}{2} \Delta x_k^T H(x_k) x_k F(xk​+Δxk​)≈F(xk​)+J(xk​)TΔxk​+21​ΔxkT​H(xk​)xk​

其中 J (x) J(x) J(x) 是 F (x) F(x) F(x) 关于 x x x 的一阶导数矩阵, H (x) H(x) H(x) 是 F (x) F(x) F(x) 关于 x x x 的二阶导数矩阵.

*   若 Δ x k \Delta x_k Δxk​取一阶导数, 则  
    Δ x k ∗ = − J ( x k ) \Delta x_k^* = -J(x_k) Δxk∗​=−J(xk​)
    
*   若 Δ x k \Delta x_k Δxk​取二阶导数, 则  
    Δ x k ∗ = arg ⁡ min ⁡ ( F ( x k ) + J ( x k ) T Δ x k + 1 2 Δ x k T H ( x k ) x k ) \Delta x_k^* = \arg \min \left(F(x_k) + J(x_k)^T \Delta x_k + \frac{1}{2} \Delta x_k^T H(x_k) x_k \right) Δxk∗​=argmin(F(xk​)+J(xk​)TΔxk​+21​ΔxkT​H(xk​)xk​)
    
    令上式对 Δ x k \Delta x_k Δxk​导数等于 0, 则 Δ x k ∗ \Delta x_k^* Δxk∗​可以取 H Δ x k = − J H \Delta x_k = -J HΔxk​=−J 的解.
    

#### 高斯牛顿法

将 f ( x k ) f(x_k) f(xk​) 而非 F ( x k ) F(x_k) F(xk​) 在 x k x_k xk​附近进行泰勒展开  
f ( x k + Δ x k ) ≈ f ( x k ) + J ( x k ) T Δ x k f(x_k+\Delta x_k) \approx f(x_k) + J(x_k)^T \Delta x_k f(xk​+Δxk​)≈f(xk​)+J(xk​)TΔxk​

则  
Δ x k ∗ = arg ⁡ min ⁡ Δ x k 1 2 ∣ ∣ f ( x k ) + J ( x k ) T Δ x k ∣ ∣ 2 \Delta x_k^* = \arg \min_{\Delta x_k} \frac{1}{2} ||f(x_k)+J(x_k)^T \Delta x_k||^2 Δxk∗​=argΔxk​min​21​∣∣f(xk​)+J(xk​)TΔxk​∣∣2

令上式对 Δ x \Delta x Δx 的导数为 0, 得到**高斯牛顿方程**  
J ( x k ) f ( x k ) + J ( x k ) J T ( x k ) Δ x k = 0 J(x_k) f(x_k) + J(x_k) J^T(x_k) \Delta x_k = 0 J(xk​)f(xk​)+J(xk​)JT(xk​)Δxk​=0

令 H (x) = J ( x ) J T ( x ) H(x)=J(x)J^T(x) H(x)=J(x)JT(x), g (x) = − J ( x ) f ( x ) g(x)=-J(x)f(x) g(x)=−J(x)f(x), 则 Δ x k ∗ \Delta x_k^* Δxk∗​可以取 H Δ x k = g H \Delta x_k = g HΔxk​=g 的解.

#### 列文伯格 - 马夸尔特方法

泰勒展开只能在展开点附近才有较好的近似效果, 因此应给 Δ x \Delta x Δx 添加一个范围, 称为**信赖区域**.

定义一个指标 ρ \rho ρ刻画这个近似的好坏程度, 其分子为实际函数下降的值, 分母是近似模型下降的值:  
ρ = f ( x + Δ x ) − f (x) J ( x ) T Δ x \rho = \frac {f(x+\Delta x)-f(x)} {J(x)^T \Delta x} ρ=J(x)TΔxf(x+Δx)−f(x)​

通过调整 ρ \rho ρ来确定信赖区域:

*   若 ρ \rho ρ接近 1, 则近似是最好的.
*   若 ρ \rho ρ太小, 说明实际下降的值远小于近似下降的值, 则认为近似比较差, 需要缩小近似范围.
*   若 ρ \rho ρ太大, 说明实际下降的比预计的更大, 我们可以放大近似范围.

改良版的非线性优化框架如下:

1.  给定初始值 x 0 x_0 x0​, 以及初始优化半径 μ \mu μ
    
2.  对于第 k k k 次迭代, 求解:  
    min ⁡ Δ x k 1 2 ∣ ∣ f ( x k ) + J ( x k ) T Δ x k ∣ ∣ 2 s.t. ∣ ∣ D Δ x k ∣ ∣ 2 ≤ μ \min_{\Delta x_k} \frac{1}{2} ||f(x_k)+J(x_k)^T \Delta x_k||^2 \quad \text{s.t.} ||D\Delta x_k||^2 \leq \mu Δxk​min​21​∣∣f(xk​)+J(xk​)TΔxk​∣∣2s.t.∣∣DΔxk​∣∣2≤μ
    
    其中, μ \mu μ是信赖区域的半径, D D D 为系数矩阵
    
3.  计算 ρ \rho ρ
    
4.  若 ρ > 3 4 \rho > \frac34 ρ>43​则 μ = 2 μ \mu =2\mu μ=2μ
    
5.  若 ρ < 1 4 \rho < \frac14 ρ<41​则 μ = 0.5 μ \mu =0.5\mu μ=0.5μ
    
6.  若 ρ \rho ρ大于某阈值, 则认为近似可行. 令 x k + 1 = x k + Δ x k x_{k +1}=x_k +\Delta x_k xk+1​=xk​+Δxk​
    
7.  判断算法是否收敛. 如不收敛则返回第 2 步, 否则结束.
    

第 2 步中 Δ x k \Delta x_k Δxk​的求解要使用拉格朗日乘数法:  
L ( Δ x k , λ ) = 1 2 ∣ ∣ f ( x k ) + J ( x k ) T Δ x k ∣ ∣ 2 + λ 2 ( ∣ ∣ D Δ x k ∣ ∣ 2 − μ ) \mathcal{L}(\Delta x_k, \lambda) = \frac{1}{2} ||f(x_k)+J(x_k)^T \Delta x_k||^2+ \frac{\lambda}{2} (||D\Delta x_k||^2 - \mu) L(Δxk​,λ)=21​∣∣f(xk​)+J(xk​)TΔxk​∣∣2+2λ​(∣∣DΔxk​∣∣2−μ)

令上式对 Δ x k \Delta x_k Δxk​导数为 0, 得到  
( H + λ D T D ) Δ x k = g (H+\lambda D^T D) \Delta x_k = g (H+λDTD)Δxk​=g

考虑简化形式, 即 D = I D=I D=I, 则相当于求解  
( H + λ I ) Δ x k = g (H+\lambda I) \Delta x_k = g (H+λI)Δxk​=g

*   当 λ \lambda λ较小时, H H H 占主要地位, 这说明二次近似模型在该范围内是比较好的, 列文伯格 - 马夸尔特方法更接近于高斯牛顿法.
*   当 λ \lambda λ比较大时, λ I \lambda I λI 占据主要地位, 这说明二次近似模型在该范围内不够好, 列文伯格 - 马夸尔特方法更接近于一阶梯度下降法.

#### 文章目录

*   [ch02 初识 SLAM](#ch02_SLAM_3)
*   *   [经典视觉 SLAM 框架](#SLAM_5)
    *   [SLAM 问题的数学表述](#SLAM_44)
*   [ch03 三维空间刚体运动](#ch03__80)
*   *   [旋转矩阵](#_82)
    *   *   [点和向量, 坐标系](#_84)
        *   [坐标系间的欧氏变换](#_137)
        *   [变换矩阵与齐次坐标](#_202)
    *   [齐次坐标 (Homogeneous Coordinate) 的优势](#Homogeneous_Coordinate_252)
    *   *   *   [优势 1: 方便判断是否在直线或平面上](#1_254)
            *   [优势 2: 方便表示线线交点和点点共线](#2_266)
            *   [优势 3: 能够区分向量和点](#3_289)
            *   [优势 4: 能够表达无穷远点](#4_294)
            *   [优势 5: 能够简洁的表示变换](#5_298)
    *   [旋转向量和欧拉角](#_310)
    *   *   [旋转向量](#_312)
        *   [欧拉角](#_337)
    *   [四元数](#_354)
    *   *   [四元数的定义](#_359)
        *   [用单位四元数表示旋转](#_436)
*   [ch04 李群与李代数](#ch04__467)
*   *   [李群与李代数基础](#_469)
    *   *   [群的定义](#_492)
        *   [李代数的定义](#_502)
        *   [李代数 s o (3) \mathfrak{so}(3) so(3)](#mathfrakso3_533)
        *   [李代数 s e (3) \mathfrak{se}(3) se(3)](#mathfrakse3_566)
    *   [李群与李代数的转换关系: 指数映射和对数映射](#_601)
    *   *   [S O (3) SO(3) SO(3) 和 s o (3) \mathfrak{so}(3) so(3) 间的转换关系](#SO3mathfrakso3_603)
        *   [S E (3) SE(3) SE(3) 和 s e (3) \mathfrak{se}(3) se(3) 间的转换关系](#SE3mathfrakse3_629)
    *   [李代数求导: 引入李代数的一大动机就是方便求导优化](#__659)
    *   *   [李群乘法与李代数加法的关系](#_661)
        *   [S O (3) SO(3) SO(3) 上的李代数求导](#SO3_758)
        *   *   [李代数求导](#_772)
            *   [扰动模型 (左乘)](#_781)
        *   [S E (3) SE(3) SE(3) 上的李代数求导](#SE3_791)
*   [ch05 相机与图像](#ch05__806)
*   *   [针孔相机模型](#_808)
    *   [畸变模型](#_884)
    *   [单目相机的成像过程](#_907)
*   [ch06 非线性优化](#ch06__923)
*   *   [状态估计问题](#_925)
    *   *   [最大后验与最大似然](#_927)
        *   [最小二乘](#_990)
        *   *   [基于观测数据 z z z 的最小二乘](#z_992)
            *   [基于观测数据 z z z 和输入数据 u u u 的最小二乘](#zu_1020)
    *   [非线性最小二乘](#_1050)
    *   *   [一阶和二阶梯度法](#_1067)
        *   [高斯牛顿法](#_1096)
        *   [列文伯格 - 马夸尔特方法](#_1122)
*   [ch07 视觉里程计 01](#ch07_01_1183)
*   *   [特征点匹配](#_1185)
    *   *   [特征点](#_1187)
    *   [根据特征点匹配计算相机运动](#_1193)
    *   *   [2D-2D 匹配: 对极几何](#2D2D__1205)
        *   *   [对极约束](#_1207)
            *   [本质矩阵 E E E 的求解](#E_1238)
            *   [对极几何的讨论](#_1284)
        *   [3D-2D 匹配: PnP(Perspective-n-Point)](#3D2D_PnPPerspectivenPoint_1310)
        *   *   [直接线性变换 (DLT): 先求解相机位姿, 再求解空间点位置](#DLT__1322)
            *   [P3P: 先求解空间点位置, 再求解相机位姿](#P3P__1370)
            *   [Bundle Adjustment: 最小化重投影误差, 同时求解空间点位置和相机位姿](#Bundle_Adjustment__1401)
        *   [3D-3D 匹配: ICP](#3D3D_ICP_1485)
        *   *   [SVD 方法](#SVD_1505)
            *   [非线性优化方法](#_1542)

ch07 视觉里程计 01
-------------

### 特征点匹配

#### 特征点

### 根据特征点匹配计算相机运动

根据特征点匹配计算相机运动. 根据相机的成像原理不同, 分为以下 3 种情况：

1.  当相机为单目时, 我们只知道匹配点的像素坐标, 是为 2D-2D 匹配, 使用对极几何求解.
2.  当相机为双目或 RGB-D 时, 我们就知道匹配点的像素坐标和深度坐标, 是为 3D-3D 匹配, 使用 ICP 求解.
3.  如果有 3D 点及其在相机的投影位置, 也能估计相机的运动, 是为 3D-2D 匹配, 使用 PnP 求解.

#### 2D-2D 匹配: 对极几何

##### 对极约束

[外链图片转存失败, 源站可能有防盗链机制, 建议将图片保存下来直接上传 (img-QVwt5blH-1587570602884)(1587436458419.png)]{:height=“50%” width=“50%”}

假设我们要求取两帧图像 I 1 I_1 I1​, I 2 I_2 I2​之间的运动, 设第一帧到第二帧的运动为 R R R, t t t, 两个相机中心分别为 O 1 O_1 O1​, O 2 O_2 O2​. 考虑 I 1 I_1 I1​中有一个特征点 p 1 p_1 p1​, 它在 I 2 I_2 I2​中对应着特征点 p 2 p_2 p2​. 连线 O 1 p 1 → \overrightarrow{O_1 p_1} O1​p1​ ​和 O 2 p 2 → \overrightarrow{O_2 p_2} O2​p2​ ​在三维空间中交于点 P P P, 这时点 O 1 O_1 O1​, O 2 O_2 O2​, P P P 三个点可以确定一个平面, 称为**极平面**. O 1 O 2 O_1O_2 O1​O2​ 连线与像平面 I 1 I_1 I1​, I 2 I_2 I2​的交点分别为 e 1 e_1 e1​, e 2 e_2 e2​. e 1 e_1 e1​, e 2 e_2 e2​称为极点, O 1 O 2 O_1O_2 O1​O2​称为基线, 极平面与两个像平面 I 1 I_1 I1​, I 2 I_2 I2​之间的相交线 l 1 l_1 l1​, l 2 l_2 l2​称为极线.

P P P 在 I 1 I_1 I1​下的相机坐标为 P = [ X , Y , Z ] T P=[X,Y,Z]^T P=[X,Y,Z]T, 两个投影像素点 p 1 p_1 p1​, p 2 p_2 p2​的像素位置为 s 1 p 1 = K P s_1 p_1 = K P s1​p1​=KP, s 2 p 2 = K ( R P + t ) s_2 p_2 = K (RP + t) s2​p2​=K(RP+t).

取 p 1 p_1 p1​, p 2 p_2 p2​的归一化坐标 x 1 = K − 1 p 1 x_1 = K^{-1}p_1 x1​=K−1p1​, x 1 = K − 1 p 2 x_1 = K^{-1} p_2 x1​=K−1p2​, 则可以推得 x 2 ≃ R x 1 + t x_2 \simeq R x_1+ t x2​≃Rx1​+t. 上式中 ≃ \simeq ≃表示尺度意义上相等, 即在齐次坐标下是相等的, 物理上表示对原点成投影关系.

经过推导, 得到:  
x 2 T t ∧ R x 1 = 0 (1) x_2^T t ^\wedge R x_1 = 0 \tag{1} x2T​t∧Rx1​=0(1)  
代入 p 1 p_1 p1​, p 2 p_2 p2​, 得到:  
p 2 T K − T t ∧ R K − 1 p 1 (2) p_2^T K^{-T} t ^\wedge R K^{-1} p_1 \tag{2} p2T​K−Tt∧RK−1p1​(2)  
式 (1) (1) (1) 和式 (2) (2) (2) 都称为对极约束, 定义基础矩阵 F F F 和本质矩阵 E E E​, 可以进一步简化对极约束:  
E = t ∧ R F = K − T E K − 1 x 2 T E x 1 = p 2 T F p 1 = 0 (3) E = t ^\wedge R \qquad F = K^{-T}EK^{-1} \qquad x_2^TEx_1=p_2^TFp_1=0 \tag{3} E=t∧RF=K−TEK−1x2T​Ex1​=p2T​Fp1​=0(3)  
由于 E E E 与 F F F 之间只差了相机内参, 相机内参是已知的, 因此实践中往往使用形式更简单的 E E E.

##### 本质矩阵 E E E 的求解

考虑到 E E E 的尺度等价性, 可以用 8 对点来估计 E E E, 是为八点法.

对于一对匹配点, 其归一化坐标 x 1 = [ u 1 , v 1 , 1 ] T x_1=[u_1,v_1,1]^T x1​=[u1​,v1​,1]T, x 2 = [ u 2 , v 2 , 1 ] T x_2=[u_2,v_2,1]^T x2​=[u2​,v2​,1]T. 根据对极约束, 有  
( u 1 , v 1 , 1 ) ( e 1 e 2 e 3 e 4 e 5 e 6 e 7 e 8 e 9 ) ( u 2 v 2 1 ) = 0 (u_1, v_1, 1) \left(

$$\begin{array}{ccc} e_1 &e_2 &e_3 \\ e_4 &e_5 &e_6 \\ e_7 &e_8 &e_9 \\ \end{array}$$

\right) \left(

$$\begin{array}{c} u_2 \\ v_2 \\ 1 \\ \end{array}$$

\right) = 0

(u1​,v1​,1)⎝⎛​e1​e4​e7​​e2​e5​e8​​e3​e6​e9​​⎠⎞​⎝⎛​u2​v2​1​⎠⎞​=0

把矩阵

E E E

展开为向量

e = [ e 1 , e 2 , e 3 , e 4 , e 5 , e 6 , e 7 , e 8 , e 9 ] T e=[e_1,e_2,e_3,e_4,e_5,e_6,e_7,e_8,e_9]^T e=[e1​,e2​,e3​,e4​,e5​,e6​,e7​,e8​,e9​]T

, 对极约束可以写成与

e e e

有关的线性形式:

[ u 1 u 2 , u 1 v 2 , u 1 , v 1 u 2 , v 1 v 2 , v 2 , u 2 , v 2 , 1 ] ⋅ e = 0 [u_1u_2,u_1v_2,u_1, v_1u_2,v_1v_2,v_2, u_2,v_2,1] \cdot e = 0 [u1​u2​,u1​v2​,u1​,v1​u2​,v1​v2​,v2​,u2​,v2​,1]⋅e=0

把八对点对应的

x 1 x_1 x1​

,

x 2 x_2 x2​

分别代入方程中, 得到线性方程组:

( u 1 1 u 2 1 u 1 1 v 2 1 u 1 1 v 1 1 u 2 1 v 1 1 v 2 1 v 2 1 u 2 1 v 2 1 1 u 1 1 u 2 2 u 1 2 v 2 2 u 1 2 v 1 2 u 2 2 v 1 2 v 2 2 v 2 2 u 2 2 v 2 2 1 ⋮ ⋮ ⋮ ⋮ ⋮ ⋮ ⋮ ⋮ ⋮ u 1 1 u 2 8 u 1 8 v 2 8 u 1 8 v 1 8 u 2 8 v 1 8 v 2 8 v 2 8 u 2 8 v 2 8 1 ) ( e 1 e 2 e 3 e 4 e 5 e 6 e 7 e 8 e 9 ) = 0 \left(

$$\begin{array}{ccccccccc} u_1^1u_2^1 & u_1^1v_2^1 & u_1^1 & v_1^1u_2^1 & v_1^1v_2^1 & v_2^1 & u_2^1 & v_2^1 & 1 \\ u_1^1u_2^2 & u_1^2v_2^2 & u_1^2 & v_1^2u_2^2 & v_1^2v_2^2 & v_2^2 & u_2^2 & v_2^2 & 1 \\ \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\ u_1^1u_2^8 & u_1^8v_2^8 & u_1^8 & v_1^8u_2^8 & v_1^8v_2^8 & v_2^8 & u_2^8 & v_2^8 & 1 \\ \end{array}$$

\right) \left(

$$\begin{array}{c} e_1 \\ e_2 \\ e_3 \\ e_4 \\ e_5 \\ e_6 \\ e_7 \\ e_8 \\ e_9 \end{array}$$

\right) =0 ⎝⎜⎜⎜⎛​u11​u21​u11​u22​⋮u11​u28​​u11​v21​u12​v22​⋮u18​v28​​u11​u12​⋮u18​​v11​u21​v12​u22​⋮v18​u28​​v11​v21​v12​v22​⋮v18​v28​​v21​v22​⋮v28​​u21​u22​⋮u28​​v21​v22​⋮v28​​11⋮1​⎠⎟⎟⎟⎞​⎝⎜⎜⎜⎜⎜⎜⎜⎜⎜⎜⎜⎜⎛​e1​e2​e3​e4​e5​e6​e7​e8​e9​​⎠⎟⎟⎟⎟⎟⎟⎟⎟⎟⎟⎟⎟⎞​=0

求得 E 后, 对

E E E

进行 SVD 分解以求取

R R R

,

t t t

: 设

E E E

的 SVD 分解为

E = U Σ V T E = U \Sigma V^T E=UΣVT

, 则对应的

R R R

,

t t t

分别为:

t ∧ = U R Z ( π 2 ) Σ U T R = U R Z T ( π 2 ) Σ U T t^\wedge = U R_Z(\frac{\pi}{2}) \Sigma U^T \qquad R = U R_Z^T(\frac{\pi}{2}) \Sigma U^T t∧=URZ​(2π​)ΣUTR=URZT​(2π​)ΣUT  
其中 R Z ( π 2 ) R_Z(\frac{\pi}{2}) RZ​(2π​) 表示沿 Z Z Z 轴旋转 90° 得到的旋转矩阵.

##### 对极几何的讨论

1.  尺度不确定性: 2D 图像不具有深度信息, 这导致了**单目视觉的尺度不确定性**.
    
    实践中设 t t t 为单位 1, 计算相机运动和和特征点的 3D 位置, 这被称为单目 SLAM 的**初始化**.
    
2.  初始化的纯旋转问题: 若相机发生纯旋转, 导致 t t t 为零, 得到的 E E E 也将为零, 会导致我们无从求解 R. 因此**单目初始化不能只有纯旋转, 必须要有一定程度的平移**.
    
3.  多于 8 对点的情况:
    
    对于八点法, 有 A e = 0 Ae=0 Ae=0, 其中 A A A 为一个 8×9 的矩阵.
    
    若匹配点的个数多于 8 个, A A A 的尺寸变化, 上述方程不成立. 因此转而求取最小化二次型  
    min ⁡ e ∣ ∣ A e ∣ ∣ 2 2 = min ⁡ e e T A T A e \min_e || Ae ||_2^2 = \min_e e^T A^T A e emin​∣∣Ae∣∣22​=emin​eTATAe
    
    是为最小二乘意义下的 E E E 矩阵.
    

#### 3D-2D 匹配: PnP(Perspective-n-Point)

2D-2D 的对极几何方法需要 8 个或 8 个以上的点对（以八点法为例），且存在着初始化、纯旋转和尺度的问题。然而，如果两张图像中其中一张特征点的 3D 位置已知，那么最少只需 3 个点对（需要至少一个额外点验证结果）就可以估计相机运动。

在双目或 RGB-D 的视觉里程计中，我们可以直接使用 PnP 估计相机运动。而在单目视觉里程计中，必须先进行初始化，然后才能使用 PnP。

PnP 问题有多种解决方法:

1.  直接线性表变换 (DLT): 先求解相机位姿, 再求解空间点位置
2.  P3P: 先求解空间点位置, 再求解相机位姿
3.  Bundle Adjustment: 最小化重投影误差, 同时求解空间点位置和相机位姿

##### 直接线性变换 (DLT): 先求解相机位姿, 再求解空间点位置

考虑某个空间点 P P P 的**齐次世界坐标**为 P = ( X , Y , Z , 1 ) T P =(X,Y,Z, 1)^T P=(X,Y,Z,1)T . 在图像 I 1 I_1 I1​中投影到特征点的**归一化像素坐标** x 1 = ( u 1 , v 1 , 1 ) T x_1 =(u_1, v_1, 1)^T x1​=(u1​,v1​,1)T. 此时相机的位姿 R R R, t t t 是未知的, 定义增广矩阵 [ R ∣ t ] [R|t] [R∣t](不同于变换矩阵 T T T) 为一个 3×4 的矩阵, 包含了旋转与平移信息, 展开形式如下:  
s ( u 1 v 1 1 ) = ( t 1 t 2 t 3 t 4 t 5 t 6 t 7 t 8 t 9 t 10 t 11 t 12 ) ( X Y Z 1 ) s \left(

$$\begin{array}{c} u_1 \\ v_1 \\ 1 \end{array}$$

\right) = \left(

$$\begin{array}{cccc} t_1 & t_2 & t_3 & t_4 \\ t_5 & t_6 & t_7 & t_8 \\ t_9 & t_{10} & t_{11} & t_{12} \end{array}$$

\right) \left(

$$\begin{array}{c} X \\ Y \\ Z \\ 1 \end{array}$$

\right)

s⎝⎛​u1​v1​1​⎠⎞​=⎝⎛​t1​t5​t9​​t2​t6​t10​​t3​t7​t11​​t4​t8​t12​​⎠⎞​⎝⎜⎜⎛​XYZ1​⎠⎟⎟⎞​

用最后一行把 s 消去, 得到两个约束:  
{ t 1 T P − t 3 T P u 1 = 0 t 2 T P − t 3 T P v 1 = 0 \left\{

$$\begin{aligned} \boldsymbol{t}_1^T P - \boldsymbol{t}_3^T P u_1= 0 \\ \boldsymbol{t}_2^T P - \boldsymbol{t}_3^T P v_1= 0 \end{aligned}$$

\right.

{t1T​P−t3T​Pu1​=0t2T​P−t3T​Pv1​=0​

其中 t 1 = ( t 1 , t 2 , t 3 , t 4 ) T \boldsymbol{t}_1 = (t_1, t_2, t_3, t_4)^T t1​=(t1​,t2​,t3​,t4​)T, t 2 = ( t 5 , t 6 , t 7 , t 8 ) T \boldsymbol{t}_2 = (t_5, t_6, t_7, t_8)^T t2​=(t5​,t6​,t7​,t8​)T, t 3 = ( t 9 , t 10 , t 11 , t 12 ) T \boldsymbol{t}_3 = (t_9, t_{10}, t_{11}, t_{12})^T t3​=(t9​,t10​,t11​,t12​)T. t 1 \boldsymbol{t}_1 t1​, t 2 \boldsymbol{t}_2 t2​, t 3 \boldsymbol{t}_3 t3​为待求量.

将 N N N 对匹配的特征点代入方程中, 得到线性方程组:  
( P 1 T 0 − u 1 P 1 T 0 P 1 T − v 1 P 1 T ⋮ ⋮ ⋮ P N T 0 − u N P N T 0 P N T − v N P N T ) ( t 1 t 2 t 3 ) = 0 \left(

$$\begin{array}{ccc} P_1^T & 0 & -u_1P_1^T \\ 0 & P_1^T & -v_1P_1^T \\ \vdots & \vdots & \vdots \\ P_N^T & 0 & -u_NP_N^T \\ 0 & P_N^T & -v_NP_N^T \\ \end{array}$$

\right) \left(

$$\begin{array}{c} \boldsymbol{t}_1 \\ \boldsymbol{t}_2 \\ \boldsymbol{t}_3 \\ \end{array}$$

\right) =0

⎝⎜⎜⎜⎜⎜⎛​P1T​0⋮PNT​0​0P1T​⋮0PNT​​−u1​P1T​−v1​P1T​⋮−uN​PNT​−vN​PNT​​⎠⎟⎟⎟⎟⎟⎞​⎝⎛​t1​t2​t3​​⎠⎞​=0

只需 6 对匹配点即可求解增广矩阵 [ R ∣ t ] [R|t] [R∣t], 若匹配点数多于 6 对时, 可以求最小二乘解. 对于求解出的旋转矩阵 R R R, 可以通过 QR 分解等手段将其投影到 S E (3) SE(3) SE(3) 上.

##### P3P: 先求解空间点位置, 再求解相机位姿

[外链图片转存失败, 源站可能有防盗链机制, 建议将图片保存下来直接上传 (img-9IuduXXH-1587570602886)(1587451327097.png)]

已知 3 对匹配点的**世界坐标** A A A, B B B, C C C 和**投影坐标** a a a, b b b, c c c, 根据三角形的余弦定理, 有  
{ O A 2 + O B 2 − 2 O A ⋅ O B ⋅ cos ⁡ ⟨ a , b ⟩ = A B 2 O B 2 + O C 2 − 2 O B ⋅ O C ⋅ cos ⁡ ⟨ b , c ⟩ = B C 2 O A 2 + O C 2 − 2 O A ⋅ O C ⋅ cos ⁡ ⟨ a , c ⟩ = A C 2 \left\{

$$\begin{aligned} OA^2 + OB^2 - 2 OA \cdot OB \cdot \cos \langle a,b \rangle = AB^2 \\ OB^2 + OC^2 - 2 OB \cdot OC \cdot \cos \langle b,c \rangle = BC^2 \\ OA^2 + OC^2 - 2 OA \cdot OC \cdot \cos \langle a,c \rangle = AC^2 \\ \end{aligned}$$

\right.

⎩⎪⎨⎪⎧​OA2+OB2−2OA⋅OB⋅cos⟨a,b⟩=AB2OB2+OC2−2OB⋅OC⋅cos⟨b,c⟩=BC2OA2+OC2−2OA⋅OC⋅cos⟨a,c⟩=AC2​

记 x = O A / O C x=OA/OC x=OA/OC, y = O B / O C y=OB/OC y=OB/OC, u = B C 2 / A B 2 u=BC^2/AB^2 u=BC2/AB2, v = A C 2 / A B 2 v=AC^2/AB^2 v=AC2/AB2  
{ ( 1 − u ) y 2 − u x 2 − cos ⁡ ⟨ b , c ⟩ y + 2 u x y cos ⁡ ⟨ a , b ⟩ + 1 = 0 ( 1 − w ) x 2 − w y 2 − cos ⁡ ⟨ a , c ⟩ y + 2 w x y cos ⁡ ⟨ a , b ⟩ + 1 = 0 \left\{

$$\begin{aligned} (1-u)y^2 - ux^2 - \cos \langle b,c \rangle y + 2uxy \cos \langle a,b \rangle + 1 &= 0 \\ (1-w)x^2 - wy^2 - \cos \langle a,c \rangle y + 2wxy \cos \langle a,b \rangle + 1 &= 0 \\ \end{aligned}$$

\right.

{(1−u)y2−ux2−cos⟨b,c⟩y+2uxycos⟨a,b⟩+1(1−w)x2−wy2−cos⟨a,c⟩y+2wxycos⟨a,b⟩+1​=0=0​

上式中, 三个余弦角 cos ⁡ ⟨ a , b ⟩ \cos \langle a,b \rangle cos⟨a,b⟩, cos ⁡ ⟨ b , c ⟩ \cos \langle b,c \rangle cos⟨b,c⟩, cos ⁡ ⟨ a , c ⟩ \cos \langle a,c \rangle cos⟨a,c⟩以及 u u u, v v v 是已知的, 可以求解出 x x x, y y y, 进而求解出 A A A, B B B, C C C 三点的相机坐标. 然后根据 3D-3D 的点对, 计算相机的运动 R R R, t t t.

##### Bundle Adjustment: 最小化重投影误差, 同时求解空间点位置和相机位姿

设相机位姿变换矩阵 T T T, 某空间点的世界坐标 P i = [ X i , Y i , Z i ] T P_i =[X_i,Y_i,Z_i]^T Pi​=[Xi​,Yi​,Zi​]T, 其投影的像素坐标为 u i = [ u i , v i ] T \boldsymbol{u}_i =[u_i ,v_i ]^T ui​=[ui​,vi​]T, 像素位置与空间点位置的关系如下:  
s i u i = K T P i s_i \boldsymbol{u}_i = K T P_i si​ui​=KTPi​

由于相机位姿未知及观测点的噪声, 上式存在一个误差, 称为**重投影误差** e = u i − 1 s i K T P i e=u_i - \frac{1}{s_i} KTP_i e=ui​−si​1​KTPi​. 因此我们对重投影误差求和, 寻找最好的相机位姿和特征点的空间位置, 最小化重投影误差:  
T ∗ = arg ⁡ min ⁡ T 1 2 ∑ i = 1 n ∣ ∣ u i − 1 s i K T P i ∣ ∣ 2 P i ∗ = arg ⁡ min ⁡ P i 1 2 ∑ i = 1 n ∣ ∣ u i − 1 s i K T P i ∣ ∣ 2 T^* = \arg \min_{T} \frac{1}{2} \sum_{i=1}^n ||u_i - \frac{1}{s_i} KTP_i||^2 \\ P_i^* = \arg \min_{P_i} \frac{1}{2} \sum_{i=1}^n ||u_i - \frac{1}{s_i} KTP_i||^2 T∗=argTmin​21​i=1∑n​∣∣ui​−si​1​KTPi​∣∣2Pi∗​=argPi​min​21​i=1∑n​∣∣ui​−si​1​KTPi​∣∣2

使用最小二乘优化, 要分别求 e e e 对 T T T 和 P P P 的导数:  
e ( x + Δ x ) ≈ e (x) + J Δ x e(x+\Delta x) \approx e(x) + J \Delta x e(x+Δx)≈e(x)+JΔx

*   求 e e e 对 T T T 的导数:
    
    当 e e e 为像素坐标误差 (2 维), x x x 为相机位姿 (6 维) 时, J J J 将是一个 2×6 的矩阵. 我们来推导 J J J 的形式:
    
    取中间变量 P ′ = ( T P ) 1 : 3 = [ X ′ , Y ′ , Z ′ ] T P'= (TP)_{1:3}=[X',Y',Z']^T P′=(TP)1:3​=[X′,Y′,Z′]T
    
    使用李代数求导的扰动模型, 对 T T T 左乘微小扰动 δ ξ \delta \xi δξ, 求导得到:  
    ∂ e ∂ δ ξ = lim ⁡ δ ξ = 0 e ( δ ξ ⊕ ξ ) − e (ξ) δ ξ = ∂ e ∂ P ′ ∂ P ′ ∂ δ ξ \frac{\partial e}{\partial \delta \xi} = \lim_{\delta \xi =0} \frac{e(\delta \xi \oplus \xi) - e(\xi)}{\delta \xi} = \frac{\partial e}{\partial P'} \frac{\partial P'}{\partial \delta \xi} ∂δξ∂e​=δξ=0lim​δξe(δξ⊕ξ)−e(ξ)​=∂P′∂e​∂δξ∂P′​
    
    其中的 ⊕ \oplus ⊕表示李代数的左乘扰动
    
    其中第一项 ∂ e ∂ P ′ \frac{\partial e}{\partial P'} ∂P′∂e​:  
    ∂ e ∂ P ′ = − [ ∂ u ∂ X ′ ∂ u ∂ Y ′ ∂ u ∂ Z ′ ∂ v ∂ X ′ ∂ v ∂ Y ′ ∂ v ∂ Z ′ ] = − [ f x Z ′ 0 − f x X ′ Z ′ 2 0 f y Z ′ − f y Y ′ Z ′ 2 ] \frac{\partial e}{\partial P'} = - \left[
    
    $$\begin{array}{cccc} \frac{\partial u}{\partial X'} & \frac{\partial u}{\partial Y'} & \frac{\partial u}{\partial Z'} \\ \frac{\partial v}{\partial X'} & \frac{\partial v}{\partial Y'} & \frac{\partial v}{\partial Z'} \end{array}$$
    
    \right] = - \left[
    
    $$\begin{array}{cccc} \frac{f_x}{Z'} & 0 & -\frac{f_x X'}{Z'^2} \\ 0 & \frac{f_y}{Z'} & -\frac{f_y Y'}{Z'^2} \end{array}$$
    
    \right] ∂P′∂e​=−[∂X′∂u​∂X′∂v​​∂Y′∂u​∂Y′∂v​​∂Z′∂u​∂Z′∂v​​]=−[Z′fx​​0​0Z′fy​​​−Z′2fx​X′​−Z′2fy​Y′​​]
    
    第二项 ∂ P ′ ∂ δ ξ \frac{\partial P'}{\partial \delta \xi} ∂δξ∂P′​为变换后的点关于李代数的导数:  
    ∂ P ′ ∂ δ ξ = ( T P ) ∂ δ ξ = ( T P ) ⊙ = [ I − P ′ ∧ 0 T 0 T ] \frac{\partial P'}{\partial \delta \xi} = \frac{(T P)}{\partial \delta \xi} = (TP) ^\odot = \left[
    
    $$\begin{array}{cc} I & -P'^\wedge \\ 0^T & 0^T \end{array}$$
    
    \right] ∂δξ∂P′​=∂δξ(TP)​=(TP)⊙=[I0T​−P′∧0T​]
    
    在 P ′ P' P′定义中, 取出前三维, 得到  
    ∂ P ′ ∂ δ ξ = [ I , − P ′ ∧ ] \frac{\partial P'}{\partial \delta \xi} = [ I , -P'^\wedge ] ∂δξ∂P′​=[I,−P′∧]
    
    将两项相乘, 得到了 2×6 的雅可比矩阵 J T J^T JT  
    J T = ∂ e ∂ δ ξ = − [ f x Z ′ 0 − f x X ′ Z ′ 2 − f x X ′ Y ′ Z ′ 2 f x + f x X ′ 2 Z ′ 2 − f x Y ′ Z ′ 0 f y Z ′ − f y Y ′ Z ′ 2 − f y − f y Y ′ 2 Z ′ 2 f y X ′ Y ′ Z ′ 2 f y X ′ Z ′ ] J^T = \frac{\partial e}{\partial \delta \xi} = - \left[
    
    $$\begin{array}{cccccc} \frac{f_x}{Z'} & 0 & -\frac{f_x X'}{Z'^2} & -\frac{f_x X' Y'}{Z'^2} & f_x+\frac{f_x X'^2}{Z'^2} & -\frac{f_x Y'}{Z'} \\ 0 & \frac{f_y}{Z'} & -\frac{f_y Y'}{Z'^2} & -f_y-\frac{f_y Y'^2}{Z'^2} & \frac{f_y X' Y'}{Z'^2} & \frac{f_y X'}{Z'} \end{array}$$
    
    \right] JT=∂δξ∂e​=−[Z′fx​​0​0Z′fy​​​−Z′2fx​X′​−Z′2fy​Y′​​−Z′2fx​X′Y′​−fy​−Z′2fy​Y′2​​fx​+Z′2fx​X′2​Z′2fy​X′Y′​​−Z′fx​Y′​Z′fy​X′​​]
    
*   求 e e e 对 P P P 的导数
    

#### 3D-3D 匹配: ICP

对于一组已配对好的 3D 点:  
P = { p 1 , ⋯   , p n } , P ′ = { p 1 ′ , ⋯   , p n ′ } P = \{p_1, \cdots ,p_n\}, \quad P'= \{p_1', \cdots, p_n'\} P={p1​,⋯,pn​},P′={p1′​,⋯,pn′​}

现在, 想要找一个欧氏变换 R R R, t t t, 使得:  
∀ i , p i = R p i ′ + t \forall i, \quad p_i = R p_i' + t ∀i,pi​=Rpi′​+t

ICP 问题的求解包含两种方式:

1.  利用线性代数的求解 (主要是 SVD)
2.  利用非线性优化方式的求解 (类似于 Bundle Adjustment)

##### SVD 方法

定义第 i i i 对点的误差项为 e i = p i − ( R p i ′ + t ) e_i = p_i - (R p'_i + t) ei​=pi​−(Rpi′​+t), 定义两组点的质心 p = 1 n ∑ i = 1 n ( p i ) p = \frac{1}{n} \sum_{i=1}^n (p_i) p=n1​∑i=1n​(pi​), p ′ = 1 n ∑ i = 1 n ( p i ′ ) p'= \frac{1}{n} \sum_{i=1}^n (p_i') p′=n1​∑i=1n​(pi′​)

构建最小二乘问题, 求取最合适的 R R R, t t t.  
min ⁡ R , t J = 1 2 ∑ i = 1 n ∣ ∣ ( p i − ( R p i ′ + t ) ) ∣ ∣ 2 2 = 1 2 ∑ i = 1 n ∣ ∣ p i − p − R ( p i ′ − p ′ ) ∣ ∣ 2 + ∣ ∣ p − R p ′ − t ∣ ∣ 2

$$\begin{aligned} \min_{R,t} J &= \frac{1}{2} \sum_{i=1}^n ||(p_i - (Rp_i' + t))||_2^2 \\ &= \frac{1}{2} \sum_{i=1}^n ||p_i-p-R(p_i'-p')||^2 + ||p - Rp' - t||^2 \end{aligned}$$

R,tmin​J​=21​i=1∑n​∣∣(pi​−(Rpi′​+t))∣∣22​=21​i=1∑n​∣∣pi​−p−R(pi′​−p′)∣∣2+∣∣p−Rp′−t∣∣2​

左边只和旋转矩阵 R R R 相关, 而右边既有 R R R 也有 t t t, 但只和质心相关. 因此令左边取最小值解出 R R R, 代入到右边令式子等于 0 求出 t t t.

定义去质心坐标 q i = p i − p q_i=p_i-p qi​=pi​−p, q i ′ = p i ′ − p ′ q'_i=p'_i-p' qi′​=pi′​−p′, 则优化目标可写成:  
R ∗ = min ⁡ R ∑ i = 1 n ∣ ∣ p i − p − R ( p i ′ − p ′ ) ∣ ∣ 2 = min ⁡ R ∑ i = 1 n − q i T R q i ′ = − t r ( R ∑ i = 1 n q i ′ q i T )

$$\begin{aligned} R ^* &= \min_{R} \sum_{i=1}^n ||p_i-p-R(p_i'-p')||^2 \\ &= \min_{R} \sum_{i=1}^n -q_i^T R q_i' \\ &= -tr \left( R \sum_{i=1}^n q'_i q_i^T \right) \end{aligned}$$

R∗​=Rmin​i=1∑n​∣∣pi​−p−R(pi′​−p′)∣∣2=Rmin​i=1∑n​−qiT​Rqi′​=−tr(Ri=1∑n​qi′​qiT​)​

省略数学证明, 定义矩阵:

W = ∑ i = 1 n q i q i ′ T W = \sum_{i=1}^n q_i q_i'^T W=i=1∑n​qi​qi′T​

对矩阵

W W W

进行 SVD 分解得到:

W = U Σ V T W = U \Sigma V^T W=UΣVT

可求解

R = U V T R = UV^T R=UVT

##### 非线性优化方法

使用李代数表达表达位姿, 目标函数可以写成  
min ⁡ ξ = 1 2 ∑ i = 1 n ∣ ∣ ( p i − exp ⁡ ( ξ ∧ ) p i ′ ) ∣ ∣ 2 2 \min_{\xi} = \frac12 \sum_{i=1}^n ||(p_i - \exp(\xi^\wedge) p_i')||_2^2 ξmin​=21​i=1∑n​∣∣(pi​−exp(ξ∧)pi′​)∣∣22​  
误差项关于位姿的导数可以用李代数求导的扰动模型, 计算导数得到:  
∂ e ∂ δ ξ = − ( exp ⁡ ( ξ ∧ ) p i ′ ) ⊙ \frac{\partial e}{\partial \delta \xi} = - (\exp (\xi^\wedge) p_i')^\odot ∂δξ∂e​=−(exp(ξ∧)pi′​)⊙  
可以直接使用最小二乘优化方法求解位姿.