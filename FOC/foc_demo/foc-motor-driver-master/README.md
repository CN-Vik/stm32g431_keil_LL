# FOC电机驱动器

#### 介绍
主控STM32G473CBT6 
带UART硬件串口、USB接口、SWD烧录口、ABZ编码器接口、霍尔编码器接口，电流采集使用 相电流采集方式。
作者自写的电机库，已经实现有感和无感的电流环、速度环。
无感支持方波高频注入、滑膜观测器、磁链观测器。
已经适配了酷飞2207电机和正点原子PMSM电机。


FOC电机驱动器

#### 软件架构
采用C代码编写，使用st的HAL库。
有关foc的代码进行模块化编写。


#### 安装教程

1.  使用mdk5进行编译，配置默认


#### 使用说明

1.  无

#### 参与贡献

1.  Fork 本仓库



#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
