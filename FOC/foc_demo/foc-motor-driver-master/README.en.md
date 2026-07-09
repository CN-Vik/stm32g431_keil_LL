# FOC电机驱动器

#### Description
Main controller: STM32G473CBT6.  
Supports UART hardware serial port, USB interface, SWD programming port, ABZ encoder interface, Hall encoder interface.  
Phase current sampling is used for current acquisition.  

The author has written a custom motor library, which has already implemented current loop and speed loop for both sensored and sensorless control.  
Sensorless control supports square-wave high-frequency injection, sliding mode observer, and flux observer.  
Already adapted to CoolFly 2207 motor and ALIENTEK PMSM motor.  

#### Software Architecture
Written in C language, using ST HAL library.  
FOC-related code is modularized.  

#### Installation

1.  Compile with MDK5, default configuration  

#### Instructions

1.  None  

#### Contribution

1.  Fork this repository  

#### Features

1.  Use Readme\_XXX.md to support different languages, such as Readme\_en.md, Readme\_zh.md  
2.  Gitee official blog [blog.gitee.com](https://blog.gitee.com)  
3.  You can explore excellent open-source projects at [https://gitee.com/explore](https://gitee.com/explore)  
4.  [GVP](https://gitee.com/gvp) is Gitee’s Most Valuable Project award, given to outstanding open-source projects  
5.  Gitee official manual [https://gitee.com/help](https://gitee.com/help)  
6.  Gitee Stars is a column to showcase outstanding Gitee members [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)  
