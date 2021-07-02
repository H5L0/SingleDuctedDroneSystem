# SingleDuctedDroneSystem
SingleDuctedDroneSystem是一个用于控制单涵道无人机的飞控，基于Arduino开发，运行在Arduino pro mini 上，主要包含无线通信、PID控制和参数储存功能。

## 如何编译
* 使用`git clone https://github.com/H5L0/SingleDuctedDroneSystem.git`命令将库克隆到本地。
* 下载安装[Arduino IDE](https://www.arduino.cc/en/software)。
* 确保你的IDE安装了程序中使用到的RF24库。
* 在IDE中打开SingleDuctedDroneSystem文件夹，点击菜单中的编译即可。

## 如何运行
* 准备一块HL board（未公开）或者Arduino pro mini开发板（5V 16MHz）。
* 将Arduino IDE中的板类型更改为Arduino pro mini（5V 16MHz）。
* 使用USB to TTL线连接开发板。
* 点击上传。

## 程序架构
程序包含4个模块，分别对应4个头文件：
- ["Model.h"](/blob/Float/Model.h) - 与无人机的控制相关的内容
- ["Synchronizer.h"](/blob/Float/Synchronizer.h) - 同步器，实现通信相关内容
- ["Storage.h"](/blob/Float/Storage.h) - 存储器，用于存取系统的参数
- ["Beeper.h"](/blob/Float/Beeper.h) - 蜂鸣器，用于控制蜂鸣器

#### Synchronizer 同步器
同步器控制RF24L01+无线通信模块，它通过“Half Link”通信协议定义的数据格式和通信流程与地面站通信，流程如下：
```
 地面站 无人机
    |    |
    |--->| 接受命令
    |    | 无人机执行命令
    |<---| 发回Feedback（Ack/Failed/Data）
    |    |
```
> 通信中开启了RF24L01+的自动确认重传，所以每个发送的箭头都可能包含多次发送

#### HL.MPU6050
基于[MPU6050_light](https://github.com/rfetick/MPU6050_light)实现的MPU6050库，功能基本相同，用于获取无人机的姿态数据。


