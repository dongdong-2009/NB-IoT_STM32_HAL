# 第一次大连快轨测试Demo（2017.11.04）
## 数据流
STM32L452RE型开发板+AD7420温度读取+ADXL377Z三轴加速度采集+NB-IoT信号强度采集+NB-IoT_CoAP协议上传到NBcenter云平台
## 云平台信息
CoAP服务器地址：[120.76.136.124:8060](http://120.76.136.124:8060)
```
账号：16120054@bjtu.edu.cn 
密码：bjtu2012
```
### 备注
1. 此项目工程需要用Keil-MDK5 软件来打开；
2. 项目中.ioc格式的文件需要用STM32CubeMX软件打开，CubeMX可以用图形化的方式来自动生成各种IDE环境（IAR、MDK、eclipse等）下的STM32硬件初始化代码。
