# 基础配置

## 1.配置RCC

![78712882a497bf4051bb047d4b85347](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/75c54388-384d-42a3-b98e-f5243f4310c3)


## 2.配置SYS

![291e6227755a39537e6913abb8ff178](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/39800bb5-d1bd-4bcd-92e1-0463ddc10808)


## 3.配置时钟树

![232aad93ef5ecc3a656de29bd93bbf5](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/447df534-5353-4eb9-a4ea-f90b278ec184)



# 控制电机配置

## 4.配置can频率以及can接收中断

![c7083d869ed3e7f24ab16621294093f](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/b5980e07-da46-41db-94ee-eaabdb4d333a)


![db71224168323bd2a5dd7a9cca46c5f](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/b7f0a586-6443-47f4-b28c-00ce217f481d)


`这里只需打开RX0的中断就行`
`can2相同步骤`


# 控制航模遥控配置

## 5.配置定时器2

`TIM2用于定时解析航模ppm值`

![32c7260aba3ddb9c0e7eadb2ae280ee](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/6ac47dd3-df4a-4db0-95ad-38bdad04c7c7)


## 6.配置PF7

![82529f8251b12eadfe10845ac58da2e](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/64bd6d79-64d5-4f0c-9ad3-5533551b284a)


`航模接收器接在PF7上`


# action配置

## 7.配置串口4

`串口4用于读取action数据`
`注意这里我们自行选择PC10和PC11复用为串口引脚`

![8484d0c901deddeb26d4a9dcdf096fa](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/64256023-06b9-4d83-88c9-40133fca261d)



# 调试通道配置
## 8.配置串口1

`串口1用于与上位机通讯，实现数据可视化`

![b7b1c3fe8716481313ac422a1460380](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/cfad1528-163b-44a8-bc05-684dfcb1fa06)



# 任务配置

## 9.配置FreeRTOS

`这里我们将默认的任务重命名成电机控制任务`
`后续的任务可以自行添加，优先级不超过BelowNormal`

电机总控任务：最高优先级

![e53610daf98c31a061b7f4f5e8c5d30 1](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/f7a5be2d-801e-4b19-a574-b176a8a2a4b5)


底盘任务：较高优先级

![721f74bf418dca99e0662dc1b5d6488](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/57bf86f9-8d05-404e-bfd3-3e2a884de765)


摩擦带发射任务：低优先级

![c66a027f68908ae6581079151863b77](https://github.com/RaySHAOWEI/2024RC_R1/assets/45119387/ea49b042-f4cf-4c15-a2e2-fd1dd18da991)


### 优先级说明：

一般情况下，电机总控任务和整车状态机任务是最高优先级，底盘是较高优先级，其他任务统一低优先级。
