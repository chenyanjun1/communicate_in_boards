# 消息定义

## 上板接受消息 && 下板发送消息

```
    float IMU_angle_yaw;        //陀螺仪yaw轴的值 [0:16]
    float IMU_angle_roll;       //陀螺仪roll轴的值 [16:32]
```


|     域     | 偏移 | 长度(bits) |               范围               |
| :--------: | :--: | :--------: | :-------------------------------: |
| 下板yaw值 |  0  |     16     | float 转`uint16_t`，范围：0 ~ 360 |
| 下板roll值 |  16  |     16     | float 转`uint16_t`，范围：0 ~ 360 |

## 上板发送消息 && 下板接收消息

```
    bool robot_state;           //使能状态[0：1]
    ChassisAction chassis_state;//底盘状态[2:4]
    float Vx;                   //前后速度[8:24]
    float Vy;                   //左右速度[24:40]
    float motor_yaw_angle;      //云台偏航角度[40:56]
```


|              域              | 偏移 | 长度(bits) |                                    范围                                    |
| :---------------------------: | :--: | :--------: | :-------------------------------------------------------------------------: |
|          robot_state          |  0  |     1     |                         布尔，0代表失能 / 1代表使能                         |
| chassis_state (ChassisAction) |  2  |     2     | 枚举，2 位(0 ~ 3) 0为独立控制<br /> 1为底盘跟随 <br />2为小陀螺 |
|              Vx              |  8  |     16     |                     float 转`int16_t` ,范围：+10 ~ -10                     |
|              Vy              |  24  |     16     |                     float 转`int16_t` ,范围：+10 ~ -10                     |
|        motor_yaw_angle        |  40  |     16     |                    float 转`int16_t`，示例范围 -pi ~ +pi                    |

# 使用介绍

## 前置工作

* 使用前先根据板子的上下在***robot_config.h**中进行定义操作，若为上板,则添加

```
#define BOARD_UP
```

* 若为下板，则添加

```
#define BOARD_DOWN
```

## 配置与使用

### 配置示例

#### 上板

```
communicateInitConfig_s Ccb_Config ={
        .topic_name = "ccb_up",
        .can_config = {
            .can_number = 1,
            .rx_id = 0x91,
            .tx_id = 0x78,
        },
        .parent_ptr_gimbal = Gimbal,
        .parent_ptr_state = robot_state,
    };
```

#### 下板

~~~
static communicateInitConfig_s Ccb_config ={
	.can_config = {
		.can_number = 2,
		.rx_id = 0x78,
		.tx_id = 0x91,
	},
	.topic_name = "Ccb_down",

};
~~~

### 使用

接收与消息处理已经在回调中处理，只需在合适时进行发送即可

~~~
Communicate_Transmit(Ccb_up)
osDelay（1）；
~~~
