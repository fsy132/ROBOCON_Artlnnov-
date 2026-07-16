# 2026-06-14 模块拆分记录

> **代码身份标识：机器人比赛 R2 机器人代码。**
>
> 后续如果同时调试 R1 和 R2，请先确认当前对话、工程目录和文档标题里的机器人编号。本文件对应 `R2_Computer`，不是 R1。

`Core/Src/main.cpp` 已经不再继续堆放所有新增逻辑，动作组、光流距离闭环、HWT101 陀螺仪和主要配置已拆到 `MDK-ARM/app` 目录，方便后期单独调试。

## 当前新增模块

| 文件 | 作用 |
| --- | --- |
| `app/app_config.h` | 集中放项目身份、底盘参数、升降台圈数、侧轮参数、MTF 光流编号、HWT101 参数、动作组距离和超时参数 |
| `app/app_shared.h` | 声明跨模块共享的电机对象、状态变量、动作组枚举 |
| `app/hwt101.h/.cpp` | UART7 维特 HWT101CT-TTL 陀螺仪解析、yaw 获取、yaw 清零、陀螺仪辅助转向 |
| `app/distance_control.h/.cpp` | 1 号/2 号 MTF-01P 光流距离闭环，包含底盘距离闭环和 2006 侧轮距离闭环 |
| `app/action_group.h/.cpp` | 动作组状态机；当前 `ACTION_FB_ENABLE = 0`，不再通过 UART3 回传动作状态 |
| `app/protocol.h/.c` | 上位机生成的二进制协议收发、CRC8、消息结构体，当前绑定 USART3 |

## main.cpp 现在保留的职责

`main.cpp` 目前主要保留：

1. HAL 初始化和 `main()` 调度循环。
2. USART3 上位机指令接收和解析入口。
3. CAN1/CAN2 电机反馈回调。
4. 底盘四个万向轮控制任务。
5. 升降台 1/2/3 控制任务。
6. 侧轮和角度机构控制任务。
7. USART3/UART2 调试反馈。
8. 全局对象和全局状态变量定义。

## Keil 工程同步

`R2_SBUS.uvprojx` 已加入：

```text
IncludePath: ./app

app group:
  app/hwt101.cpp
  app/distance_control.cpp
  app/action_group.cpp
```

## 动作组指令和状态

上位机通过 USART3 发送 `$ACT,id` 启动动作组；发送 `$ACT,0` 或 `$STOP` 停止当前动作并回到空闲状态。

通信周期统一按 50Hz 设计，也就是 20ms 一帧。UART 接收使用中断逐字节接收，不需要轮询；上位机控制类指令建议按 20ms 周期连续发送。

```text
$ACT,1   一阶向上爬前半段不吸取
$ACT,2   二阶向上爬前半段不吸取
$ACT,3   一阶向下爬
$ACT,4   二阶向下爬
$ACT,5   低处往高处吸取方块
$ACT,6   高处往低处吸取方块
$ACT,8   存放方块
$ACT,9   梅林放置方块
$ACT,10  取出存放方块
$ACT,11  向上爬后半段，一阶/二阶通用
$ACT,13  一阶向上爬前半段并吸取方块
$ACT,14  二阶向上爬前半段并吸取方块
$ACT,15  原地舵机夹取矛头并翻转上摆，底盘锁定，不启动距离闭环
$ACT,16  释放武器端头，夹爪先松开，然后翻转舵机下垂
$ACT,17  对抗区放置方块
$ACT,0   停止动作组
$STOP    停止当前动作并回到空闲状态
```

注意：手动 ASCII `$ACT,6` 仍然是“高处往低处吸取方块”。上位机二进制 `ActionGroupCmd(action_id=6)` 按上位机协议定义为“底盘锁死”，下位机会只执行底盘强制锁定，不会再透传成本地 `$ACT,6`。

当前下位机不再回传动作状态：

```text
ACTION_FB_ENABLE = 0
```

当前为纯 UART3 上位机控制测试配置：

```text
RC_SBUS_ENABLE = 0
```

该配置下遥控器不会覆盖 `$CHS/CmdVel` 底盘速度，方便直接用电脑串口测试 `$CHS,500,0,0`。

`status` 含义：

```text
0 = 未执行动作
1 = 执行动作中
2 = 执行动作完成
3 = 执行动作失败
```

方块动作均通过气缸伸出机构和真空泵/吸盘完成：PI6 置为高电平会触发继电器驱动气缸，让吸盘伸出更远；PI7 是真空泵/吸盘输出；PI5 用于方块存取流程里的吸取输出。只有 `$ACT,15` 使用 UART4 舵机夹取矛头。`MX_GPIO_Init()` 会把 PI5、PI6、PI7 初始置为低电平；`$STOP` 会停止动作组，并把 PI5/PI6/PI7 拉低。

### `$ACT,1` 一阶向上爬前半段不吸取

```text
1. 使用 1 号光流，底盘向前走 200 mm。
2. 升降台1到第一档高度，升降台2到第一档高度。
3. 等待 ACTION_LIFT_WAIT_MS。
4. 使用 1 号光流，底盘和四个 2006 侧轮一起向前走 150 mm。
5. 升降台1回到零点。
6. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,13` 一阶向上爬前半段并吸取方块

```text
1. 使用 1 号光流，底盘向前走 200 mm。
2. 升降台1到第一档高度，升降台2到第一档高度，升降台3抬升到 10 cm 档位。
3. 等待 ACTION_LIFT_WAIT_MS。
4. PI6 置为高电平，触发继电器驱动气缸伸出吸盘。
5. PI7 置为高电平，开启真空泵/吸盘吸取。
6. 使用 1 号光流，底盘和四个 2006 侧轮一起向前走 150 mm。
7. 升降台1回到零点。
8. 等待 ACTION_GPIO_WAIT_MS。
9. 升降台3抬升到 40 cm 档位。
10. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,11` 向上爬后半段，一阶/二阶通用

```text
1. 使用 2 号光流，底盘和四个 2006 侧轮一起向前走 500 mm。
2. 升降台2回到零点。
3. 等待 ACTION_LIFT_WAIT_MS。
4. 使用 2 号光流，底盘向前走 200 mm。
5. 底盘 200 mm 完成后状态变为完成。
```

### `$ACT,2` 二阶向上爬前半段不吸取

```text
1. 使用 1 号光流，底盘向前走 200 mm。
2. 升降台1到第二档高度，升降台2到第二档高度。
3. 等待 ACTION_LIFT_WAIT_MS。
4. 使用 1 号光流，底盘和四个 2006 侧轮一起向前走 150 mm。
5. 升降台1回到零点。
6. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,14` 二阶向上爬前半段并吸取方块

```text
1. 使用 1 号光流，底盘向前走 200 mm。
2. 升降台1到第二档高度，升降台2到第二档高度，升降台3抬升到 10 cm 档位。
3. 等待 ACTION_LIFT_WAIT_MS。
4. PI6 置为高电平，触发继电器驱动气缸伸出吸盘。
5. PI7 置为高电平，开启真空泵/吸盘吸取。
6. 使用 1 号光流，底盘和四个 2006 侧轮一起向前走 150 mm。
7. 升降台1回到零点。
8. 等待 ACTION_GPIO_WAIT_MS。
9. 升降台3抬升到 40 cm 档位。
10. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,15` 原地舵机夹取矛头

```text
1. 立即强制锁定底盘，清零底盘速度和内部速度斜坡，并记录当前轮角作为保持目标。
2. 锁定后前 `CHASSIS_FORCE_LOCK_IGNORE_MS`，当前 80ms，丢弃上位机速度指令，过滤串口队列里的旧速度。
3. 不启动底盘距离闭环，不主动发送底盘前进命令。
4. 通过 UART4 控制夹爪舵机 ID 1 到 `GRIPPER_SERVO_GRIP_POS`，用时 `GRIPPER_SERVO_GRIP_TIME_MS`，执行矛头夹取。
5. 等待 `GRIPPER_PICK_FLIP_DELAY_MS`，当前为 `GRIPPER_SERVO_GRIP_TIME_MS + GRIPPER_PICK_FLIP_EXTRA_DELAY_MS`，也就是夹爪运动完成后额外等待 1500ms。
6. 翻转舵机到上摆位置 `FLIP_SERVO_UP_POS`。
7. 等待 `GRIPPER_SERVO_GRIP_TIME_MS` 和 `GRIPPER_PICK_FLIP_DELAY_MS + FLIP_SERVO_TIME_MS` 中较大的时间后，动作状态变为完成。
8. 底盘继续保持锁定，动作运行中遥控器不会解锁；动作完成后，收到非 0 的 `$CHS`、上位机 CmdVel，或打开遥控器接管后才恢复运动。
```

### `$ACT,16` 释放武器端头

```text
1. 立即强制锁定底盘，清零底盘速度和内部速度斜坡。
2. 通过 UART4 控制夹爪舵机 ID 1 到 `GRIPPER_SERVO_RELEASE_POS`，用时 `GRIPPER_SERVO_RELEASE_TIME_MS`，先松开夹爪。
3. 等待 `GRIPPER_SERVO_RELEASE_TIME_MS`。
4. 通过 UART4 控制翻转舵机 ID 2 到下垂位置 `FLIP_SERVO_DOWN_POS`，用时 `FLIP_SERVO_TIME_MS`。
5. 等待 `GRIPPER_SERVO_RELEASE_TIME_MS + FLIP_SERVO_TIME_MS` 后动作状态变为完成。
6. 上位机二进制 `ActionGroupCmd(action_id=2)` 会映射到该动作。
```

### `$ACT,3` 一阶向下爬

```text
1. 使用 1 号光流，底盘向后走 300 mm。
2. 升降台2到第一档高度。
3. 等待 ACTION_LIFT_WAIT_MS。
4. 使用 2 号光流，底盘向后走 300 mm。
5. 升降台1到第一档高度。
6. 等待 ACTION_LIFT_WAIT_MS。
7. 使用 1 号光流，四个 2006 侧轮向后走 200 mm。
8. 升降台1和升降台2回到零点。
9. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,4` 二阶向下爬

```text
1. 使用 1 号光流，底盘向后走 300 mm。
2. 升降台2到第二档高度。
3. 等待 ACTION_LIFT_WAIT_MS。
4. 使用 2 号光流，底盘向后走 300 mm。
5. 升降台1到第二档高度。
6. 等待 ACTION_LIFT_WAIT_MS。
7. 使用 1 号光流，四个 2006 侧轮向后走 200 mm。
8. 升降台1和升降台2回到零点。
9. 等待 ACTION_LIFT_WAIT_MS 后状态变为完成。
```

### `$ACT,5` 低处往高处吸取方块

```text
1. 升降台3抬升到 40 cm 档位。
2. 等待 ACTION_LIFT_WAIT_MS。
3. PI6 置为高电平，触发继电器驱动气缸伸出吸盘。
4. 等待 ACTION_GPIO_WAIT_MS。
5. PI7 置为高电平，开启真空泵/吸盘吸取。
6. 状态变为完成。
```

### `$ACT,6` 高处往低处吸取方块

```text
1. 升降台3抬升到 10 cm 档位。
2. 等待 ACTION_LIFT_WAIT_MS。
3. PI6 置为高电平，触发继电器驱动气缸伸出吸盘。
4. 等待 ACTION_GPIO_WAIT_MS。
5. PI7 置为高电平，开启真空泵/吸盘吸取。
6. 状态变为完成。
```

### `$ACT,8` 存放方块

```text
1. 升降台3抬升到 40 cm 档位。
2. 等待 ACTION_LIFT_WAIT_MS。
3. motor12 逆时针旋转 270 度。
4. 等待 ACTION_MOTOR_WAIT_MS。
5. PI7 置为低电平，关闭真空泵/吸盘吸取。
6. 等待 ACTION_GPIO_WAIT_MS。
7. PI6 置为低电平，收回气缸伸出机构。
8. 升降台3下降/移动到 20 cm 档位。
9. 等待 ACTION_LIFT_WAIT_MS。
10. PI5 置为低电平。
11. motor12 顺时针恢复到零点。
12. 等待 ACTION_MOTOR_WAIT_MS 后状态变为完成。
```

### `$ACT,9` 梅林放置方块

```text
1. 升降台1和升降台2同时抬到第三档，升降台3抬升到 60 cm 档位。
2. 等待 ACTION_LIFT_WAIT_MS。
3. motor12 逆时针旋转 90 度。
4. PI6 置为高电平。
5. 2 秒后 PI5 置为高电平。
6. 3 秒后 PI7 置为低电平。
7. 等待 ACTION_MOTOR_WAIT_MS 后状态变为完成。
```

### `$ACT,17` 对抗区放置方块

```text
1. 升降台1和升降台2同时抬到第三档，升降台3抬升到 20 cm 档位。
2. 等待 ACTION_LIFT_WAIT_MS。
3. motor12 逆时针旋转 90 度。
4. PI5 置为高电平。
5. 3 秒后 PI7 置为低电平。
6. 等待 ACTION_MOTOR_WAIT_MS 后状态变为完成。
```

### `$ACT,10` 取出存放方块

```text
1. motor12 逆时针旋转 270 度。
2. 等待 ACTION_MOTOR_WAIT_MS。
3. 升降台3抬升到 10 cm 档位。
4. 等待 ACTION_LIFT_WAIT_MS。
5. PI5 置为高电平。
6. motor12 顺时针旋转到 180 度位置。
7. 等待 ACTION_MOTOR_WAIT_MS 后状态变为完成。
```

## 光流模块使用约定

```text
1 号光流：车中间，USART6，代码编号 MTF01_MIDDLE_INDEX
2 号光流：车头，UART8，代码编号 MTF01_FRONT_INDEX
```

光流模块切换由下位机动作组自动完成，不需要上位机额外发送切换指令。

## 后续建议继续拆分

当前已经完成第一阶段拆分。后续如果继续减小 `main.cpp`，建议按这个顺序拆：

1. `chassis_control.h/.cpp`：底盘万向轮运动学、yaw hold、速度反馈。
2. `lift_control.h/.cpp`：升降台 1/2/3 的零点、目标角、控制任务。
3. `side_wheel.h/.cpp`：2006 侧轮速度控制和 2006 角度机构。
4. `host_protocol.h/.cpp`：上位机串口指令解析和反馈。

因为这些部分仍大量使用 C++ 电机类，所以建议继续使用 `.cpp/.h`，不要强行改成 `.c/.h`。


# R2_Computer A板1 下位机使用手册

本文档用于说明 `R2_Computer` 工程的项目内容。后期开新对话时上传本文档，可以让协作者快速了解当前下位机代码结构、CAN 电机分配、上位机串口指令和常用参数修改位置。

## 1. 工程定位

`R2_Computer` 是 ROBOCON2026 R2 全自动机器人下位机工程之一。

```text
工程目录：
  C:\Users\27559\Desktop\R2_Computer

核心文件：
  Core/Src/main.cpp

工具链：
  Keil + VSCode + STM32CubeMX

开发库：
  STM32 HAL

主控板：
  大疆 A板

控制入口：
  电脑通过 USB 转 TTL 连接 USART3（PD8/PD9）

辅助调试：
  大疆 A板 USART1 自带反相电路，可接 SBUS 遥控器用于升降台手动点动和调零救援

预留接口：
  UART8 接 2 号 MTF-01P 光流模块，UART4 接 BusLinker 舵机总线
```

工程身份宏位于 `Core/Src/main.cpp` 的 `USER CODE BEGIN PD` 区域：

```cpp
#define R2_PROJECT_SBUS        0
#define R2_PROJECT_COMPUTER    1
#define R2_PROJECT_UP1         0

#define R2_PROJECT_NAME        "R2_Computer"
#if (R2_PROJECT_SBUS + R2_PROJECT_COMPUTER + R2_PROJECT_UP1) != 1
#error "Exactly one R2_PROJECT_xxx macro must be set to 1."
#endif
```

判断方式：

```text
R2_PROJECT_COMPUTER = 1 表示当前文件属于 R2_Computer。
R2_PROJECT_SBUS = 1 表示遥控器 SBUS 工程。
R2_PROJECT_UP1 = 1 表示最终烧录到 A板的整合工程。
```

## 2. main.cpp 结构

`main.cpp` 按下面顺序阅读和维护：

```text
1. Include 区
2. 工程身份和宏定义
3. 全局变量
4. 函数声明
5. USART3 接收中断回调
6. CAN1/CAN2 电机反馈回调
7. USART3 指令解析
8. 底盘控制
9. 升降台控制
10. 侧轮和角度机构控制
11. 反馈输出
12. motor_pid_init()
13. motor_can_init()
14. motor_pid_task()
15. main()
```

`main()` 只做调度，控制逻辑放在各任务函数中。

```text
USART3 收指令
  -> uart6_cmd_parse()
  -> 各机构 task 更新目标
  -> motor_pid_task() 计算 PID
  -> TIM_CAN_PeriodElapsedCallback() 发送 CAN
  -> USART3 输出状态和光流调试反馈
```

## 3. CAN1 电机分配

CAN1 负责底盘四个 3508、三个 2006 侧轮，以及 motor12 角度机构。

| 电机编号 | CAN ID | 电机类型 | 机构位置/用途 | 控制方式 | USART3 指令 |
| --- | --- | --- | --- | --- | --- |
| `motor1` | `0x201` | 3508/C620 | 底盘左前万向轮 | 角度闭环 | `$CHS` |
| `motor2` | `0x202` | 3508/C620 | 底盘左后万向轮 | 角度闭环 | `$CHS` |
| `motor3` | `0x203` | 3508/C620 | 底盘右后万向轮 | 角度闭环 | `$CHS` |
| `motor4` | `0x204` | 3508/C620 | 底盘右前万向轮 | 角度闭环 | `$CHS` |
| `motor9` | `0x205` | 2006/C610 | 侧轮1/普通橡胶轮 | 速度闭环 | `$SW`/动作组 |
| `motor10` | `0x206` | 2006/C610 | 侧轮2/普通橡胶轮 | 速度闭环 | `$SW`/动作组 |
| `motor11` | `0x207` | 2006/C610 | 侧轮3/普通橡胶轮 | 速度闭环 | `$SW`/动作组 |
| `motor12` | `0x208` | 3508/C620 | CAN1 角度机构 | 角度闭环 | `$A12` |

CAN1 回调入口：

```text
CAN1_Call_Back()
```

CAN1 初始化入口：

```text
motor_can_init()
```

## 4. CAN2 电机分配

CAN2 负责三个升降台和 motor15 侧轮。

| 电机编号 | CAN ID | 电机类型 | 机构位置/用途 | 控制方式 | USART3 指令 |
| --- | --- | --- | --- | --- | --- |
| `motor5` | `0x201` | 3508/C620 | 升降台1 电机A | 角度闭环 | `$L1` |
| `motor6` | `0x202` | 3508/C620 | 升降台1 电机B | 角度闭环 | `$L1` |
| `motor7` | `0x203` | 3508/C620 | 升降台2 电机A | 角度闭环 | `$L2` |
| `motor8` | `0x204` | 3508/C620 | 升降台2 电机B | 角度闭环 | `$L2` |
| `motor15` | `0x205` | 2006/C610 | 侧轮4/普通橡胶轮 | 速度闭环 | `$SW`/动作组 |
| `motor13` | `0x206` | 3508/C620 | 升降台3 电机A | 角度闭环 | `$L3` |
| `motor14` | `0x207` | 3508/C620 | 升降台3 电机B | 角度闭环 | `$L3` |

CAN2 回调入口：

```text
CAN2_Call_Back()
```

CAN2 初始化入口：

```text
motor_can_init()
```

## 5. 上位机串口指令表

当前 `USART3` 是 `R2_Computer` 的唯一主控入口。

USART3 同时兼容两种格式：调试用 ASCII 指令，以及上位机二进制协议帧。二进制协议当前按下面格式解析：

```text
0x5A 0xA5 id len payload crc8
```

CRC8 使用多项式 `0x31`，计算范围为 `id/len/payload`。多字节字段按小端解析。

```text
Heartbeat: id=0x00, payload=count(u32)，上位机 1Hz 发送，下位机回发同 count 作为 ACK。
Handshake: id=0xFF, payload=protocol_hash(u32)，上位机握手 3 次；下位机收到后回发同 id 和同 hash，最多回 3 次。
CmdVel: id=0x01, payload=vx(f32),vy(f32),rotation(f32)，单位按上位机协议为 m/s、m/s、deg/s；vx 表示前进速度，vy 表示左移速度，下位机转换成内部底盘速度控制量。上位机应连续发送移动指令，当前 `CHASSIS_CMD_TIMEOUT_MS = 300ms`，超过该时间没有新 CmdVel 时下位机会自动清零底盘速度。
ConfrontClimbCmd: id=0x0B, payload=mode(u8)。`mode=1` 进入对抗区入口爬坡模式，下位机切换到爬坡 PID；`mode=0` 退出爬坡模式，下位机恢复普通底盘 PID。该模式会保持到下一次 `ConfrontClimbCmd` 修改。
ActionGroupCmd: id=0x09, payload=action_id(u8)。上位机二进制动作 ID 会先经过下位机映射；当前 `action_id=1` 映射为本地 `ACTION_ID_GRIPPER_PICK=15`，用于原地舵机夹取矛头并让翻转舵机上摆；`action_id=2` 映射为本地 `ACTION_ID_GRIPPER_RELEASE=16`，夹爪先松开，然后翻转舵机下垂；`action_id=6` 是上位机底盘锁死指令，只锁定底盘，不启动本地 `$ACT,6`。
WeaponFlipCmd: id=0x0A, payload=flip(u8)。`flip=0` 下垂，`flip=1` 摆平，`flip=2` 上摆；下位机收到后等价于执行 `$FLIP,0/1/2`，会锁住底盘。
ActionGroupFeedback: 当前 `ACTION_FB_ENABLE = 0`，下位机不再回传动作状态。
WheelOdom: id=0x25, payload=pos_x(f32),pos_y(f32),yaw(f32),vx(f32),vy(f32),omega(f32)。当前 `CHASSIS_VEL_FB_ENABLE = 1`，下位机按最新上位机协议回传完整 WheelOdom 帧；回传周期由 `CHASSIS_VEL_FB_PERIOD_MS` 控制，当前为 100ms/10Hz。`pos_x/pos_y/yaw` 为下位机累计里程计，`vx` 为前进速度，`vy` 为左移速度，`omega` 为当前角速度。动作组和舵机夹取运行中也会持续以 10Hz 回传 WheelOdom。为避免轮子残余转速、刹车回弹和编码器抖动影响上位机，`vx/vy` 小于 `CHASSIS_VEL_FB_DEADBAND_M_S` 时上报 0；底盘强制锁定时 `CHASSIS_VEL_FB_LOCK_ZERO = 1` 会强制 `vx/vy/omega` 上报 0。
```

这部分对应 `protocol.yaml` 和上位机生成的 `protocol.c/.h`：当前 `PROTOCOL_HASH = 0x2DD00572`，USART3 波特率 `115200`，帧头 `0x5A 0xA5`，校验 `CRC8`，`require_handshake=false`，`enable_heartbeat=true`，心跳超时建议按 `3000ms` 判断。当前下位机收到心跳后回 ACK；收到握手后回发本地下位机 `PROTOCOL_HASH`，主循环中最多缓存 3 帧握手回复。

联调时建议先只发二进制握手帧，确认 USART3 能收到下位机回包；再以 1Hz 发送心跳帧。ASCII 调试指令仍然保留，二进制帧被解析后不会进入 `$CHS/$L1/...` 文本指令解析。

`protocol.c/.h` 已放在 `MDK-ARM/app/` 并加入 Keil 工程。下次上位机重新生成协议文件后，覆盖这两个文件时需要保留下位机兼容修改：

```text
1. protocol.c 中 weak 回调用 Keil ARMCC 写法：__weak，而不是 __attribute__((weak))。
2. protocol.h 用 #ifdef __cplusplus extern "C" 包住函数声明，方便 main.cpp/action_group.cpp 调用。
3. 如果 Keil 把 protocol.c 按 C++ 方式处理，protocol.c 顶部和底部也保留 extern "C" 包裹。
```

下位机实际控制逻辑不直接写在 `protocol.c` 里，而是在 `main.cpp` 覆盖这些回调：

```text
on_receive_Heartbeat()
on_receive_Handshake()
on_receive_CmdVel()
on_receive_ActionGroupCmd()
```

| 指令 | 控制对象 | 操作 |
| --- | --- | --- |
| `$CHS,forward_mm_s,left_mm_s,wz_mdeg_s` | `motor1~motor4` | 控制底盘平移和自转 |
| `$TURN,angle_deg` | `motor1~motor4` + HWT101 | 使用陀螺仪相对转指定角度后停止 |
| `$L1,cmd` | `motor5~motor6` | 控制升降台1 回零/上升 |
| `$L2,cmd` | `motor7~motor8` | 控制升降台2 回零/上升 |
| `$L12,cmd` | `motor5~motor8` | 同时控制升降台1和2 回零/上升 |
| `$L3,cmd` | `motor13~motor14` | 控制升降台3 回零/一阶抬升/二阶抬升 |
| `$ZERO,id` | `motor5~motor8` | 临时把当前位置记录为升降台1/2零点，仅调试用 |
| `$SW,enable,rpm` | `motor9/motor10/motor11/motor15` | 调试侧轮目标转速 |
| `$A12,angle_mdeg` | `motor12` | 控制 CAN1 3508 角度机构 |
| `$GRIP,1` | `UART4` 总线舵机 | 夹爪夹取，实际位置和时间在 `app_config.h` 中调 |
| `$GRIP,0` | `UART4` 总线舵机 | 夹爪松开，实际位置和时间在 `app_config.h` 中调 |
| `$FLIP,0` | `UART4` 总线舵机 | ID=2 翻转舵机到下垂位置 842 |
| `$FLIP,1` | `UART4` 总线舵机 | ID=2 翻转舵机到摆平位置 489 |
| `$FLIP,2` | `UART4` 总线舵机 | ID=2 翻转舵机到上摆位置 119 |
| `$PI5,level` | `PI5` | 方块存取流程里的吸取输出，level=1 开启，level=0 关闭 |
| `$PI6,level` | `PI6` | 继电器/气缸伸出机构，level=1 开启，level=0 关闭 |
| `$PI7,level` | `PI7` | 真空泵/吸盘输出，level=1 开启，level=0 关闭 |
| `$SWD,distance_mm` | `motor9/motor10/motor11/motor15` + MTF-01P | 光流闭环前进指定距离 |
| `$STOP` | 全部机构 | 底盘停、侧轮停、升降台回零、角度机构回零 |

指令解析函数：

```text
uart6_cmd_parse(char *line)
```

## 6. 指令参数说明

### `$CHS,forward_mm_s,left_mm_s,wz_mdeg_s`

```text
forward_mm_s:
  前后速度，单位 mm/s，正数向前，负数向后。

left_mm_s:
  左右速度，单位 mm/s，正数向左，负数向右。

wz_mdeg_s:
  自转角速度，单位 0.001 deg/s，正数逆时针，负数顺时针。
```

示例：

```text
$CHS,500,0,0      向前 500 mm/s
$CHS,-500,0,0     向后 500 mm/s
$CHS,0,500,0      向左 500 mm/s
$CHS,0,-500,0     向右 500 mm/s
$CHS,0,0,90000    逆时针 90 deg/s
$CHS,0,0,-90000   顺时针 90 deg/s
```

`$CHS` 是速度指令。如果上位机 50Hz 连续发送 `$CHS,0,0,90000`，车会一直以 90 deg/s 逆时针旋转。要转指定角度后停止，使用 `$TURN`。

### `$TURN,angle_deg`

```text
angle_deg:
  相对当前车头方向的目标旋转角度，单位 deg。
  正数逆时针，负数顺时针。
```

示例：

```text
$TURN,90     逆时针转 90 度后停止
$TURN,-90    顺时针转 90 度后停止
```

`$TURN` 依赖 UART7 的 HWT101 陀螺仪。如果陀螺仪没有有效 yaw 数据，指令不会启动定角旋转。

### `$L1,cmd` / `$L2,cmd` / `$L12,cmd` / `$L3,cmd`

```text
cmd = 0:
  回到上电零点。

cmd = 1:
  升降台到第一档高度，也就是低位高度。

cmd = 2:
  升降台到第二档高度，也就是高位高度。

cmd = 3:
  升降台1/2到第三档高度。`$L3,3` 表示升降台3到 10cm 档位。
```

当前升降台1、升降台2使用三档高度；升降台3使用 0~6 档位。

示例：

```text
$L1,0   升降台1回零
$L1,1   升降台1到第一档高度
$L1,2   升降台1到第二档高度
$L1,3   升降台1到第三档高度

$L2,0   升降台2回零
$L2,1   升降台2到第一档高度
$L2,2   升降台2到第二档高度
$L2,3   升降台2到第三档高度

$L12,1  升降台1和2同时到第一档高度
$L12,2  升降台1和2同时到第二档高度
$L12,3  升降台1和2同时到第三档高度

$L3,0   升降台3回零
$L3,1   升降台3到第一档高度
$L3,2   升降台3到第二档高度

$ZERO,1   临时记录升降台1当前位置为零点
$ZERO,2   临时记录升降台2当前位置为零点
$ZERO,12  临时记录升降台1和2当前位置为零点
```

`$ZERO,id` 只用于台架调试。正式运行中升降台1/2会在开机后自动向上抬升2cm并记录当前位置为零点。

### `$GRIP` 夹爪舵机

```text
$GRIP,1:
  夹爪夹取。实际发送 ID=1，位置为 GRIPPER_SERVO_GRIP_POS，时间为 GRIPPER_SERVO_GRIP_TIME_MS。

$GRIP,0:
  夹爪松开。实际发送 ID=1，位置为 GRIPPER_SERVO_RELEASE_POS，时间为 GRIPPER_SERVO_RELEASE_TIME_MS。
```

上位机只需要发 `$GRIP,1` 和 `$GRIP,0`。夹爪的实际位置和时间都在 `app_config.h` 里调。

### `$FLIP` 三位置翻转舵机

```text
$FLIP,0:
  控制 ID=2 的翻转舵机到下垂位置，位置参数为 FLIP_SERVO_DOWN_POS，当前值 842。

$FLIP,1:
  控制 ID=2 的翻转舵机到摆平位置，位置参数为 FLIP_SERVO_FLAT_POS，当前值 489。

$FLIP,2:
  控制 ID=2 的翻转舵机到上摆位置，位置参数为 FLIP_SERVO_UP_POS，当前值 119。
```

翻转舵机动作时间由 `FLIP_SERVO_TIME_MS` 决定。如果实际角度有偏差，直接微调 `FLIP_SERVO_DOWN_POS`、`FLIP_SERVO_FLAT_POS`、`FLIP_SERVO_UP_POS`。

当前上电默认姿态为：翻转舵机下垂，夹爪舵机打开。

`FLIP_SERVO_BOOT_DOWN_ENABLE = 1` 时，下位机上电初始化完成后会延时 `FLIP_SERVO_BOOT_DELAY_MS`，然后按 `FLIP_SERVO_BOOT_RETRY_PERIOD_MS` 周期重复发送下垂位置 `FLIP_SERVO_DOWN_POS`。当前 `FLIP_SERVO_BOOT_REPEAT_FOREVER = 0`，`FLIP_SERVO_BOOT_RETRY_COUNT = 16`，所以上电后约 5 秒内重复发送下垂指令，然后自动停止。

`GRIPPER_SERVO_BOOT_RELEASE_ENABLE = 1` 时，下位机上电初始化完成后会延时 `GRIPPER_SERVO_BOOT_DELAY_MS`，然后按 `GRIPPER_SERVO_BOOT_RETRY_PERIOD_MS` 周期重复发送夹爪打开位置 `GRIPPER_SERVO_RELEASE_POS`。当前 `GRIPPER_SERVO_BOOT_REPEAT_FOREVER = 0`，`GRIPPER_SERVO_BOOT_RETRY_COUNT = 16`，所以上电后约 5 秒内重复发送夹爪打开指令，然后自动停止。

这样可以避免舵机或 BusLinker 刚上电还没准备好导致第一帧丢失。这些上电初始化指令不会强制锁住底盘；如果期间手动发送 `$FLIP` 或执行 `$ACT,15`，会取消后续翻转舵机上电重发；如果期间手动发送 `$GRIP` 或执行 `$ACT,15`，会取消后续夹爪上电重发。

执行 `$GRIP` 或 `$FLIP` 时下位机会立刻锁住底盘：清零底盘速度、清零内部速度斜坡，记录当前轮角作为保持目标，并在底盘最终输出层拦截所有来源的底盘速度，包括内部距离闭环写入的速度。锁定后前 `CHASSIS_FORCE_LOCK_IGNORE_MS`，当前 80ms，会丢弃所有 `$CHS/CmdVel`，用于过滤串口队列里的旧速度；之后仍保持锁定，但不会反复把目标角刷新成当前角度，所以轮子应有保持力。动作运行中遥控器不会解锁，动作完成后收到非 0 的 `$CHS`、上位机速度指令，或打开遥控器接管后才解锁。

UART4 总线舵机发送使用非阻塞队列：`bus_servo_move_time_write()` 只把 10 字节舵机帧放入队列，实际发送由 `HAL_UART_Transmit_IT()` 和 `HAL_UART_TxCpltCallback()` 完成，不再阻塞底盘 1ms 控制循环。队列深度由 `BUS_SERVO_TX_QUEUE_SIZE` 控制。

```text
$GRIP,1
$GRIP,0
$FLIP,0
$FLIP,1
$FLIP,2
```

如果指令没反应，优先检查 UART4 接线、BusLinker 控制模式、电源、共地、舵机 ID 是否分别为 1 和 2。

### `$SW,enable,rpm`

```text
enable = 0:
  四个侧轮停止，目标速度置 0。

enable = 1:
  四个侧轮进入速度闭环，目标转速为 rpm。

rpm:
  目标转速，单位 rpm。正负号决定前进/后退方向。
```

侧轮方向由 `SIDE_WHEEL9_DIR`、`SIDE_WHEEL10_DIR`、`SIDE_WHEEL11_DIR`、`SIDE_WHEEL15_DIR` 决定。`$SW` 是调试用速度指令，动作组和 `$SWD` 会直接给侧轮速度闭环输出目标速度。

示例：

```text
$SW,1,300    四个侧轮按 300 rpm 运行
$SW,1,-300   四个侧轮按 -300 rpm 反向运行
$SW,0,0      四个侧轮停止
```

### `$A12,angle_mdeg`

```text
angle_mdeg:
  motor12 相对上电零点的目标角度。
  单位是 0.001 度。
```

示例：

```text
$A12,90000     motor12 转到 +90 度
$A12,-45000    motor12 转到 -45 度
$A12,0         motor12 回零
```

下位机内部换算：

```cpp
motor12_target_angle_rad = angle_mdeg * MDEG_TO_RAD;
```

## 7. 常用参数修改位置

### 升降台上升高度

位置：`MDK-ARM/app/app_config.h`。

```cpp
#define LIFT_ROUNDS_TO_ANGLE(rounds) ((rounds) * 6.283185307f)
#define LIFT1_LOW_ROUNDS         2.0f
#define LIFT1_HIGH_ROUNDS        5.0f
#define LIFT2_LOW_ROUNDS         2.0f
#define LIFT2_HIGH_ROUNDS        5.0f
#define LIFT3_LOW_ROUNDS         2.0f
#define LIFT3_HIGH_ROUNDS        5.0f
#define LIFT3_10CM_ROUNDS        1.0f
#define LIFT3_20CM_ROUNDS        2.0f
#define LIFT3_40CM_ROUNDS        4.0f
#define LIFT3_60CM_ROUNDS        6.0f
```

含义：

```text
1.0f 表示电机输出轴目标转 1 圈。
2.0f 表示电机输出轴目标转 2 圈。
5.0f 表示电机输出轴目标转 5 圈。
```

调试时先用小值：

```cpp
#define LIFT1_LOW_ROUNDS         0.2f
```

确认方向和机械限位安全后，再改成实际值。

`LIFT3_10CM_ROUNDS`、`LIFT3_20CM_ROUNDS`、`LIFT3_40CM_ROUNDS`、`LIFT3_60CM_ROUNDS` 是升降台3离地高度的标定值。现在先用圈数占位，实车调试后按实际高度修正这些宏即可。

### 升降台电机方向

位置：`app/app_config.h`。

```cpp
#define LIFT_MOTOR_CCW_DIR        1.0f
#define LIFT_MOTOR_CW_DIR        -1.0f

#define LIFT1_MOTOR5_STAGE_DIR   LIFT_MOTOR_CCW_DIR
#define LIFT1_MOTOR6_STAGE_DIR   LIFT_MOTOR_CCW_DIR
#define LIFT2_MOTOR7_STAGE_DIR   LIFT_MOTOR_CCW_DIR
#define LIFT2_MOTOR8_STAGE_DIR   LIFT_MOTOR_CCW_DIR

#define LIFT1_MOTOR5_HOME_DIR    LIFT_MOTOR_CW_DIR
#define LIFT1_MOTOR6_HOME_DIR    LIFT_MOTOR_CW_DIR
#define LIFT2_MOTOR7_HOME_DIR    LIFT_MOTOR_CW_DIR
#define LIFT2_MOTOR8_HOME_DIR    LIFT_MOTOR_CW_DIR

#define LIFT3_MOTOR13_DIR        1.0f
#define LIFT3_MOTOR14_DIR       -1.0f
```

升降台1/2方向含义：

```text
STAGE_DIR：$L1,1 / $L1,2 / $L2,1 / $L2,2 使用，电机逆时针，升降台下降，车体上升。
HOME_DIR：开机零点校准和回零方向，和 STAGE_DIR 相反。
```

如果实际方向反了，先确认“逆时针是否为下降”，再调整 `LIFT_MOTOR_CCW_DIR` / `LIFT_MOTOR_CW_DIR` 或单个电机的 `STAGE_DIR` / `HOME_DIR`。

### 侧轮速度环参数和方向

位置：`app/app_config.h`。

```cpp
#define SIDE_WHEEL_MAX_RPM        1500.0f
#define SIDE_WHEEL_RPM_TO_RADPS   0.104719755f
#define SIDE_WHEEL_CLIMB_FOLLOW_ENABLE 1
#define SIDE_WHEEL_CLIMB_RPM_PER_MM_S 0.6f
#define SIDE_WHEEL_CLIMB_MIN_MM_S 20.0f
#define SIDE_WHEEL9_DIR           1.0f
#define SIDE_WHEEL10_DIR          1.0f
#define SIDE_WHEEL11_DIR         -1.0f
#define SIDE_WHEEL15_DIR         -1.0f
```

说明：四个侧轮按底盘四轮同位置映射：`motor9/motor10/motor11/motor15` 对应左前/左后/右后/右前，所以方向系数为 `1,1,-1,-1`。`$SW,enable,rpm` 和 `$SWD,distance_mm` 最终都会给四个侧轮设置目标角速度，电机本身使用速度闭环 `Control_Method_OMEGA`。

爬坡模式下，如果没有启用 `$SWD` 光流距离闭环，四个侧轮会自动跟随底盘前后速度：底盘前进时侧轮前进，底盘后退时侧轮后退，左右平移不参与侧轮跟随。跟随比例由 `SIDE_WHEEL_CLIMB_RPM_PER_MM_S` 决定，例如底盘前进 500 mm/s 时侧轮目标约为 300 rpm。

### 角度机构方向和速度

位置：`Core/Src/main.cpp` 宏定义区。

```cpp
#define ANGLE_2006_MAX_OMEGA     12.0f
#define CAN2_2006_ANGLE_DIR      1.0f
```

说明：

```text
ANGLE_2006_MAX_OMEGA 控制 motor12 的角度动作速度上限。
CAN2_2006_ANGLE_DIR 控制 motor12 的角度方向。motor12 实际电机类型是 3508/C620。
```

### 底盘方向

位置：`Core/Src/main.cpp` 宏定义区。

```cpp
#define MOTOR1_DIR               1.0f
#define MOTOR2_DIR               1.0f
#define MOTOR3_DIR               1.0f
#define MOTOR4_DIR               1.0f
```

底盘方向异常时，同时检查：

```text
1. motor1~motor4 CAN ID 是否和实际位置一致
2. MOTOR1_DIR~MOTOR4_DIR 是否正确
3. omni_move() 中四轮解算和实际轮子安装方向是否一致
```


## 8. 升降台两档高度代码改法

升降台1、升降台2、升降台3的命令含义固定为：

```text
$L1,0: 升降台1回零
$L1,1: 升降台1第一档高度
$L1,2: 升降台1第二档高度

$L2,0: 升降台2回零
$L2,1: 升降台2第一档高度
$L2,2: 升降台2第二档高度

$L3,0: 升降台3回零
$L3,1: 升降台3第一档高度
$L3,2: 升降台3第二档高度
```

升降台3的两个电机以刚上电的位置为零点，抬升时 `motor13` 和 `motor14` 一正一反转动：

```cpp
#define LIFT3_MOTOR13_DIR        1.0f
#define LIFT3_MOTOR14_DIR       -1.0f
```

`lift1_control_task()` 的目标角度选择逻辑为：

```cpp
if (lift1_cmd == 1)
{
    target_angle_5 = lift1_zero_angle_5 + LIFT1_MOTOR5_STAGE_DIR * LIFT1_LOW_ANGLE;
    target_angle_6 = lift1_zero_angle_6 + LIFT1_MOTOR6_STAGE_DIR * LIFT1_LOW_ANGLE;
}
else if (lift1_cmd == 2)
{
    target_angle_5 = lift1_zero_angle_5 + LIFT1_MOTOR5_STAGE_DIR * LIFT1_HIGH_ANGLE;
    target_angle_6 = lift1_zero_angle_6 + LIFT1_MOTOR6_STAGE_DIR * LIFT1_HIGH_ANGLE;
}
else
{
    target_angle_5 = lift1_zero_angle_5;
    target_angle_6 = lift1_zero_angle_6;
}
```

`lift2_control_task()` 的目标角度选择逻辑为：

```cpp
if (lift2_cmd == 1)
{
    target_angle_7 = lift2_zero_angle_7 + LIFT2_MOTOR7_STAGE_DIR * LIFT2_LOW_ANGLE;
    target_angle_8 = lift2_zero_angle_8 + LIFT2_MOTOR8_STAGE_DIR * LIFT2_LOW_ANGLE;
}
else if (lift2_cmd == 2)
{
    target_angle_7 = lift2_zero_angle_7 + LIFT2_MOTOR7_STAGE_DIR * LIFT2_HIGH_ANGLE;
    target_angle_8 = lift2_zero_angle_8 + LIFT2_MOTOR8_STAGE_DIR * LIFT2_HIGH_ANGLE;
}
else
{
    target_angle_7 = lift2_zero_angle_7;
    target_angle_8 = lift2_zero_angle_8;
}
```

`uart6_cmd_parse()` 里 `$L1` 和 `$L2` 的解析可以保持不变，因为 `lift1_cmd` 和 `lift2_cmd` 本来就是从命令里读取整数。

### 四个 2006 侧轮和 MTF-01P 距离闭环

`motor9` 和 `motor10` 保持速度控制，用于执行 MTF-01P 距离闭环输出的目标速度。

```text
$SW,enable,rpm:
  调试侧轮速度。

$SWD,distance_mm:
  使用 MTF-01P 光流闭环，让小车前进指定距离。
```

侧轮速度参数：

```cpp
#define SIDE_WHEEL_MAX_RPM       1500.0f
#define SIDE_WHEEL_RPM_TO_RADPS  0.104719755f
#define SIDE_WHEEL_CLIMB_FOLLOW_ENABLE 1
#define SIDE_WHEEL_CLIMB_RPM_PER_MM_S 0.6f
#define SIDE_WHEEL_CLIMB_MIN_MM_S 20.0f
#define SIDE_WHEEL9_DIR          1.0f
#define SIDE_WHEEL10_DIR         1.0f
#define SIDE_WHEEL11_DIR        -1.0f
#define SIDE_WHEEL15_DIR        -1.0f
```

`motor_can_init()` 中 `motor9/motor10/motor11/motor15` 的控制方式为速度闭环：

```cpp
motor9.Init(&hcan1, CAN_Motor_ID_0x205, Control_Method_OMEGA, 36.0f, 10000.0f);
motor10.Init(&hcan1, CAN_Motor_ID_0x206, Control_Method_OMEGA, 36.0f, 10000.0f);
motor11.Init(&hcan1, CAN_Motor_ID_0x207, Control_Method_OMEGA, 36.0f, 10000.0f);
motor15.Init(&hcan2, CAN_Motor_ID_0x205, Control_Method_OMEGA, 36.0f, 10000.0f);
```

`main()` 中侧轮任务按距离闭环状态切换：

```cpp
if (side_distance_enable)
{
    side_distance_control_task();
}
else
{
    side_wheel_control_task();
}
```

1号 MTF-01P 使用 USART6：

```text
USART6_TX = PG14
USART6_RX = PG9
波特率 = 115200
格式 = 8N1
```

MTF-01P 接线：

```text
MTF-01P Tx -> PG9 / USART6_RX
MTF-01P Rx -> PG14 / USART6_TX
MTF-01P GND -> A板 GND
MTF-01P 5V -> A板 5V
```

`HAL_UART_RxCpltCallback()` 中 1号 MTF 接收分支使用 USART6：

```cpp
if (huart->Instance == USART6)
{
    mtf01_decode(mtf01_rx_data);
    HAL_UART_Receive_IT(&huart6, &mtf01_rx_data, 1);
}
```

`uart_init_task()` 中启动 USART6 单字节接收：

```cpp
HAL_UART_Receive_IT(&huart6, &mtf01_rx_data, 1);
```

`mtf01.h` 需要包含 C/C++ 兼容声明，保证 `main.cpp` 能链接 `mtf01_decode()`：

```c
#ifdef __cplusplus
extern "C" {
#endif

extern volatile MTF01_Data_t mtf01_data;
void mtf01_decode(uint8_t data);

#ifdef __cplusplus
}
#endif
```

`mtf01.c` 文件开头顺序为：

```c
#include "mtf01.h"

volatile MTF01_Data_t mtf01_data;
```
## 9. 主要函数职责

| 函数 | 职责 |
| --- | --- |
| `HAL_UART_RxCpltCallback()` | USART3 单字节接收中断，拼接一行指令 |
| `CAN1_Call_Back()` | 接收 CAN1 电机反馈并分发给 `motor1~motor4/motor9/motor10/motor11/motor12` |
| `CAN2_Call_Back()` | 接收 CAN2 电机反馈并分发给 `motor5~motor8/motor15/motor13~motor14` |
| `uart6_cmd_parse()` | 解析 `$CHS/$L1/$L2/$L3/$SW/$A12/$STOP` |
| `chassis_control_task()` | 根据 `$CHS` 更新底盘目标 |
| `lift1_control_task()` | 控制升降台1 |
| `lift2_control_task()` | 控制升降台2 |
| `lift3_control_task()` | 控制升降台3 |
| `side_wheel_control_task()` | 调试模式下控制 `motor9/motor10/motor11/motor15` 侧轮目标转速 |
| `side_distance_control_task()` | MTF-01P 距离闭环，输出 `motor9/motor10/motor11/motor15` 目标转速 |
| `can2_2006_angle_task()` | 控制 `motor12` 角度 |
| `motor_pid_init()` | 初始化所有电机 PID |
| `motor_can_init()` | 绑定所有电机 CAN 总线和 CAN ID |
| `motor_pid_task()` | 周期执行所有电机 PID |
| `TIM_CAN_PeriodElapsedCallback()` | 周期发送 CAN 控制帧 |
| `uart6_feedback_task()` | USART3 输出状态反馈 |
| `uart2_mtf_feedback_task()` | UART3 输出光流调试反馈 |

## 10. main() 调度内容

`main()` 的 1ms 周期中包含：

```cpp
chassis_control_task();

lift1_control_task();
lift2_control_task();
lift3_control_task();

side_wheel_control_task();
can2_2006_angle_task();

motor_pid_task();
TIM_CAN_PeriodElapsedCallback();

uart2_mtf_feedback_task();
#if ACTION_FB_ENABLE
action_feedback_task();
#endif
```

顺序含义：

```text
1. 更新底盘、升降台、侧轮速度或 MTF 距离闭环、角度机构目标
2. 计算 PID
3. 发送 CAN 控制帧
4. 更新反馈
5. 串口输出反馈
```

## 11. 调试流程

### CAN1 motor12

```text
1. 只接 CAN1 的 motor12，电机类型为 3508/C620。
2. 确认电调 ID 为 0x208。
3. 发送 $A12,30000。
4. motor12 应转到 +30 度。
5. 发送 $A12,0。
6. motor12 应回到上电零点。
7. 方向反了就改 CAN2_2006_ANGLE_DIR。
```

### 升降台

```text
1. 第一次测试把对应 LIFTx_UP_ROUNDS 改成 0.2f。
2. 只接当前升降台两个电机。
3. 发送 $Lx,1。
4. 两个电机应配合上升。
5. 发送 $Lx,0。
6. 升降台应回零。
7. 两个电机互相打架时，检查对应 LIFTx_MOTORx_DIR。
```

## 12. 风险点

### CAN ID 冲突

```text
同一条 CAN 总线上不能有两个电调使用同一个 ID。
电调实际 ID 必须和 motor_can_init()、CANx_Call_Back() 中的 ID 一致。
```

### 单位混乱

```text
USART3 的 $A12 使用 mdeg，控制 motor12 角度。
motor12 角度控制内部使用 rad，必须通过 MDEG_TO_RAD 转换。
```

### 上电零点

```text
升降台和 2006 角度机构以上电位置作为零点。
上电时机构必须放在安全初始位置。
```

### 机械限位

```text
第一次测试不要直接使用 5 圈。
先用 0.2 圈或 0.5 圈确认方向和限位。
```

### 遥控器调零与串口分配

```text
USART3：电脑上位机主控入口。
USART6：1号 MTF-01P 光流模块。
UART4：舵机总线调试口。
UART7：HWT101CT-TTL 陀螺仪。
USART1：SBUS 遥控器输入。大疆 A板 USART1 自带反相电路，可直接接 SBUS。
UART8：2号 MTF-01P 光流模块。
```

USART1 遥控器只建议用于升降台1/2的手动点动、调零救援和维护模式，不建议作为正式零点来源。当前版本升降台1/2不使用霍尔零点开关，开机后通过向上抬升约2cm来建立临时零点。

建议控制逻辑：

```text
1. 上电后自动请求升降台1/2零点校准。
2. 升降台1和升降台2同时向上抬升 LIFT12_BOOT_HOME_ROUNDS，当前设为 2.0 圈，按 1圈约1cm 估算为 2cm。
3. 到位后记录当前 motor5~motor8 角度为升降台1/2零点。
4. 零点校准完成后，才允许执行 $L1/$L2/$ACT。
```

### 开机触发升降台1/2零点校准

开机后会自动执行一次。当前版本不再使用 HOME/$HOME 手动触发零点校准。

当前输入脚约定：

```text
PD12：当前零点流程不使用，备用输入
PD13：当前零点流程不使用，备用输入
PD14：当前零点流程不使用，备用输入
PD15：当前零点流程不使用，备用输入
PH10/PH11/PH12：备用输入
```

回零流程：

```text
1. 开机后 lift12_home_active = 1。
2. 1ms 控制周期中由 lift1_control_task() 处理开机零点流程。
3. 目标角度 = 当前角度 + 对应电机方向 * LIFT12_BOOT_HOME_ANGLE。
4. 升降台1和升降台2同时抬升到目标角度。
5. 等待 LIFT12_BOOT_HOME_WAIT_MS 后，记录当前角度为零点。
6. lift1_zero_inited/lift2_zero_inited = 1。
```

调试时先确认 `LIFT12_BOOT_HOME_ROUNDS` 是否对应真实 2cm。如果开机后升降台不是向上抬升，应先停止并调整 `LIFT1_MOTOR5_HOME_DIR/LIFT1_MOTOR6_HOME_DIR/LIFT2_MOTOR7_HOME_DIR/LIFT2_MOTOR8_HOME_DIR`。

### UART4 舵机总线调试

UART4 接 BusLinker TTL 串口口：

```text
PA0 / UART4_TX -> BusLinker RX
PA1 / UART4_RX -> BusLinker TX
GND -> GND
```

UART4 总线上有两个总线舵机：

```text
ID=1  夹爪舵机
ID=2  三位置翻转舵机
```

上位机通过 USART3 发下面固定指令即可调试：

```text
$GRIP,1             夹爪夹取，实际参数在 app_config.h 中调
$GRIP,0             夹爪松开，实际参数在 app_config.h 中调
$FLIP,0             翻转舵机到下垂位置 842
$FLIP,1             翻转舵机到摆平位置 489
$FLIP,2             翻转舵机到上摆位置 119
```

使用 `$GRIP,1` 和 `$GRIP,0` 调试夹爪；旧 `$SERVO` 夹爪指令已移除。

执行 `$GRIP` 或 `$FLIP` 时底盘会强制停车并锁定；锁定时会记录当前轮角作为保持目标。锁定期间即使内部距离闭环写入底盘速度，最终输出也会被清零。锁定后前 `CHASSIS_FORCE_LOCK_IGNORE_MS`，当前 80ms，会丢弃所有速度指令，避免旧速度残留导致车往前蹭；之后上位机继续发送 0 速度时保持不动并保持当前轮角，下一次收到非 0 速度后恢复底盘运动。

上位机二进制协议使用 `ActionGroupCmd(id=0x09, action_id=1)` 触发同样的原地舵机夹取矛头动作；下位机会把上位机 `action_id=1` 映射到本地 `ACTION_ID_GRIPPER_PICK=15`。手动串口调试时仍可直接发送 `$ACT,15`。舵机 ID=1 到 `GRIPPER_SERVO_GRIP_POS`，用时 `GRIPPER_SERVO_GRIP_TIME_MS`，等待 `GRIPPER_PICK_FLIP_DELAY_MS` 后 ID=2 翻转舵机到 `FLIP_SERVO_UP_POS`；当前 `GRIPPER_PICK_FLIP_DELAY_MS = GRIPPER_SERVO_GRIP_TIME_MS + 1500ms`，也就是夹紧后再等 1500ms 翻转；底盘锁定，不启动底盘距离闭环。

## 13. MTF-01P 光流测距模块可行性评估

MTF-01P 可以用于 R2_Computer 的侧轮前进距离闭环。它比“侧轮转固定圈数”更接近真实位移控制，因为它测量的是小车相对地面的实际运动，而不是轮子自身转了多少圈。

### 硬件和通信条件

```text
供电：5V
平均电流：约 100mA
通信电平：LVTTL 3.3V 串口
默认波特率：115200
数据频率：100Hz
接口顺序：GND / 5V / Rx / Tx
```

接线规则：

```text
MTF-01P GND -> A板 GND
MTF-01P 5V  -> A板 5V
MTF-01P Rx  -> A板对应 UART_TX
MTF-01P Tx  -> A板对应 UART_RX
```

R2_Computer 当前 USART3 已经用于电脑上位机控制，1号 MTF-01P 固定使用 USART6。

当前工程中 USART6 是 115200、8N1，适合接 1号 MTF-01P。USART1 是 100000、8E2，属于遥控器/SBUS 风格配置，不适合直接接 MTF-01P。

### 协议和数据内容

MTF-01P 的 Micolink 帧格式：

```text
帧头：0xEF
设备 ID：0x0F
系统 ID：0x00
消息 ID：0x51
负载长度：0x14，也就是 20 字节
校验：前面所有数据累加和
```

`mtf01.h` 中的有效负载结构：

```cpp
typedef struct
{
    uint32_t time_ms;
    uint32_t distance;
    uint8_t  strength;
    uint8_t  precision;
    uint8_t  tof_status;
    uint8_t  reserved1;
    int16_t  flow_vel_x;
    int16_t  flow_vel_y;
    uint8_t  flow_quality;
    uint8_t  flow_status;
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
```

关键字段含义：

```text
distance:
  距离地面的高度，单位 mm。
  0 表示距离数据不可用。

flow_vel_x / flow_vel_y:
  光流速度，单位 cm/s @ 1m。
  真实速度 cm/s = 光流速度 * 高度(m)。

flow_quality:
  光流质量，数值越大可信度越高。

flow_status:
  1 表示光流数据可用。

tof_status:
  1 表示测距数据可用。
```

### 用于侧轮距离闭环的方式

侧轮固定圈数属于开环控制：

```text
目标距离 -> 换算成轮子圈数 -> 侧轮转指定圈数
```

MTF-01P 闭环控制方式：

```text
目标距离 -> 光流累计实际位移 -> 计算距离误差 -> 输出侧轮目标速度 -> 侧轮速度闭环
```

用于 MTF-01P 时，motor9/motor10/motor11/motor15 更适合保持速度控制，而不是角度固定圈数控制。

控制逻辑：

```text
1. 收到侧轮距离指令，例如 $SWD,1000。
2. 清零光流累计位移。
3. 根据目标距离和当前累计位移计算误差。
4. 误差较大时给侧轮较高目标 rpm。
5. 接近目标距离时降低 rpm。
6. 到达允许误差范围后停止侧轮。
```

距离累计示例：

```cpp
height_m = distance_mm / 1000.0f;
vel_y_cm_s = flow_vel_y * height_m;
vel_y_mm_s = vel_y_cm_s * 10.0f;
flow_y_mm += vel_y_mm_s * dt_s;
```

### 可行性结论

```text
可行。
```

适合的原因：

```text
1. 数据频率 100Hz，足够做低速侧轮距离闭环。
2. 通信是 115200 串口，STM32 HAL 可以直接接收解析。
3. 模块同时提供高度和光流速度，可以把光流速度换算成真实地面速度。
4. 能补偿侧轮打滑、轮胎压缩、地面摩擦变化造成的固定圈数误差。
```

限制条件：

```text
1. 需要地面有纹理，纯色、强反光、透明或过暗地面会降低光流质量。
2. 模块安装高度不能太低，光流最小工作距离约 8cm。
3. 车体震动会影响光流质量，需要减震固定。
4. 高度数据为 0 或状态无效时，不能用于闭环。
5. 光流速度单位依赖高度，距离传感器异常会直接影响速度换算。
```

### 接入优先级

当前侧轮还需要先验证 CAN 和电机基础控制。接入顺序固定为：

```text
1. 先让 motor9/motor10/motor11/motor15 能通过 CAN 正常速度转动。
2. 再接入 MTF-01P 串口，确认能解析 distance、flow_vel_x、flow_vel_y。
3. 静止时确认光流速度接近 0。
4. 手推小车，确认累计位移方向和数值合理。
5. 最后做 $SWD,distance_mm 的闭环前进距离控制。
```

### 最终控制建议

侧轮相关指令分成两类：

```text
$SW,enable,rpm:
  调试用，直接控制 motor9/motor10/motor11/motor15 目标转速。

$SWD,distance_mm:
  比赛用，通过 MTF-01P 光流闭环控制小车前进固定距离。
```

最终比赛逻辑使用 `$SWD,distance_mm`，`$SW,enable,rpm` 只保留给调试。

## 14. 2026-06-12 更新：双光流模块与 HWT101 陀螺仪接入

本节记录当前 `R2_Computer` 下位机新增的传感器接入方案。目标是先完成代码骨架，后续再上车调参数。

### 14.1 双 MTF-01P 光流模块

机器人安装两个 MTF-01P 光流模块：

| 编号 | 安装位置 | 串口 | 数据索引 | 用途 |
| --- | --- | --- | --- | --- |
| 1号光流 | 车中间 | USART6 | `mtf01_data[0]` | 中间位置动作阶段的距离闭环 |
| 2号光流 | 车头 | UART8 | `mtf01_data[1]` | 车头位置动作阶段的距离闭环 |

接线：

```text
1号光流：
MTF1 Tx -> PG9 / USART6_RX
MTF1 Rx -> PG14 / USART6_TX
MTF1 GND -> GND
MTF1 5V  -> 5V

2号光流：
MTF2 Tx -> PE0 / UART8_RX
MTF2 Rx -> PE1 / UART8_TX
MTF2 GND -> GND
MTF2 5V  -> 5V
```

`mtf01.h` 中使用数组保存两路数据：

```c
#define MTF01_COUNT 2

extern volatile MTF01_Data_t mtf01_data[MTF01_COUNT];
void mtf01_decode(uint8_t index, uint8_t data);
```

`mtf01.c` 中每一路必须有独立的解析状态：

```c
void mtf01_decode(uint8_t index, uint8_t data)
{
    static MICOLINK_MSG_t msg[MTF01_COUNT];
    ...
}
```

不能让两个串口共用一个 `static MICOLINK_MSG_t msg`，否则 USART6 和 UART8 的字节会互相打乱协议帧。

`main.cpp` 中的串口接收分配：

```cpp
if (huart->Instance == USART6)
{
    mtf01_decode(0, mtf01_rx_data_1);
    HAL_UART_Receive_IT(&huart6, &mtf01_rx_data_1, 1);
}

if (huart->Instance == UART8)
{
    mtf01_decode(1, mtf01_rx_data_2);
    HAL_UART_Receive_IT(&huart8, &mtf01_rx_data_2, 1);
}
```

`main.cpp` 中的索引宏：

```cpp
#define MTF01_MIDDLE_INDEX       0
#define MTF01_FRONT_INDEX        1
```

两个光流模块一直同时接收并更新数据。实际使用哪一个，不由上位机指令切换，而是由下位机动作组在进入某个动作阶段时自动选择。

动作组调用方式：

```cpp
side_distance_start(MTF01_MIDDLE_INDEX, 800.0f); // 使用车中间 1号光流
side_distance_start(MTF01_FRONT_INDEX, 500.0f);  // 使用车头 2号光流
```

`side_distance_start()` 只应在进入动作阶段时调用一次，不要在 1ms 周期中反复调用，否则 `side_distance_now_mm` 会不断清零，距离无法累计。

### 14.2 HWT101CT-TTL 陀螺仪模块

UART7 接入维特 HWT101CT-TTL，用于辅助底盘走直线和旋转闭环。

接线：

```text
HWT101 Tx -> PF6 / UART7_RX
HWT101 Rx -> PF7 / UART7_TX
HWT101 GND -> GND
HWT101 VCC -> 模块要求电压，HWT101CT 通常按资料使用 5~36V/9~36V，建议先用 12V
```

串口参数：

```text
UART7 = 9600 bps, 8N1
```

工程中 `MX_UART7_Init()` 已配置 UART7，主循环初始化阶段需要调用：

```cpp
MX_UART7_Init();
```

`uart_init_task()` 中启动单字节中断接收：

```cpp
HAL_UART_Receive_IT(&huart7, &hwt101_rx_data, 1);
```

`HAL_UART_RxCpltCallback()` 中接入解码：

```cpp
if (huart->Instance == UART7)
{
    hwt101_decode(hwt101_rx_data);
    HAL_UART_Receive_IT(&huart7, &hwt101_rx_data, 1);
}
```

HWT101 数据帧按维特常见格式解析：

```text
帧头：0x55
角速度帧：0x52
角度帧：0x53
帧长：11 字节
校验：前 10 字节累加和
```

当前主要使用：

```text
gyro_z_dps：Z 轴角速度，单位 deg/s
yaw_deg：Z 轴航向角，范围约 -180 到 +180 deg
```

当前工程 USART2 引脚是 `PD5=TX`、`PD6=RX`，不是 PA2/PA3；参数为 `115200 8N1`。

当前没有使用 USART2 查看上位机数据：`UART2_FORWARD_UART3_RX = 0`，`UART2_HOST_ASCII_DEBUG = 0`，并且 main.cpp 中没有初始化 `MX_USART2_UART_Init()`。如果后续需要临时旁路查看 UART3 数据，再打开对应宏并初始化 USART2。

### 14.3 底盘直线 yaw hold

当上位机或动作组给底盘平移速度，且旋转速度为 0 时，下位机使用 HWT101 的 yaw 自动修正底盘偏航。

触发条件：

```text
1. HWT101 数据有效
2. 当前 wz 接近 0
3. vx 或 vy 不为 0
```

核心参数：

```cpp
#define HWT101_YAW_HOLD_KP           0.04f
#define HWT101_YAW_HOLD_KD           0.010f
#define HWT101_YAW_HOLD_MAX_WZ       1.2f
#define HWT101_YAW_HOLD_CLIMB_KP     0.04f
#define HWT101_YAW_HOLD_CLIMB_KD     0.010f
#define HWT101_YAW_HOLD_CLIMB_MAX_WZ 1.2f
#define HWT101_YAW_VALID_TIMEOUT_MS  100
#define HWT101_YAW_CORRECT_DIR       1.0f
```

如果平地车头保持力度不够，提高 `HWT101_YAW_HOLD_KP`。如果上坡车头偏转较大，优先提高 `HWT101_YAW_HOLD_CLIMB_KP` 或 `HWT101_YAW_HOLD_CLIMB_MAX_WZ`，这样不会影响平地手感。如果开始左右摆头，优先提高对应模式的 `KD` 做角速度阻尼；如果仍然摆，再降低对应模式的 KP 或 MAX_WZ。

如果直线修正方向反了，只改：

```cpp
#define HWT101_YAW_CORRECT_DIR      -1.0f
```

### 14.4 光流横漂补偿

当前底盘前进时，如果 HWT101 已经能把车头角度保持很直，但车身仍然因为轮组装配误差产生水平位移，可以用 1号、2号 MTF-01P 光流做横向位移融合补偿。1号车中间光流为主，2号车头光流为辅助。

光流横漂积分使用 MTF-01P 数据帧里的 `time_ms` 计算实际 dt，不再固定假设 100Hz。这样光流帧率有波动时，补偿量会更贴近真实位移，实时性和稳定性更好。

实时性优化：

```text
1. UART3 回传使用非阻塞发送队列，不再用 HAL_UART_Transmit 阻塞 1ms 控制循环。
2. USART3 ASCII 调试反馈周期默认从 20ms 调到 50ms，减少调试输出占用。
3. HWT101 yaw hold 修正量加入斜坡 HWT101_YAW_HOLD_SLEW_RAD_S2，避免修正角速度瞬间跳变。
4. 光流横漂速度加入低延迟 IIR 滤波 CHASSIS_FLOW_FILTER_ALPHA，降低噪声抖动。
```

触发条件：

```text
1. 底盘当前有前进/后退速度，速度绝对值超过 CHASSIS_FLOW_HOLD_MIN_FORWARD_MM_S。
2. 1号车中间光流 distance_valid 和 flow_valid 都有效。
3. 1号光流有新数据帧时才更新横漂积分，没有新帧时保持上一次补偿值。
4. 2号车头光流有效且数据可信时参与融合，否则只相信 1号车中间光流。
```

补偿逻辑：

```text
1号车中间光流为主，2号车头光流为辅：

两个都有效，且数据方向一致、差异不大：
    横漂速度 = 0.7 * 中间光流 + 0.3 * 车头光流

两个差异很大：
    横漂补偿只相信中间光流；如果差值超过 CHASSIS_FLOW_YAW_DIFF_MIN_MM_S，则额外执行车头纠正

车头光流变化明显但中间光流不明显：
    判断为车体旋转带来的假横漂，横漂补偿只相信中间光流，同时可输出少量 wz 抑制车头偏转

融合后的横向速度 -> 累计横向偏移量 -> 输出少量 left 速度补偿
前后光流横向速度差 -> 输出少量 wz 车头纠正
```

核心参数在 `app/app_config.h`：

```cpp
#define CHASSIS_FLOW_HOLD_ENABLE           1
#define CHASSIS_FLOW_HOLD_MTF_INDEX        MTF01_MIDDLE_INDEX
#define CHASSIS_FLOW_HOLD_USE_X_AXIS       1
#define CHASSIS_FLOW_HOLD_DIR              1.0f
#define CHASSIS_FLOW_HOLD_KP               1.2f
#define CHASSIS_FLOW_HOLD_MAX_MM_S         120.0f
#define CHASSIS_FLOW_FILTER_ALPHA          0.8f
#define CHASSIS_FLOW_HOLD_MIN_FORWARD_MM_S 50.0f
#define CHASSIS_FLOW_HOLD_DUAL_ENABLE      1
#define CHASSIS_FLOW_HOLD_MIDDLE_WEIGHT    0.7f
#define CHASSIS_FLOW_HOLD_FRONT_WEIGHT     0.3f
#define CHASSIS_FLOW_HOLD_DIFF_MAX_MM_S    100.0f
#define CHASSIS_FLOW_HOLD_FRONT_ONLY_MM_S  50.0f
#define CHASSIS_FLOW_HOLD_MIDDLE_STILL_MM_S 20.0f
#define CHASSIS_FLOW_YAW_CORRECT_ENABLE    0
#define CHASSIS_FLOW_YAW_DISABLE_IN_CLIMB  1
#define CHASSIS_FLOW_YAW_DIFF_MIN_MM_S     120.0f
#define CHASSIS_FLOW_YAW_KP                0.002f
#define CHASSIS_FLOW_YAW_MAX_WZ            0.5f
#define CHASSIS_FLOW_YAW_DIR               1.0f
```

调试时先用 UART3 看 1号光流反馈，手动把车向左/向右平移。当前输出格式为 `$MTF1,flow_x方向,flow_y方向,测距有效,光流有效`：

```text
如果横向移动主要变化的是 flow_x，保持 CHASSIS_FLOW_HOLD_USE_X_AXIS = 1。
如果横向移动主要变化的是 flow_y，改成 CHASSIS_FLOW_HOLD_USE_X_AXIS = 0。
如果补偿后越修越偏，把 CHASSIS_FLOW_HOLD_DIR 从 1.0f 改成 -1.0f。
上坡时光流模块和车体/地面夹角会变化，前后光流差值容易和陀螺仪 yaw 判断冲突。当前 `CHASSIS_FLOW_YAW_DISABLE_IN_CLIMB = 1`，所以爬坡模式下车头纠正优先相信 HWT101 陀螺仪，光流不再参与车头 yaw 修正；光流仍可用于横漂/距离补偿。

如果非爬坡模式下前后差值触发车头纠正后越修越偏，把 CHASSIS_FLOW_YAW_DIR 从 1.0f 改成 -1.0f。
如果车头纠正太猛，降低 CHASSIS_FLOW_YAW_KP 或 CHASSIS_FLOW_YAW_MAX_WZ。
如果车头偏了但没有明显纠正，降低 CHASSIS_FLOW_YAW_DIFF_MIN_MM_S 或提高 CHASSIS_FLOW_YAW_KP。
```

### 14.5 动作组旋转闭环

后续动作组需要旋转指定角度时，不建议用固定时间开环旋转，而是调用陀螺仪旋转闭环。

建议接口：

```cpp
chassis_gyro_turn_start(90.0f);   // 左转 90 度
chassis_gyro_turn_start(-90.0f);  // 右转 90 度
```

主循环 1ms 周期中，在 `chassis_control_task()` 前调用：

```cpp
chassis_gyro_turn_task();
chassis_control_task();
```

旋转闭环参数：

```cpp
#define HWT101_TURN_KP_MDEG          3000.0f
#define HWT101_TURN_MAX_MDEG_S       90000.0f
#define HWT101_TURN_DONE_DEG         1.0f
```

达到目标角度后，`chassis_gyro_turn_task()` 会清零底盘旋转速度，并结束本次旋转阶段。

### 14.6 建议调试顺序

```text
1. 先确认 UART7 能收到 HWT101 数据，yaw_deg 会随车体旋转变化。
2. 确认静止时 yaw 基本稳定，不大幅跳变。
3. 测试直线 yaw hold：发 $CHS,500,0,0，看车体偏航是否被拉回。
4. 如果越修越偏，修改 HWT101_YAW_CORRECT_DIR 的正负。
5. 测试光流横漂补偿：发 $CHS,500,0,0，看水平位移是否逐渐被拉回。
6. 如果水平位移越修越大，修改 CHASSIS_FLOW_HOLD_DIR 的正负。
7. 再测试动作组旋转闭环：chassis_gyro_turn_start(90.0f)。
8. 最后再调 HWT101_YAW_HOLD_KP、HWT101_TURN_KP_MDEG、CHASSIS_FLOW_HOLD_KP 等参数。
```

## 15. 底盘启动过快、到点刹不住和震荡调参

当前先使用保守版参数，优先保证能停住、少震荡，再逐步提速。

```cpp
#define CHASSIS_FRONT_BACK_ACCEL_MM_S2 1200.0f
#define CHASSIS_FRONT_BACK_DECEL_MM_S2 4000.0f
#define CHASSIS_LEFT_RIGHT_ACCEL_MM_S2 1000.0f
#define CHASSIS_LEFT_RIGHT_DECEL_MM_S2 4000.0f
#define CHASSIS_ANGULAR_ACCEL_MDEG_S2 400000.0f
#define CHASSIS_ANGULAR_DECEL_MDEG_S2 600000.0f

#define CHASSIS_DISTANCE_KP       0.35f
#define CHASSIS_DISTANCE_MAX_MM_S 300.0f
#define CHASSIS_DISTANCE_MIN_MM_S 30.0f
#define CHASSIS_DISTANCE_STOP_MM  25.0f
```

底盘速度输入已改为加速度限幅。上位机即使从 0 突然发到 500 mm/s，下位机也会按加速度参数缓慢爬升，避免急加速瞬间因为前后轮响应不一致导致车头偏转。

`CHASSIS_FRONT_BACK_ACCEL_MM_S2` 控制前后移动启动有多猛。`CHASSIS_LEFT_RIGHT_ACCEL_MM_S2` 控制左右移动启动有多猛，左右移动容易带出车头偏转时可以单独调小。对应的 `DECEL` 控制松开或到点时刹车有多快，值可以比加速度大一些。旋转方向同理使用 `CHASSIS_ANGULAR_ACCEL_MDEG_S2` 和 `CHASSIS_ANGULAR_DECEL_MDEG_S2`。

侧轮光流距离闭环也同步保守化：

```cpp
kp = 0.35f
stop_error_mm = 25.0f
max_rpm = 300.0f
min_rpm = 30.0f
```

调试顺序：

```text
1. 如果前后突然加速时车头仍然明显偏转，降低 CHASSIS_FRONT_BACK_ACCEL_MM_S2，例如 1200 -> 900。
2. 如果左右突然加速时车头仍然明显偏转，降低 CHASSIS_LEFT_RIGHT_ACCEL_MM_S2，例如 1000 -> 800。
3. 如果到点仍然冲过头，降低 CHASSIS_DISTANCE_MAX_MM_S 或增大 CHASSIS_DISTANCE_STOP_MM。
4. 如果接近目标时来回抖，降低 CHASSIS_DISTANCE_KP，并保持 CHASSIS_DISTANCE_MIN_MM_S 不要太高。
5. 如果松开速度后刹车太慢，提高对应方向的 DECEL，例如 4000 -> 5000。
6. 如果车响应过慢、不跟手，再小幅提高对应方向的 ACCEL 或 CHASSIS_DISTANCE_MAX_MM_S。
```

### 15.1 平面/爬坡双 PID

底盘 motor1~motor4 现在使用两套 PID：

```text
平面模式：默认上电使用，适合平面运动，优先稳定、不震荡。
爬坡模式：上位机发送 `ConfrontClimbCmd mode=1` 或触发爬坡动作组时切换，速度环 P/I、积分限幅和总输出限幅更大，优先保证爬坡力气。
```

切换规则：

```text
1. 上电后使用平面 PID。
2. 上位机发送 `ConfrontClimbCmd mode=1` 后保持爬坡 PID；发送 `mode=0` 后恢复普通底盘 PID。
3. ACTION_ID_CLIMB_UP_LOW / HIGH、CLIMB_DOWN_LOW / HIGH、CLIMB_UP_LOW_FINISH、CLIMB_UP_LOW_GRAB、CLIMB_UP_HIGH_GRAB 开始时临时切换到爬坡 PID。
4. 遥控器调试时，CH6 接管底盘，CH5 高于 `RC_SBUS_CLIMB_PID_THRESHOLD` 时切换到爬坡 PID，CH5 低于阈值时使用上位机当前请求状态。
5. 方块动作、舵机夹取动作不切爬坡 PID。
6. 动作完成、失败或 `$ACT,0` 停止后恢复到上位机 `ConfrontClimbCmd` 请求的状态。
```

主要参数在 `app/app_config.h`：

```cpp
CHASSIS_PID_FLAT_ANGLE_KP
CHASSIS_PID_FLAT_OMEGA_KP
CHASSIS_PID_FLAT_OMEGA_KI
CHASSIS_PID_FLAT_OMEGA_I_MAX
CHASSIS_PID_FLAT_OMEGA_OUT_MAX

CHASSIS_PID_CLIMB_ANGLE_KP
CHASSIS_PID_CLIMB_OMEGA_KP
CHASSIS_PID_CLIMB_OMEGA_KI
CHASSIS_PID_CLIMB_OMEGA_I_MAX
CHASSIS_PID_CLIMB_OMEGA_OUT_MAX
```

如果爬坡还是无力，优先小幅提高 `CHASSIS_PID_CLIMB_OMEGA_OUT_MAX`，再提高 `CHASSIS_PID_CLIMB_OMEGA_I_MAX` 或 `CHASSIS_PID_CLIMB_OMEGA_KI`。如果爬坡时抖动明显，先降低 `CHASSIS_PID_CLIMB_OMEGA_KI`。

## 16. USART1 SBUS 遥控器底盘调试

USART1 已按 SBUS 参数配置：

```text
100000 baud
8E2
PB7 = USART1_RX
PA9 = USART1_TX
```

接线：

```text
SBUS 信号线 -> PB7 / USART1_RX
SBUS GND    -> A板 GND
接收机供电按接收机要求接 5V 或外部 BEC
```

当前通道映射：

```text
CH1 -> 左右平移，软件内已取反
CH2 -> 前进/后退，软件内已取反
CH4 -> 原地旋转，软件内已取反
CH5 -> 爬坡 PID 调试开关，高于 RC_SBUS_CLIMB_PID_THRESHOLD 时使用爬坡 PID
CH6 -> 遥控器接管开关，通道值大于 1200 时接管底盘
```

速度限制：

```cpp
#define RC_SBUS_CH_DEADBAND         40
#define RC_CHASSIS_LINEAR_SCALE_MM_S 0.6097561f
#define RC_CHASSIS_WZ_SCALE_MDEG_S   109.7561f
```

满杆约等于：

```text
平移 500 mm/s
旋转 90 deg/s
```

注意：

```text
1. 当前 USART2 未初始化，不再旁路查看 UART3 上位机数据。
2. 为了放进 Keil 免费版 32KB 限制，遥控器调试时已关闭 UART3 光流调试输出。
3. 光流横漂补偿没有关闭，仍然参与底盘前进时的横漂修正。
4. CH1/CH2/CH4 已按当前实车反馈统一反向，前后、左右、旋转方向应恢复正常。
5. CH6 从接管切到关闭时，下位机会自动清零底盘速度，上位机不需要先补发 $CHS,0,0,0。
6. 动作组运行中遥控器不会解除底盘锁定；动作完成后，CH6 接管才允许恢复遥控器控制。
7. 如果开启遥控器后仍原地旋转，优先把 RC_SBUS_CH_DEADBAND 从 40 调到 60 或 80。
7. 如果 CH6 开关方向反了，修改 RC_SBUS_ENABLE_THRESHOLD 或判断方向。
```

## 17. 现场调试参数清单

调试时建议按下面顺序逐项确认。

### 升降台3

```cpp
LIFT3_LOW_ROUNDS
LIFT3_HIGH_ROUNDS
LIFT3_10CM_ROUNDS
LIFT3_20CM_ROUNDS
LIFT3_40CM_ROUNDS
LIFT3_60CM_ROUNDS
LIFT3_MOTOR13_DIR
LIFT3_MOTOR14_DIR
LIFT_MAX_OMEGA
LIFT_HOME_TIMEOUT_MS
```

### 升降台2

```cpp
LIFT2_LOW_ROUNDS
LIFT2_HIGH_ROUNDS
LIFT2_MOTOR7_STAGE_DIR
LIFT2_MOTOR8_STAGE_DIR
LIFT2_MOTOR7_HOME_DIR
LIFT2_MOTOR8_HOME_DIR
LIFT_MAX_OMEGA
LIFT12_BOOT_HOME_ROUNDS
LIFT12_BOOT_HOME_WAIT_MS
```

### 升降台1

```cpp
LIFT1_LOW_ROUNDS
LIFT1_HIGH_ROUNDS
LIFT1_MOTOR5_STAGE_DIR
LIFT1_MOTOR6_STAGE_DIR
LIFT1_MOTOR5_HOME_DIR
LIFT1_MOTOR6_HOME_DIR
LIFT_MAX_OMEGA
LIFT12_BOOT_HOME_ROUNDS
LIFT12_BOOT_HOME_WAIT_MS
```

### 侧轮

```cpp
SIDE_WHEEL_MAX_RPM
SIDE_WHEEL9_DIR
SIDE_WHEEL10_DIR
SIDE_WHEEL_CMD_TIMEOUT_MS
CLIMB_SIDE_UP_MM
CLIMB_DOWN_SIDE_MM
```

### 电磁阀

```cpp
ACTION_GPIO_WAIT_MS
PI5 输出状态
PI6 输出状态
PI7 输出状态
```

### 翻转

```cpp
BLOCK_MOTOR12_CCW_90_ANGLE
BLOCK_MOTOR12_CCW_180_ANGLE
BLOCK_MOTOR12_CCW_270_ANGLE
CAN2_2006_ANGLE_DIR
ANGLE_2006_MAX_OMEGA
ACTION_MOTOR_WAIT_MS
```

### 三位置翻转舵机

```cpp
$FLIP,0
$FLIP,1
$FLIP,2
FLIP_SERVO_ID
FLIP_SERVO_DOWN_POS
FLIP_SERVO_FLAT_POS
FLIP_SERVO_UP_POS
FLIP_SERVO_TIME_MS
FLIP_SERVO_BOOT_DOWN_ENABLE
FLIP_SERVO_BOOT_DELAY_MS
FLIP_SERVO_BOOT_RETRY_PERIOD_MS
FLIP_SERVO_BOOT_RETRY_COUNT
```

### 舵机夹取矛头

```cpp
$GRIP,1
$GRIP,0
GRIPPER_SERVO_ID
GRIPPER_SERVO_GRIP_POS
GRIPPER_SERVO_GRIP_TIME_MS
GRIPPER_PICK_FLIP_EXTRA_DELAY_MS
GRIPPER_PICK_FLIP_DELAY_MS
GRIPPER_SERVO_RELEASE_POS
GRIPPER_SERVO_RELEASE_TIME_MS
GRIPPER_SERVO_BOOT_RELEASE_ENABLE
GRIPPER_SERVO_BOOT_DELAY_MS
GRIPPER_SERVO_BOOT_RETRY_PERIOD_MS
GRIPPER_SERVO_BOOT_RETRY_COUNT
BUS_SERVO_POS_MAX
BUS_SERVO_TIME_DEFAULT_MS
BUS_SERVO_TIME_MAX_MS
BUS_SERVO_TX_QUEUE_SIZE
```

## 18. 底盘常见问题处理清单

现场调底盘时不要一次改很多参数。先确认 `$DBG` 里的上位机指令是否正确，再按现象逐项处理。

### 启动太猛，车头瞬间偏转

优先处理启动加速度，而不是先改 PID。

```text
1. 前后移动启动猛：降低 CHASSIS_FRONT_BACK_ACCEL_MM_S2，例如 1200 -> 900 -> 700。
2. 左右移动启动猛：降低 CHASSIS_LEFT_RIGHT_ACCEL_MM_S2，例如 1000 -> 800 -> 600。
3. 旋转启动猛：降低 CHASSIS_ANGULAR_ACCEL_MDEG_S2，例如 400000 -> 300000。
4. 如果慢慢启动时不偏、突然加速才偏，基本就是加速度太大。
```

### 停下后还会慢慢转动

先判断是不是上位机或遥控器仍在给旋转指令。

```text
1. 查看 $DBG 前三个值，停止时应为 $DBG,0,0,0,...
2. 如果第三个 wz 不是 0，先处理上位机 angular.z 或遥控器中位死区。
3. 如果 wz 已经是 0 仍慢慢转，降低 motor1~motor4 的 PID_Omega 的 I。
4. PID_Omega 的 I 是 Init(...) 的第二个参数，例如 1000 -> 500 -> 300。
5. 同时可把积分输出限幅从 2000 降到 1500 或 1000。
6. 如果只是松开后一小段时间才停住，提高 CHASSIS_ANGULAR_DECEL_MDEG_S2，例如 600000 -> 900000。
```

### 上位机发移动指令时整车轻微来回摇晃

这种现象通常是车头保持、光流补偿或速度环积分过强。

```text
1. 先看 $DBG 的 wz 是否为 0。移动但不旋转时，第三个值必须为 0。
2. 如果车头像左右摆头，降低 HWT101_YAW_HOLD_KP，例如 0.04 -> 0.025 -> 0.015。
3. 如果回正动作太猛，降低 HWT101_YAW_HOLD_MAX_WZ，例如 1.2 -> 0.8 -> 0.5。
4. 如果像横向蛇形修正，降低 CHASSIS_FLOW_HOLD_KP。
5. 如果四个轮子有一顿一顿的感觉，降低 motor1~motor4 的 PID_Omega 的 I。
```

### 到目标位置刹不住或冲过头

优先调距离闭环，再调速度环。

```text
1. 降低 CHASSIS_DISTANCE_MAX_MM_S，例如 300 -> 250 -> 200。
2. 降低 CHASSIS_DISTANCE_KP，例如 0.35 -> 0.25 -> 0.2。
3. 增大 CHASSIS_DISTANCE_STOP_MM，例如 25 -> 30 -> 40。
4. 如果速度降下来后仍冲过，检查光流距离方向和距离累计是否正确。
```

### 接近目标时来回震荡

通常是距离闭环 P 偏大，或者最小速度太高。

```text
1. 降低 CHASSIS_DISTANCE_KP。
2. 降低 CHASSIS_DISTANCE_MIN_MM_S，例如 30 -> 20。
3. 如果震荡伴随轮子一直有力顶住，降低 PID_Omega 的 I 和积分限幅。
4. 不要为了停得准盲目提高 CHASSIS_DISTANCE_KP。
```

### 前进能保持车头很直，但车身水平漂移

这是光流横漂补偿要处理的问题。

```text
1. 确认双光流都有效，且光流方向一致。
2. 如果补偿方向反了，修改 CHASSIS_FLOW_HOLD_DIR 的正负。
3. 如果横向移动变化的轴不对，修改 CHASSIS_FLOW_HOLD_USE_X_AXIS。
4. 如果修正太弱，提高 CHASSIS_FLOW_HOLD_KP 或 CHASSIS_FLOW_HOLD_MAX_MM_S。
5. 如果修正来回摆，降低 CHASSIS_FLOW_HOLD_KP。
```

### 光流越修越偏

先不要改 PID，先改方向。

```text
1. 把 CHASSIS_FLOW_HOLD_DIR 从 1.0f 改成 -1.0f，或反过来。
2. 如果换方向后仍不对，检查 CHASSIS_FLOW_HOLD_USE_X_AXIS。
3. 通过 UART3 查看光流数据，手动左右推车，确认 flow_x 或 flow_y 哪个对应横向位移。
```

### 遥控器正常，上位机控制时不正常

先检查上位机发来的数据格式和频率。

```text
1. 上位机需要 50Hz 连续发送速度指令，也就是约每 20ms 一帧。
2. 查看 $DBG，确认下位机收到的 forward、left、wz 与预期一致。
3. 如果 $DBG 正确但车摇晃，按 HWT101_YAW_HOLD_KP、CHASSIS_FLOW_HOLD_KP、PID_Omega 的 I 顺序调。
4. 如果 $DBG 不正确，先修上位机坐标轴、单位和 angular.z。
```

### 遥控器一打开车就原地旋转

优先处理遥控器通道中位和死区。

```text
1. 增大 RC_SBUS_CH_DEADBAND，例如 40 -> 60 -> 80。
2. 检查 CH4 旋转通道中位是否在 1024 附近。
3. 如果方向反了，检查遥控器通道映射或软件取反。
4. CH6 关闭后应切回上位机控制，并自动清零底盘速度。
```

### 四个轮子响应不一致

先统一底盘 1~4 号电机速度环参数，再查机械。

```text
1. motor1~motor4 的 PID_Omega 先统一成相同 P/I/D。
2. 建议初始值：P=900，I=500，D=0，积分输出限幅=1500，总输出限幅=8000。
3. 如果某一个轮明显慢，检查电机 ID、CAN 反馈、减速箱、轮子安装阻力。
4. 如果手转很顺但启动慢，检查该轮 PID 或电机反馈方向。
```
