#pragma once

// Robot identity: R2 competition robot code. Do not use this project as R1 firmware.
// ==================== 工程选择 / Project Selection ====================
// 只能有一个 R2_PROJECT_xxx 为 1。EN: Only one R2_PROJECT_xxx can be 1.
#define R2_PROJECT_SBUS        0    // 1 表示编译 SBUS 遥控器工程。EN: 1 builds the SBUS remote-control project.
#define R2_PROJECT_COMPUTER    1    // 1 表示编译上位机控制工程。EN: 1 builds the upper-computer controlled project.
#define R2_PROJECT_UP1         0    // 1 表示编译 UP1 版本工程。EN: 1 builds the UP1 variant project.

#define R2_PROJECT_NAME        "R2_Computer" // 工程名称，用于识别和调试。EN: Project name string used for identification/debug.
#if (R2_PROJECT_SBUS + R2_PROJECT_COMPUTER + R2_PROJECT_UP1) != 1
#error "Exactly one R2_PROJECT_xxx macro must be set to 1."
#endif

// ==================== 底盘双 PID / Chassis Dual PID ====================
#define CHASSIS_PID_FLAT_ANGLE_KP      4.5f      // 平面模式角度外环 P；越大跟随越硬。EN: Flat-mode angle-loop P; larger = stiffer tracking.
#define CHASSIS_PID_FLAT_OMEGA_KP      1000.0f   // 平面模式速度内环 P。EN: Flat-mode speed-loop P.
#define CHASSIS_PID_FLAT_OMEGA_KI      500.0f    // 平面模式速度内环 I。EN: Flat-mode speed-loop I.
#define CHASSIS_PID_FLAT_OMEGA_I_MAX   1000.0f   // 平面模式速度环积分输出限幅。EN: Flat-mode speed-loop integral output limit.
#define CHASSIS_PID_FLAT_OMEGA_OUT_MAX 8000.0f   // 平面模式速度环总输出限幅。EN: Flat-mode speed-loop total output limit.
#define CHASSIS_PID_CLIMB_ANGLE_KP     5.0f      // 爬坡模式角度外环 P；过大会抖动/限幅。EN: Climb-mode angle-loop P; too high can oscillate/saturate.
#define CHASSIS_PID_CLIMB_OMEGA_KP     1500.0f   // 爬坡模式速度内环 P；提高抗负载能力，摆头时调小。EN: Climb-mode speed-loop P; lower if heading oscillates.
#define CHASSIS_PID_CLIMB_OMEGA_KI     800.0f    // 爬坡模式速度内环 I；提高持续爬坡力，摆头时调小。EN: Climb-mode speed-loop I; lower if heading oscillates.
#define CHASSIS_PID_CLIMB_OMEGA_I_MAX  2500.0f   // 爬坡模式速度环积分输出限幅。EN: Climb-mode speed-loop integral output limit.
#define CHASSIS_PID_CLIMB_OMEGA_OUT_MAX 14000.0f // 爬坡模式速度环总输出限幅；越大越有力但电机更热。EN: Climb-mode speed-loop total output limit; higher means stronger but hotter.
#define RC_SBUS_CLIMB_PID_CH           4         // 遥控器爬坡 PID 调试开关使用的 SBUS 通道编号，4 表示 CH5。EN: SBUS channel index for climb-PID debug switch; 4 means CH5.
#define RC_SBUS_CLIMB_PID_THRESHOLD    1200      // CH5 高于该值时遥控器底盘使用爬坡 PID，否则使用平面 PID。EN: Above this value remote chassis uses climb PID; otherwise flat PID.

// ==================== 串口和单位换算 / UART and Unit Conversion ====================
#define ENABLE_UART6_CMD_RX       1              // 1 表示开启 USART3 ASCII 指令接收；宏名保留 UART6 是历史命名。EN: 1 enables USART3 ASCII command receive; UART6 name is legacy.
#define UART6_FB_PERIOD_MS        50             // USART3 ASCII 调试反馈周期，单位 ms；50ms=20Hz；宏名保留 UART6 是历史命名。EN: USART3 ASCII debug feedback period; UART6 name is legacy.
#define UART3_TX_QUEUE_SIZE       8              // UART3 非阻塞发送队列深度。EN: UART3 non-blocking TX queue depth.
#define UART3_TX_FRAME_MAX_LEN    160            // UART3 单帧发送缓存最大长度。EN: UART3 max queued TX frame length.
#define CHASSIS_ODOM_FB_ENABLE    0              // 历史兼容开关；当前 WheelOdom 按新协议填 pos_x/pos_y/yaw/vx/vy/omega。EN: Legacy gate; WheelOdom now carries full odom fields.
#define CHASSIS_VEL_FB_ENABLE     1              // 1 表示周期发送 WheelOdom 帧。EN: 1 sends the periodic WheelOdom frame.
#define CHASSIS_VEL_FB_PERIOD_MS  100            // 当前速度回传周期，单位 ms；100ms=10Hz。EN: Current velocity feedback period in ms; 100ms = 10Hz.
#define CHASSIS_VEL_FB_DEADBAND_M_S 0.05f        // 当前速度回传死区，单位 m/s；小于该值上报 0。EN: Velocity feedback deadband in m/s; values below this are reported as 0.
#define CHASSIS_VEL_FB_LOCK_ZERO  1              // 1 表示底盘强制锁定时 WheelOdom vx/vy/omega 上报 0。EN: 1 reports WheelOdom vx/vy/omega as 0 while chassis is force-locked.
#define ACTION_FB_ENABLE          0              // 1 表示通过 UART3 回传动作组状态；0 表示不发状态。EN: 1 enables action status feedback on UART3; 0 disables it.
#define UART2_FB_PERIOD_MS        20             // UART2 调试反馈周期，单位 ms；20ms=50Hz。EN: UART2 debug feedback period in ms; 20ms = 50Hz.
#define RAD_TO_MDEG               57295.7795f    // 弧度转毫度的系数。EN: radian to millidegree conversion.
#define MDEG_TO_RAD               0.00001745329252f // 毫度转弧度的系数。EN: millidegree to radian conversion.

// ==================== 底盘几何和速度限制 / Chassis Geometry and Speed Limit ====================
#define CHASSIS_HALF_LENGTH       0.235f         // 底盘半长，单位 m。EN: Half chassis length, meter.
#define CHASSIS_HALF_WIDTH        0.16f          // 底盘半宽，单位 m。EN: Half chassis width, meter.
#define CHASSIS_K                 (CHASSIS_HALF_LENGTH + CHASSIS_HALF_WIDTH) // 旋转运动学半径，单位 m。EN: Rotation kinematic radius, meter.
#define WHEEL_RADIUS              0.076f         // 全向轮半径，单位 m。EN: Omni wheel radius, meter.
#define GEAR_RATIO                19.0f          // 3508/C620 减速比。EN: 3508/C620 gearbox ratio.
#define MAX_WHEEL_SPEED           90.0f          // 最大轮速指令，单位 rad/s。EN: Max wheel angular speed command, rad/s.
#define MAX_CHASSIS_SPEED         (MAX_WHEEL_SPEED * WHEEL_RADIUS) // 最大底盘平移速度，单位 m/s。EN: Max chassis linear speed, m/s.
#define MAX_CHASSIS_OMEGA         (MAX_CHASSIS_SPEED / CHASSIS_K)  // 最大底盘旋转速度，单位 rad/s。EN: Max chassis angular speed, rad/s.

// ==================== 底盘加速度限制 / Chassis Acceleration Limit ====================
#define CHASSIS_FRONT_BACK_ACCEL_MM_S2 1200.0f   // 前后移动加速度限制，单位 mm/s^2；越小启动越柔。EN: Forward/backward acceleration limit, mm/s^2; smaller = smoother start.
#define CHASSIS_FRONT_BACK_DECEL_MM_S2 8000.0f   // 前后移动减速度限制，单位 mm/s^2；越大刹车越快。EN: Forward/backward deceleration limit, mm/s^2; larger = faster braking.
#define CHASSIS_LEFT_RIGHT_ACCEL_MM_S2 8000.0f   // 左右移动加速度限制，单位 mm/s^2；横移起步偏头时调小。EN: Left/right acceleration limit, mm/s^2; lower this if side move yaws at start.
#define CHASSIS_LEFT_RIGHT_DECEL_MM_S2 4000.0f   // 左右移动减速度限制，单位 mm/s^2；越大刹车越快。EN: Left/right deceleration limit, mm/s^2; larger = faster braking.
#define CHASSIS_ANGULAR_ACCEL_MDEG_S2 400000.0f  // 旋转加速度限制，单位 mdeg/s^2；400000=400deg/s^2。EN: Rotation acceleration limit, mdeg/s^2; 400000 = 400deg/s^2.
#define CHASSIS_ANGULAR_DECEL_MDEG_S2 900000.0f  // 旋转减速度限制，单位 mdeg/s^2；越大停止越快。EN: Rotation deceleration limit, mdeg/s^2; larger = faster stop.

// ==================== 底盘电机方向 / Chassis Motor Direction ====================
#define MOTOR1_DIR                1.0f           // motor1 轮子方向系数；方向反了改成 -1。EN: Motor1 wheel direction sign; change to -1 if motor1 runs backward.
#define MOTOR2_DIR                1.0f           // motor2 轮子方向系数；方向反了改成 -1。EN: Motor2 wheel direction sign; change to -1 if motor2 runs backward.
#define MOTOR3_DIR               -1.0f           // motor3 轮子方向系数；方向反了改成 1。EN: Motor3 wheel direction sign; change to 1 if motor3 runs backward.
#define MOTOR4_DIR               -1.0f           // motor4 轮子方向系数；方向反了改成 1。EN: Motor4 wheel direction sign; change to 1 if motor4 runs backward.

#define CHASSIS_CMD_TIMEOUT_MS    300           // 底盘指令超时时间，单位 ms；超时后底盘速度清零。EN: Chassis command timeout in ms; timeout forces chassis command to zero.
#define CHASSIS_FORCE_LOCK_IGNORE_MS 80U         // 底盘强制锁定后短时间丢弃旧速度帧，单位 ms。EN: Short stale-CmdVel discard window after force lock, ms.
#define SIDE_WHEEL_CMD_TIMEOUT_MS 300            // 侧轮指令超时时间，单位 ms；超时后侧轮停止。EN: Side-wheel command timeout in ms; timeout stops side wheels.

// ==================== SBUS 遥控器 / SBUS Remote Control ====================
#define RC_SBUS_ENABLE              0            // 1 表示启用 SBUS 遥控器接管底盘；0 表示纯 UART3 上位机控制。EN: 1 enables SBUS chassis takeover; 0 means UART3-only control.
#define RC_SBUS_CH_CENTER           992          // SBUS 通道中值。EN: SBUS channel center value.
#define RC_SBUS_CH_RANGE            820          // SBUS 通道半量程，用于归一化。EN: SBUS channel half range used for normalization.
#define RC_SBUS_CH_DEADBAND         40           // SBUS 中位死区；越大越不容易被摇杆漂移影响。EN: SBUS deadband around center; larger = less joystick drift.
#define RC_SBUS_ENABLE_CH           5            // 遥控器接管开关使用的 SBUS 通道编号。EN: SBUS channel index used as remote-control enable switch.
#define RC_SBUS_ENABLE_THRESHOLD    1200         // 接管阈值；通道值高于该值时遥控器接管底盘。EN: Enable switch threshold; above this value remote controls chassis.
#define RC_CHASSIS_MAX_LINEAR_MM_S  500.0f       // 遥控器最大平移速度，单位 mm/s。EN: Remote-control max chassis linear speed, mm/s.
#define RC_CHASSIS_MAX_WZ_MDEG_S    90000.0f     // 遥控器最大旋转速度，单位 mdeg/s；90000=90deg/s。EN: Remote-control max yaw speed, mdeg/s; 90000 = 90deg/s.
#define RC_CHASSIS_LINEAR_SCALE_MM_S 0.6097561f  // SBUS 原始值到平移速度的比例，单位 mm/s/count。EN: SBUS raw value to linear speed scale, mm/s per count.
#define RC_CHASSIS_WZ_SCALE_MDEG_S   109.7561f   // SBUS 原始值到旋转速度的比例，单位 mdeg/s/count。EN: SBUS raw value to yaw speed scale, mdeg/s per count.

// ==================== 升降台 1/2 通用参数 / Lift 1/2 Common Parameters ====================
#define LIFT_ZERO_DELAY_MS        200            // 回零后的等待时间，单位 ms。EN: Delay after zero/home operation, ms.
#define LIFT_MAX_OMEGA            20.0f          // 升降台电机最大角速度指令，单位 rad/s。EN: Lift motor max angular speed command, rad/s.
#define LIFT12_MAX_OMEGA          4.0f           // 升降台 1/2 最大角速度指令，速度慢但更稳。EN: Lift1/2 max angular speed command, slower but stronger.
#define LIFT_HOME_STEP_ANGLE      0.002f         // 回零搜索时每 1ms 增加的目标角度，单位 rad。EN: Home-search target step per 1ms cycle, rad.
#define LIFT_HOME_TIMEOUT_MS      8000           // 回零搜索超时时间，单位 ms。EN: Home-search timeout, ms.
#define LIFT_ROUNDS_TO_ANGLE(rounds) ((rounds) * 6.283185307f) // 输出轴圈数转弧度；2.0 表示输出轴转 2 圈。EN: Output shaft rounds to rad; 2.0 means 2 output shaft turns.
#define LIFT12_BOOT_HOME_ROUNDS   0.1f           // 升降台 1/2 开机回零移动圈数，单位输出轴圈数。EN: Lift1/2 boot home movement, output shaft rounds.
#define LIFT12_BOOT_HOME_ANGLE    LIFT_ROUNDS_TO_ANGLE(LIFT12_BOOT_HOME_ROUNDS) // 升降台 1/2 开机回零角度，单位 rad。EN: Lift1/2 boot home angle, rad.
#define LIFT12_BOOT_HOME_WAIT_MS  3000U          // 开机回零最大等待时间，单位 ms。EN: Max wait time for boot home action, ms.

// ==================== 升降台 1 高度参数 / Lift 1 Height Parameters ====================
#define LIFT1_LOW_ROUNDS          1.0f          // 升降台 1 第一档高度，单位输出轴圈数。EN: Lift1 first-stage height, output shaft rounds.
#define LIFT1_HIGH_ROUNDS         3.0f          // 升降台 1 第二档高度，单位输出轴圈数。EN: Lift1 second-stage height, output shaft rounds.
#define LIFT1_THIRD_ROUNDS        5.0f          // 升降台 1 第三档高度，单位输出轴圈数。EN: Lift1 third-stage height, output shaft rounds.
#define LIFT1_LOW_ANGLE           LIFT_ROUNDS_TO_ANGLE(LIFT1_LOW_ROUNDS)  // 升降台 1 第一档目标角度，单位 rad。EN: Lift1 first-stage target angle, rad.
#define LIFT1_HIGH_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT1_HIGH_ROUNDS) // 升降台 1 第二档目标角度，单位 rad。EN: Lift1 second-stage target angle, rad.
#define LIFT1_THIRD_ANGLE         LIFT_ROUNDS_TO_ANGLE(LIFT1_THIRD_ROUNDS) // 升降台 1 第三档目标角度，单位 rad。EN: Lift1 third-stage target angle, rad.

// ==================== 升降台 2 高度参数 / Lift 2 Height Parameters ====================
#define LIFT2_LOW_ROUNDS          1.0f          // 升降台 2 第一档高度，单位输出轴圈数。EN: Lift2 first-stage height, output shaft rounds.
#define LIFT2_HIGH_ROUNDS         3.0f          // 升降台 2 第二档高度，单位输出轴圈数。EN: Lift2 second-stage height, output shaft rounds.
#define LIFT2_THIRD_ROUNDS        5.0f          // 升降台 2 第三档高度，单位输出轴圈数。EN: Lift2 third-stage height, output shaft rounds.
#define LIFT2_LOW_ANGLE           LIFT_ROUNDS_TO_ANGLE(LIFT2_LOW_ROUNDS)  // 升降台 2 第一档目标角度，单位 rad。EN: Lift2 first-stage target angle, rad.
#define LIFT2_HIGH_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT2_HIGH_ROUNDS) // 升降台 2 第二档目标角度，单位 rad。EN: Lift2 second-stage target angle, rad.
#define LIFT2_THIRD_ANGLE         LIFT_ROUNDS_TO_ANGLE(LIFT2_THIRD_ROUNDS) // 升降台 2 第三档目标角度，单位 rad。EN: Lift2 third-stage target angle, rad.

// ==================== 升降台 1/2 方向 / Lift 1/2 Direction ====================
#define LIFT_MOTOR_CCW_DIR        1.0f           // 逆时针方向系数；当前含义是升降台下降、车体上升。EN: Counter-clockwise direction sign; current meaning: lift descends, robot rises.
#define LIFT_MOTOR_CW_DIR        -1.0f           // 顺时针方向系数；当前含义是升降台向上回零。EN: Clockwise direction sign; current meaning: lift returns upward to zero.

#define LIFT1_MOTOR5_STAGE_DIR   LIFT_MOTOR_CW_DIR  // motor5 到档位高度时的方向。EN: Motor5 direction when Lift1 goes to stage height.
#define LIFT1_MOTOR6_STAGE_DIR   LIFT_MOTOR_CCW_DIR // motor6 到档位高度时的方向。EN: Motor6 direction when Lift1 goes to stage height.
#define LIFT2_MOTOR7_STAGE_DIR   LIFT_MOTOR_CCW_DIR // motor7 到档位高度时的方向。EN: Motor7 direction when Lift2 goes to stage height.
#define LIFT2_MOTOR8_STAGE_DIR   LIFT_MOTOR_CW_DIR  // motor8 到档位高度时的方向。EN: Motor8 direction when Lift2 goes to stage height.
#define LIFT1_MOTOR5_STAGE_SCALE 1.1f               // motor5 抬升高度比例。EN: Motor5 lift stage scale.
#define LIFT1_MOTOR6_STAGE_SCALE 1.1f               // motor6 抬升高度比例；高于 motor5 时调小。EN: Motor6 lift stage scale.
#define LIFT2_MOTOR7_STAGE_SCALE 1.0f               // motor7 抬升高度比例。EN: Motor7 lift stage scale.
#define LIFT2_MOTOR8_STAGE_SCALE 1.0f               // motor8 抬升高度比例；高于 motor7 时调小。EN: Motor8 lift stage scale.

#define LIFT1_MOTOR5_HOME_DIR    LIFT_MOTOR_CCW_DIR // motor5 回零方向。EN: Motor5 direction when Lift1 returns to zero.
#define LIFT1_MOTOR6_HOME_DIR    LIFT_MOTOR_CW_DIR  // motor6 回零方向。EN: Motor6 direction when Lift1 returns to zero.
#define LIFT2_MOTOR7_HOME_DIR    LIFT_MOTOR_CW_DIR  // motor7 回零方向。EN: Motor7 direction when Lift2 returns to zero.
#define LIFT2_MOTOR8_HOME_DIR    LIFT_MOTOR_CCW_DIR // motor8 回零方向。EN: Motor8 direction when Lift2 returns to zero.

// ==================== CAN2 角度电机和升降台 3 / CAN2 Angle Motor and Lift 3 ====================
#define ANGLE_2006_MAX_OMEGA      12.0f          // 角度位置电机最大角速度，单位 rad/s。EN: Max angular speed for angle-position motor, rad/s.
#define CAN2_2006_ANGLE_DIR       1.0f           // CAN2 角度电机方向系数。EN: CAN2 angle motor direction sign.

#define LIFT3_LOW_ROUNDS          2.0f           // 升降台 3 低档位，单位输出轴圈数。EN: Lift3 low stage, output shaft rounds.
#define LIFT3_HIGH_ROUNDS         5.0f           // 升降台 3 高档位，单位输出轴圈数。EN: Lift3 high stage, output shaft rounds.
#define LIFT3_10CM_ROUNDS         1.0f           // 升降台 3 的 10cm 档位，单位输出轴圈数。EN: Lift3 10cm position, output shaft rounds.
#define LIFT3_20CM_ROUNDS         2.0f           // 升降台 3 的 20cm 档位，单位输出轴圈数。EN: Lift3 20cm position, output shaft rounds.
#define LIFT3_40CM_ROUNDS         4.0f           // 升降台 3 的 40cm 档位，单位输出轴圈数。EN: Lift3 40cm position, output shaft rounds.
#define LIFT3_60CM_ROUNDS         6.0f           // 升降台 3 的 60cm 档位，单位输出轴圈数。EN: Lift3 60cm position, output shaft rounds.
#define LIFT3_LOW_ANGLE           LIFT_ROUNDS_TO_ANGLE(LIFT3_LOW_ROUNDS)  // 升降台 3 低档位目标角度，单位 rad。EN: Lift3 low target angle, rad.
#define LIFT3_HIGH_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT3_HIGH_ROUNDS) // 升降台 3 高档位目标角度，单位 rad。EN: Lift3 high target angle, rad.
#define LIFT3_10CM_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT3_10CM_ROUNDS) // 升降台 3 的 10cm 目标角度，单位 rad。EN: Lift3 10cm target angle, rad.
#define LIFT3_20CM_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT3_20CM_ROUNDS) // 升降台 3 的 20cm 目标角度，单位 rad。EN: Lift3 20cm target angle, rad.
#define LIFT3_40CM_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT3_40CM_ROUNDS) // 升降台 3 的 40cm 目标角度，单位 rad。EN: Lift3 40cm target angle, rad.
#define LIFT3_60CM_ANGLE          LIFT_ROUNDS_TO_ANGLE(LIFT3_60CM_ROUNDS) // 升降台 3 的 60cm 目标角度，单位 rad。EN: Lift3 60cm target angle, rad.
#define LIFT3_MAX_OMEGA           10.0f          // 升降台 3 最大角速度指令，单位 rad/s。EN: Lift3 max angular speed command, rad/s.
#define LIFT3_MOTOR13_DIR         1.0f           // 升降台 3 motor13 方向系数。EN: Lift3 motor13 direction sign.
#define LIFT3_MOTOR14_DIR        -1.0f           // 升降台 3 motor14 方向系数；与 motor13 相反。EN: Lift3 motor14 direction sign, opposite of motor13.

#define LIFT3_CMD_ZERO            0              // 升降台 3 指令编号：零点/默认位置。EN: Lift3 command id: zero/default position.
#define LIFT3_CMD_LOW             1              // 升降台 3 指令编号：低档位。EN: Lift3 command id: low stage.
#define LIFT3_CMD_HIGH            2              // 升降台 3 指令编号：高档位。EN: Lift3 command id: high stage.
#define LIFT3_CMD_10CM            3              // 升降台 3 指令编号：10cm。EN: Lift3 command id: 10cm.
#define LIFT3_CMD_20CM            4              // 升降台 3 指令编号：20cm。EN: Lift3 command id: 20cm.
#define LIFT3_CMD_40CM            5              // 升降台 3 指令编号：40cm。EN: Lift3 command id: 40cm.
#define LIFT3_CMD_60CM            6              // 升降台 3 指令编号：60cm。EN: Lift3 command id: 60cm.

// ==================== 侧轮参数 / Side Wheel Parameters ====================
#define SIDE_WHEEL_MAX_RPM        1500.0f        // 侧轮最大速度指令，单位 rpm。EN: Side-wheel max speed command, rpm.
#define SIDE_WHEEL_RPM_TO_RADPS   0.104719755f   // rpm 转 rad/s 的系数。EN: rpm to rad/s conversion.
#define SIDE_WHEEL_CLIMB_FOLLOW_ENABLE 1         // 1 表示爬坡模式下侧轮自动跟随底盘前后速度。EN: 1 lets side wheels follow chassis forward/back speed in climb mode.
#define SIDE_WHEEL_CLIMB_RPM_PER_MM_S 0.6f       // 爬坡跟随比例，底盘前后速度 mm/s 乘以该值得到侧轮 rpm。EN: Climb follow scale from chassis mm/s to side-wheel rpm.
#define SIDE_WHEEL_CLIMB_MIN_MM_S 20.0f          // 前后速度小于该值时侧轮停止，避免零点抖动。EN: Stop side wheels below this forward/back speed to avoid jitter.
#define SIDE_WHEEL9_DIR           1.0f           // 侧轮 motor9 方向系数。EN: Side wheel motor9 direction sign.
#define SIDE_WHEEL10_DIR          1.0f           // 侧轮 motor10 方向系数。EN: Side wheel motor10 direction sign.
#define SIDE_WHEEL11_DIR         -1.0f           // 侧轮 motor11 方向系数。EN: Side wheel motor11 direction sign.
#define SIDE_WHEEL15_DIR         -1.0f           // 侧轮 motor15 方向系数。EN: Side wheel motor15 direction sign.

// ==================== MTF-01P 光流编号 / MTF-01P Optical Flow Index ====================
#define MTF01_MIDDLE_INDEX        0              // 1 号光流，安装在车体中间附近。EN: Optical flow module 1, mounted near chassis center.
#define MTF01_FRONT_INDEX         1              // 2 号光流，安装在车头附近。EN: Optical flow module 2, mounted near chassis front.

// ==================== 光流横漂补偿 / Optical Flow Lateral Hold ====================
#define CHASSIS_FLOW_HOLD_ENABLE          1      // 1 表示启用光流横漂补偿。EN: 1 enables optical-flow lateral drift compensation.
#define CHASSIS_FLOW_HOLD_MTF_INDEX       MTF01_MIDDLE_INDEX // 主光流编号，用于触发和更新补偿。EN: Primary flow module used to trigger/update compensation.
#define CHASSIS_FLOW_HOLD_USE_X_AXIS      1      // 1 使用 flow_x 作为横向轴，0 使用 flow_y。EN: 1 uses flow_x as lateral axis; 0 uses flow_y.
#define CHASSIS_FLOW_HOLD_DIR             1.0f   // 横漂补偿方向系数；越修越偏时改为 -1。EN: Lateral compensation direction sign; change to -1 if compensation is reversed.
#define CHASSIS_FLOW_HOLD_KP              1.2f   // 横向位移 P 增益；越大修正越强。EN: Lateral offset P gain; larger = stronger correction.
#define CHASSIS_FLOW_HOLD_MAX_MM_S        120.0f // 最大横向补偿速度，单位 mm/s；上坡左右平移时调大。EN: Max lateral compensation speed, mm/s.
#define CHASSIS_FLOW_FILTER_ALPHA         0.8f   // 光流横漂速度低延迟滤波系数；越大越跟手，越小越稳。EN: Optical-flow lateral velocity filter alpha; higher = lower latency.
#define CHASSIS_FLOW_HOLD_MIN_FORWARD_MM_S 50.0f // 前后速度超过该值才启用横漂补偿，单位 mm/s。EN: Enable compensation only when forward/backward speed exceeds this, mm/s.
#define CHASSIS_FLOW_HOLD_DUAL_ENABLE     1      // 1 表示融合中间和车头两个光流。EN: 1 fuses middle and front optical flow data.
#define CHASSIS_FLOW_HOLD_MIDDLE_WEIGHT   0.7f   // 两个光流方向一致时，中间光流权重。EN: Middle flow weight when two modules agree.
#define CHASSIS_FLOW_HOLD_FRONT_WEIGHT    0.3f   // 两个光流方向一致时，车头光流权重。EN: Front flow weight when two modules agree.
#define CHASSIS_FLOW_HOLD_DIFF_MAX_MM_S   100.0f // 两个光流速度差小于该值才融合，单位 mm/s。EN: Max speed difference for two-flow fusion, mm/s.
#define CHASSIS_FLOW_HOLD_FRONT_ONLY_MM_S 50.0f  // 车头光流超过该值且中间光流接近静止时，判断为旋转。EN: If front flow is above this while middle is still, treat as rotation.
#define CHASSIS_FLOW_HOLD_MIDDLE_STILL_MM_S 20.0f // 中间光流接近静止的阈值，单位 mm/s。EN: Middle-flow still threshold used by rotation rejection, mm/s.
#define CHASSIS_FLOW_YAW_CORRECT_ENABLE   0      // 1 表示当前后光流差值较大时启用车头纠正。EN: 1 enables heading correction from front-middle flow difference.
#define CHASSIS_FLOW_YAW_DISABLE_IN_CLIMB 1      // 1 表示爬坡模式下关闭光流车头纠正，车头优先由陀螺仪保持。EN: 1 disables optical-flow yaw correction in climb mode; gyro yaw hold has priority.
#define CHASSIS_FLOW_YAW_DIFF_MIN_MM_S    120.0f // 前后光流横向速度差超过该值才纠正车头，单位 mm/s。EN: Min front-middle lateral speed difference for yaw correction, mm/s.
#define CHASSIS_FLOW_YAW_KP               0.002f // 光流差值到旋转修正的 P 增益，单位 rad/s per mm/s。EN: Flow-difference to yaw correction gain, rad/s per mm/s.
#define CHASSIS_FLOW_YAW_MAX_WZ           0.5f   // 光流车头纠正最大角速度，单位 rad/s。EN: Max optical-flow yaw correction angular speed, rad/s.
#define CHASSIS_FLOW_YAW_DIR              1.0f   // 光流车头纠正方向；越修越偏时改为 -1。EN: Optical-flow yaw correction direction; change to -1 if reversed.

// ==================== HWT101 帧参数 / HWT101 Frame Parameters ====================
#define HWT101_FRAME_LEN          11             // HWT101 单帧长度，单位 byte。EN: HWT101 frame length, bytes.
#define HWT101_FRAME_HEAD         0x55           // HWT101 帧头字节。EN: HWT101 frame header byte.
#define HWT101_FRAME_GYRO         0x52           // HWT101 角速度帧 ID。EN: HWT101 gyro frame id.
#define HWT101_FRAME_ANGLE        0x53           // HWT101 角度帧 ID。EN: HWT101 angle frame id.

// ==================== HWT101 航向保持和定角旋转 / HWT101 Yaw Hold and Turn Control ====================
#define HWT101_GYRO_RANGE_DPS     2000.0f        // HWT101 角速度量程，单位 deg/s。EN: HWT101 gyro full-scale range, deg/s.
#define HWT101_YAW_HOLD_KP        0.04f          // 平移时车头保持 P 增益；越大纠偏越强。EN: Yaw-hold P gain during chassis translation; larger = stronger heading correction.
#define HWT101_YAW_HOLD_KD        0.010f         // 平移时车头保持角速度阻尼；越大越抑制摆头。EN: Yaw-hold gyro-rate damping; larger suppresses heading oscillation more.
#define HWT101_YAW_HOLD_MAX_WZ    1.2f           // 车头保持最大修正角速度，单位 rad/s。EN: Max yaw-hold correction angular speed, rad/s.
#define HWT101_YAW_HOLD_CLIMB_KP  0.04f          // 爬坡模式车头保持 P 增益；上坡偏头大时调大。EN: Climb-mode yaw-hold P gain; increase if heading drifts uphill.
#define HWT101_YAW_HOLD_CLIMB_KD  0.010f         // 爬坡模式车头保持角速度阻尼；摆头时调大。EN: Climb-mode yaw-hold gyro-rate damping; increase if heading oscillates.
#define HWT101_YAW_HOLD_CLIMB_MAX_WZ 1.2f        // 爬坡模式车头保持最大修正角速度，单位 rad/s。EN: Climb-mode max yaw-hold correction angular speed, rad/s.
#define HWT101_YAW_HOLD_SLEW_RAD_S2 20.0f        // 车头保持修正角速度斜坡，单位 rad/s^2；越大响应越快。EN: Yaw-hold correction slew rate, rad/s^2; higher = faster response.
#define HWT101_YAW_VALID_TIMEOUT_MS 100          // 陀螺仪数据超时时间，单位 ms；超时后关闭车头保持。EN: HWT101 data timeout in ms; exceeding this disables yaw hold.
#define HWT101_YAW_CORRECT_DIR    1.0f           // 车头保持修正方向；越修越偏时改为 -1。EN: Yaw correction direction sign; change to -1 if heading correction is reversed.

#define HWT101_TURN_KP_MDEG       3000.0f        // 定角旋转 P 增益；输出单位为 mdeg/s 每 deg 误差。EN: Fixed-angle turn P gain, output mdeg/s per deg error.
#define HWT101_TURN_MAX_MDEG_S    90000.0f       // 定角旋转最大速度，单位 mdeg/s；90000=90deg/s。EN: Fixed-angle turn max speed, mdeg/s; 90000 = 90deg/s.
#define HWT101_TURN_DONE_DEG      1.0f           // 旋转误差小于该角度时认为完成，单位 deg。EN: Turn completes when yaw error is below this, deg.

// ==================== 动作组时间参数 / Action Group Timing ====================
#define ACTION_FB_PERIOD_MS       1000           // 动作状态反馈周期，单位 ms；1000ms=1Hz。EN: Action status feedback period, ms; 1000ms = 1Hz.
#define ACTION_LIFT_WAIT_MS       1000           // 升降台动作后的等待时间，单位 ms。EN: Wait time after lift motion step, ms.
#define ACTION_GPIO_WAIT_MS       1000           // GPIO/电磁阀步骤之间的等待时间，单位 ms。EN: Wait time between GPIO valve steps, ms.
#define ACTION_MOTOR_WAIT_MS      1000           // 角度电机动作后的等待时间，单位 ms。EN: Wait time after angle motor step, ms.
#define ACTION_STEP_TIMEOUT_MS    5000           // 通用动作步骤超时时间，单位 ms。EN: Generic action-step timeout, ms.

// ==================== 方块翻转 motor12 角度 / Block Flip Motor12 Angles ====================
#define BLOCK_MOTOR12_CCW_90_ANGLE   (90.0f * 0.01745329252f)  // motor12 逆时针 90 度目标角，单位 rad。EN: Motor12 CCW 90deg target angle, rad.
#define BLOCK_MOTOR12_CCW_180_ANGLE  (180.0f * 0.01745329252f) // motor12 逆时针 180 度目标角，单位 rad。EN: Motor12 CCW 180deg target angle, rad.
#define BLOCK_MOTOR12_CCW_270_ANGLE  (270.0f * 0.01745329252f) // motor12 逆时针 270 度目标角，单位 rad。EN: Motor12 CCW 270deg target angle, rad.

// ==================== 爬台动作距离 / Climb Action Distances ====================
#define CLIMB_UP_FRONT_CHASSIS_MM 200.0f         // 向上爬前半段第一次底盘距离，单位 mm。EN: Up-climb front-half first chassis distance, mm.
#define CLIMB_UP_FRONT_COMBINED_MM 150.0f        // 向上爬前半段底盘+侧轮共同前进距离，单位 mm。EN: Up-climb front-half combined chassis + side-wheel distance, mm.
#define CLIMB_CHASSIS_UP_MM       500.0f         // 向上爬主底盘距离，单位 mm。EN: Main up-climb chassis distance, mm.
#define CLIMB_SIDE_UP_MM          500.0f         // 向上爬主侧轮距离，单位 mm。EN: Main up-climb side-wheel distance, mm.
#define CLIMB_UP_FINISH_EXTRA_CHASSIS_MM 200.0f  // 升降台 2 回零后的额外底盘前进距离，单位 mm。EN: Extra chassis distance after lift2 returns to zero, mm.
#define CLIMB_DOWN_CHASSIS_MM    -300.0f         // 向下爬底盘距离，单位 mm；负数表示后退。EN: Down-climb chassis distance, mm; negative = backward.
#define CLIMB_DOWN_SIDE_MM       -200.0f         // 向下爬侧轮距离，单位 mm；负数表示后退。EN: Down-climb side-wheel distance, mm; negative = backward.

// ==================== 光流距离闭环 / Optical-Flow Distance Closed Loop ====================
#define CHASSIS_DISTANCE_KP       0.35f          // 距离闭环 P 增益；越大速度越快但越容易冲过。EN: Distance-control P gain; larger = faster but easier to overshoot.
#define CHASSIS_DISTANCE_MAX_MM_S 500.0f         // 距离闭环最大底盘速度，单位 mm/s。EN: Distance-control max chassis speed, mm/s.
#define CHASSIS_DISTANCE_MIN_MM_S 30.0f          // 距离闭环最小底盘速度，单位 mm/s；未进入停止范围前使用。EN: Distance-control min chassis speed, mm/s, used before reaching stop band.
#define CHASSIS_DISTANCE_STOP_MM  25.0f          // 距离误差小于该值时停止，单位 mm。EN: Distance error stop threshold, mm.

// ==================== UART 调试开关 / UART Debug Switches ====================
#define UART2_FORWARD_HWT101_RAW    0             // 1 表示把 HWT101 原始字节转发到 UART2，用 HEX 查看。EN: 1 forwards raw HWT101 bytes to UART2 for HEX debug.
#define UART2_FORWARD_UART3_RX      0             // 1 表示把 UART3 收到的上位机原始字节转发到 UART2。EN: 1 forwards raw UART3 host RX bytes to UART2.
#define UART2_HOST_ASCII_DEBUG      0             // 1 表示把解析后的上位机命令用 ASCII 输出到 UART2。EN: 1 outputs parsed host commands as ASCII on UART2.
#define UART2_FORWARD_BUF_SIZE      256           // UART3 到 UART2 转发缓冲区字节数。EN: UART3-to-UART2 forward buffer size in bytes.
#define UART2_DEBUG_MTF             0             // 1 表示在 UART2 输出 MTF 光流调试数据。EN: 1 outputs MTF optical-flow debug data on UART2.
#define UART2_DEBUG_MTF_INDEX       MTF01_MIDDLE_INDEX // UART2 调试输出选择的 MTF 模块。EN: MTF module selected for UART2 debug output.
#define UART3_DEBUG_MTF             0             // 1 表示在 UART3 输出 MTF 光流调试数据。EN: 1 outputs MTF optical-flow debug data on UART3.
#define UART3_DEBUG_MTF_INDEX       MTF01_MIDDLE_INDEX // UART3 调试输出选择的 MTF 模块。EN: MTF module selected for UART3 debug output.

// ==================== 总线舵机参数 / Bus Servo Parameters ====================
#define BUS_SERVO_MOVE_TIME_WRITE      1          // 总线舵机指令 ID：带时间的位置控制。EN: Bus servo command id: move time write.
#define BUS_SERVO_MOVE_TIME_DATA_LEN   7          // 总线舵机移动指令数据长度。EN: Bus servo move command data length.
#define BUS_SERVO_POS_MAX              1500       // 允许的最大舵机位置指令。EN: Max allowed bus servo position command.
#define BUS_SERVO_TIME_DEFAULT_MS      1000       // 舵机默认移动时间，单位 ms。EN: Default bus servo move time, ms.
#define BUS_SERVO_TIME_MAX_MS          30000      // 允许的最大舵机移动时间，单位 ms。EN: Max allowed bus servo move time, ms.
#define BUS_SERVO_TX_QUEUE_SIZE        8          // UART4 总线舵机非阻塞发送队列深度。EN: UART4 bus-servo non-blocking TX queue depth.
#define GRIPPER_SERVO_ID               1          // 夹爪舵机 ID。EN: Gripper bus servo id.
#define GRIPPER_SERVO_GRIP_POS         300        // 夹爪夹取位置。EN: Gripper closed/grip position.
#define GRIPPER_SERVO_GRIP_TIME_MS     2000       // 夹爪夹取动作时间，单位 ms。EN: Gripper grip move time, ms.
#define GRIPPER_SERVO_RELEASE_POS      1000       // 夹爪松开位置。EN: Gripper release/open position.
#define GRIPPER_SERVO_RELEASE_TIME_MS  1000       // 夹爪松开动作时间，单位 ms。EN: Gripper release move time, ms.
#define GRIPPER_PICK_FLIP_EXTRA_DELAY_MS 1500     // `$ACT,15` 中夹爪到位后额外等待再翻转，单位 ms。EN: Extra wait after gripper grip before flip-up in ACT 15, ms.
#define GRIPPER_PICK_FLIP_DELAY_MS     (GRIPPER_SERVO_GRIP_TIME_MS + GRIPPER_PICK_FLIP_EXTRA_DELAY_MS) // 从夹爪指令发出到翻转的总延时。EN: Total delay from grip command to flip-up.
#define GRIPPER_SERVO_BOOT_RELEASE_ENABLE 1       // 1 表示上电后自动让夹爪打开。EN: 1 opens/releases the gripper after boot.
#define GRIPPER_SERVO_BOOT_DELAY_MS    300        // 上电后延时再发送夹爪打开指令，单位 ms。EN: Delay before boot gripper release command, ms.
#define GRIPPER_SERVO_BOOT_RETRY_PERIOD_MS 300    // 上电夹爪打开指令重发周期，单位 ms。EN: Boot gripper release retry period, ms.
#define GRIPPER_SERVO_BOOT_RETRY_COUNT 16         // 上电后 5 秒内夹爪打开指令重发次数。EN: Boot gripper release retry count within the first 5 seconds.
#define GRIPPER_SERVO_BOOT_REPEAT_FOREVER 0       // 1 表示上电后持续重发夹爪打开指令；0 表示按重发次数停止。EN: 1 retries forever after boot; 0 stops after retry count.
#define FLIP_SERVO_ID                  2          // 翻转舵机 ID。EN: Flip bus servo id.
#define FLIP_SERVO_DOWN_POS            860       // 翻转舵机下垂位置。EN: Flip servo hanging-down position.
#define FLIP_SERVO_FLAT_POS            505        // 翻转舵机摆平位置。EN: Flip servo flat position.
#define FLIP_SERVO_UP_POS              110       // 翻转舵机上摆位置。EN: Flip servo upward position.
#define FLIP_SERVO_TIME_MS             1000       // 翻转舵机动作时间，单位 ms。EN: Flip servo move time, ms.
#define FLIP_SERVO_BOOT_DOWN_ENABLE    1          // 1 表示上电后自动让翻转舵机下垂。EN: 1 moves the flip servo down after boot.
#define FLIP_SERVO_BOOT_DELAY_MS       300        // 上电后延时再发送下垂指令，单位 ms。EN: Delay before boot down command, ms.
#define FLIP_SERVO_BOOT_RETRY_PERIOD_MS 300       // 上电下垂指令重发周期，单位 ms。EN: Boot down command retry period, ms.
#define FLIP_SERVO_BOOT_RETRY_COUNT    16         // 上电后 5 秒内下垂指令重发次数，用于等待舵机/BusLinker准备好。EN: Boot down retry count within the first 5 seconds, waits for servo/BusLinker ready.
#define FLIP_SERVO_BOOT_REPEAT_FOREVER 0          // 1 表示上电后持续重发翻转舵机下垂指令；0 表示按重发次数停止。EN: 1 retries forever after boot; 0 stops after retry count.
#ifndef SIDE_WHEEL_FEEDBACK_TIMEOUT_MS
#define SIDE_WHEEL_FEEDBACK_TIMEOUT_MS 50U
#endif
