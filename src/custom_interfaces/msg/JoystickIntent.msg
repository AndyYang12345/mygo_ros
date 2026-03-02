# 摇杆意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点

# 轴映射参考（来自 /joy 的 axes）：
# 0: LS_x（+1=左, -1=右）
# 1: LS_y（+1=上, -1=下）
# 2: LT（+1=释放, -1=按下）
# 3: RS_x（+1=左, -1=右）
# 4: RS_y（+1=上, -1=下）
# 5: RT（+1=释放, -1=按下）
# 6: Dpad_x（+1=左箭头, -1=右箭头）
# 7: Dpad_y（+1=上箭头, -1=下箭头）

# 摇杆编号
# 0: 左摇杆
# 1: 右摇杆
# 2: 十字键（Dpad）
uint8 joystick_id

# 摇杆位置
# 范围: -1.0 到 1.0
float32 x  # 左右: 左负右正
float32 y  # 前后: 上正下负（根据手柄坐标系调整）

# 原始值（可选，用于需要精确控制的场景）
float32 raw_x  # 未经死区处理的原始值
float32 raw_y

# 时间戳
builtin_interfaces/Time timestamp

# 事件来源
string source

# 摇杆状态
# 0: NORMAL - 正常移动
# 1: CENTERED - 回中
# 2: EDGE - 推到边缘
uint8 state