# 按键意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点

# 按键编号
# 手柄映射：
# 0: A
# 1: B
# 2: X
# 3: Y
# 4: LB
# 5: RB
# 6: -
# 7: +
# 8: HOME
# 9: LS（左摇杆按下）
# 10: RS（右摇杆按下）
uint8 button_id

# 事件类型
# 0: PRESS - 按键按下
# 1: RELEASE - 按键释放
# 2: LONG_PRESS - 长按（>1秒）
# 3: DOUBLE_CLICK - 双击
uint8 event_type

# 时间戳
builtin_interfaces/Time timestamp

# 事件来源
# "joystick" - 来自手柄
# "gui" - 来自GUI
# "voice" - 来自语音
# "keyboard" - 来自键盘
string source

# 附加数据（可选，用于扩展）
float32 pressure  # 按键压力（如果支持）