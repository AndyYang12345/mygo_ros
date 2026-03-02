# 扳机意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点
# 扳机是模拟量输入，可以感知按压力度

# 扳机编号
# 0: LT (左扳机) - Left Trigger
# 1: RT (右扳机) - Right Trigger
uint8 trigger_id

# 扳机按压力度
# 范围: -1.0 到 1.0
# 1.0: 完全松开
# -1.0: 完全按下
# 中间值表示不同程度的按下
float32 value

# 事件类型
# 0: RELEASED (完全松开)
# 1: FULLY_PRESSED (完全按下)
# 2: PRESSED (按下但未完全按下)
uint8 event_type
a
# 时间戳
builtin_interfaces/Time timestamp

# 事件来源
# "joystick" - 来自手柄
# "gui" - 来自GUI
# "voice" - 来自语音
# "keyboard" - 来自键盘
string source

# 原始ADC值（可选，用于调试）
uint16 raw_value