# 扳机意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点
# 扳机是模拟量输入，可以感知按压力度

# 扳机编号
# 0: LT (左扳机) - Left Trigger
# 1: RT (右扳机) - Right Trigger
# 2: LT2 (如果有第二个左扳机)
# 3: RT2 (如果有第二个右扳机)
uint8 trigger_id

# 扳机按压力度
# 范围: 0.0 到 1.0
# 0.0: 完全松开
# 1.0: 完全按下
# 中间值表示不同程度的按下
float32 value

# 触发阈值分段（可选，用于更精细的控制）
# 根据value自动计算
# 0: RELEASED - 松开 (value < 0.05)
# 1: LIGHT_TOUCH - 轻触 (0.05 <= value < 0.2)
# 2: HALF_PRESS - 半按 (0.2 <= value < 0.6)
# 3: FULL_PRESS - 全按 (0.6 <= value < 0.9)
# 4: HARD_PRESS - 用力按 (value >= 0.9)
uint8 press_level

# 事件类型
# 0: UPDATE - 值更新（连续变化）
# 1: PRESSED - 从无到有 (value从<0.05变为>=0.05)
# 2: RELEASED - 从有到无 (value从>=0.05变为<0.05)
# 3: FULL_PRESS_REACHED - 达到全按阈值
# 4: HALF_PRESS_REACHED - 达到半按阈值
uint8 event_type

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