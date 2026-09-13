# sun: 仿真遥控器输入；主通道和辅助通道均归一化，时间戳单位为微秒。
uint64 timestamp

# Stick positions [-1,1]
float32 roll     
float32 pitch    
float32 yaw      
float32 throttle

# sun: aux1~aux4 由控制节点量化为 DOWN/MID/UP 三档，用于模式、解锁和安全开关。
float32 aux1
float32 aux2
float32 aux3
float32 aux4
