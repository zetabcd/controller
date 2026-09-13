# sun: 顶层控制器发送给高频角速度环的接口，所有向量均按内部 FLU 机体系表达。
uint64 timestamp	# time since system start (microseconds)

float32 thrust      # N
float32[3] bodyrates		# [rad/s] bodyrate setpoint
# sun: rate_dot_ref 是轨迹微分平坦产生的角加速度前馈，不包含 PID 反馈项。
float32[3] rate_dot_ref		# [rad/s^2] feedforward angular acceleration
