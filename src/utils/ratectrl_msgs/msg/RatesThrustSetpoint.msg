# sun: 顶层控制器发送给高频角速度环的接口，所有向量均按内部 FLU 机体系表达。
uint64 timestamp	# time since system start (microseconds)

float32 thrust      # N
float32[3] bodyrates		# [rad/s] bodyrate setpoint
# Nominal angular acceleration in current FLU body axes, held until next message.
# Contains no feedback contribution. Invalid means explicitly disabled, not stale.
float32[3] rate_dot_ref		# [rad/s^2] feedforward angular acceleration

bool rate_dot_ref_valid
