# sun: 顶层状态机和外环控制调试消息；timestamp 使用 PX4 风格的微秒时间。
uint64 timestamp

# sun: state 对应 PX4CtrlFSM::procedure_id_ 的当前状态编号。
uint32 state 

float32 ref_p_x
float32 ref_p_y
float32 ref_p_z

float32 ref_v_x
float32 ref_v_y
float32 ref_v_z

float32 fb_a_x
float32 fb_a_y
float32 fb_a_z

float32 ref_a_x
float32 ref_a_y
float32 ref_a_z

float32 ref_rate_x
float32 ref_rate_y
float32 ref_rate_z

float32 ref_rate_dot_x
float32 ref_rate_dot_y
float32 ref_rate_dot_z

float32 des_a_x
float32 des_a_y
float32 des_a_z

float32 des_q_x
float32 des_q_y
float32 des_q_z
float32 des_q_w
float32 des_pitch
float32 des_roll
float32 des_yaw

float32 des_rate_x
float32 des_rate_y
float32 des_rate_z

float32 des_thrust
float32 voltage

# sun: 在线辨识得到的推力信号到竖直加速度比例，用于观察模型收敛。
float32 thr2acc

