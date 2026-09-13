# sun: 底层角速度环调试消息，记录角加速度、力矩和四电机分配结果。
uint64 timestamp

float32 des_rate_dot_x
float32 des_rate_dot_y
float32 des_rate_dot_z

float32 cur_rate_dot_x
float32 cur_rate_dot_y
float32 cur_rate_dot_z

float32 des_tau_x
float32 des_tau_y
float32 des_tau_z

float32 cur_tau_x
float32 cur_tau_y
float32 cur_tau_z

float32[4] des_motor_rpm

# sun: des_u_1~4 为限幅后的单电机推力解，单位 N。
float32 des_u_1
float32 des_u_2
float32 des_u_3
float32 des_u_4

float32 solve_time_ms
