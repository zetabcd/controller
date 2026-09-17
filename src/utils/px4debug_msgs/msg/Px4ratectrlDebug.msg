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

# Schema 2. Legacy scalar vectors above retain FRD signs (x forward, y right, z down).
# ALL new body vectors below are controller FLU (x forward, y left, z up).
# Motor array indices 0..3 = controller motors 1..4, NOT esc_status array slots.
# cur_tau_* / tau_sol are ALLOCATION MODEL results, never measured torque.
uint16 schema_version 2
uint64 cycle_id
# ROS-clock microseconds; source timestamps are copied unchanged from input messages.
uint64 control_start_timestamp
uint64 actuator_timestamp
# Host steady clock, microseconds since rate node construction; independent of ROS /clock.
uint64 steady_elapsed_us
float64 elapsed_s
float64 dt_s
bool control_updated
bool control_inputs_valid
bool control_result_finite
bool deadline_missed
# Complete previous runControlCycle(), including both diagnostic publications.
# NaN on the first cycle. cycle_interval_s also includes intervening spin/sleep.
float32 previous_cycle_time_ms
float64 cycle_interval_s
uint32 fsm_state
bool is_take_off
# Source order: gyro, local position, attitude, rate/thrust setpoint, outer debug.
uint64[5] input_timestamp
uint64[5] input_sequence
# Host steady time since last receipt; NaN when never received. Not transport latency.
float32[5] input_age_s
bool[5] input_valid
uint64 gyro_integral_dt_us
uint8 gyro_clipping # raw SensorCombined clipping bit mask (FRD x/y/z bits)
uint8 gyro_calibration_count

# rad/s, rad/s^2. LPF is observed only; the existing TVR uses rate_cur (raw gyro).
float64[3] rate_cur
float64[3] rate_cur_lpf
float64[3] rate_des
float64[3] rate_err
float64[3] rate_err_integrator
float64[3] rate_dot_ref
float64[3] rate_dot_cur
float64[3] ome_dot_des
# P + I_used + D + feedforward = ome_dot_des; integral updated AFTER this output.
float64[3] p_term
float64[3] i_term
float64[3] d_term
float64[3] ome_int_after
float64[3] i_factor
float64[3] i_candidate
bool[3] i_clipped
bool[3] i_update_finite
bool[3] saturation_positive_used
bool[3] saturation_negative_used
bool[3] saturation_positive
bool[3] saturation_negative
float64[3] gain_p
float64[3] gain_i
float64[3] gain_d
float64[3] integral_limit
float64[3] inertia_diagonal # kg m^2
float64[3] gyro_torque # omega cross (J omega), N m
float64[3] tau_from_rate_dot # J*rate_dot_cur + gyro_torque, inverse-dynamics estimate, N m
float64[3] tau_des # N m
float64[3] tau_sol # N m, clipped allocation model
float64[3] tau_residual # desired minus allocated, N m
float64 thrust_des # N
float64 thrust_sol # N
float64 thrust_residual # N

# Model working point. Controller world frame preserves existing x,-y,-z conversion.
float64[3] velocity_world # m/s; exact input used, including existing reset-delta subtraction
float64[3] wind_world # m/s, currently assumed zero
float64[3] va_b # FLU m/s
float64[4] attitude_wxyz # controller body-to-world quaternion
float64[4] motor_rad_for_model # previous desired angular speeds, rad/s
float64[4] cts # N/(rad/s)^2
float64[4] cms # N m/(rad/s)^2
float64[16] effectiveness # row-major: [total thrust, FLU torque xyz] = E * motor thrust
float64[16] mix # row-major; inverse after existing small-element zeroing
float64 effectiveness_determinant
float64[4] motor_thrust_raw # before clipping, N
float64[4] motor_thrust_sol # after clipping, N
bool[4] thrust_clipped_low
bool[4] thrust_clipped_high
float64 thrust_min # per motor, N
float64 thrust_max # per motor, N
float64[4] motor_rad_sol # rad/s
float64[4] motor_reaction_torque_sol # cm * omega_des^2, positive magnitude, N m
float64[4] thro_discriminant
float64[4] thro_setpoint_i # inverse speed curve result before final min(...,1)
bool[4] speed_curve_limited # above parabola maximum: existing controller returns throttle 1
float64[4] thro_setpoint # calculated normalized command before fallback
float64[4] actuator_control # exact float values sent to PX4 after fallback; -1 when inactive
bool[4] actuator_fallback # NaN calculation replaced by previous valid command
float64[3] rc2speed_coefficients # [a,b,c]: omega = a*u^2 + b*u + c, rad/s
float64 calibration_voltage # V; metadata, not an online voltage compensation
float64[3] ct_coefficients # dimensionless CT(J) polynomial [a,b,c]
float64[3] cq_coefficients # dimensionless CQ(J) polynomial [a,b,c]
float64 air_density # kg/m^3
float64 propeller_radius # m
float64 arm_length # m
float64 arm_angle_rad
float64 nominal_rate_hz
float64[3] gyro_lpf_cutoff_hz
# TVR status: 0 warmup; 1 success; 2 symbolic failure; 3 numeric failure; 4 solve failure.
uint8[3] tvr_status
bool[3] tvr_output_valid
uint32[3] tvr_samples
float64[3] tvr_mean_dt_s
float64[3] tvr_time_ms
# [window_size, lambda_tv, extrapolation_count, fit_count, attenuation, order, weight_scale]
float64[7] tvr_parameters
# Control calculation timing excludes actuator/debug serialization and publication.
# work_time_ms includes actuator publish + ESC estimation, excludes debug publishing/spin/sleep.
float32 work_time_ms
