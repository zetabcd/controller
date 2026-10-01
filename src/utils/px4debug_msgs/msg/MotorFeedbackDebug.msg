# Mechanical ESC telemetry and QUASI-STEADY MODEL estimates, never measured force/torque.
# Published at the rate loop frequency, also when the controller is inactive.
# Repeated esc_sequence means reused feedback; use timestamps/ages/valid bits.
uint16 schema_version 2
uint64 timestamp # ROS-clock us, same as matching ratectrl message
uint64 cycle_id
uint64 actuator_timestamp
uint64 steady_elapsed_us
uint64 esc_sequence # callback count, zero = no esc_status received
uint64 esc_timestamp # copied from EscStatus, PX4/DDS source clock
float32 esc_receive_age_s # host steady time; NaN if never received
uint16 counter
uint8 esc_count
uint8 esc_connectiontype
uint8 esc_online_flags
uint8 esc_armed_flags
# All eight raw EscReport slots are preserved, in the original PX4 order.
uint64[8] report_timestamp
int32[8] esc_rpm # mechanical RPM as reported by PX4, no abs() or pole-pair conversion
float32[8] esc_voltage # V, NaN/zero may mean unsupported in the driver
float32[8] esc_current # A
float32[8] esc_temperature # degrees C
uint32[8] esc_errorcount
uint8[8] esc_address
uint8[8] esc_cmdcount
uint8[8] esc_state
uint8[8] actuator_function
uint16[8] failures
int8[8] esc_power # raw PX4 power %, -1 invalid
float32[8] report_receive_age_s # since this slot's report timestamp last changed
float32[8] report_source_age_s # Relative to newest online EscReport, not outer timestamp; diagnostic only

# Arrays below follow controller motor order 1..4. Mapping uses actuator_function 101..104.
# mapping_mode 0 = actuator_function, 1 = explicitly configured slot mapping.
uint8 mapping_mode
uint16[4] expected_actuator_function
int8[4] esc_slot # -1 missing, -2 ambiguous/duplicate
bool[4] online
bool[4] armed
bool[4] rpm_valid # mapped, fresh report, online, rpm >= 0; failures retained separately
bool[4] electrical_valid # fresh online report, finite V > 0 and finite I >= 0
float64[4] motor_rpm
float64[4] motor_rad # rad/s
float64[4] motor_voltage # V
float64[4] motor_current # A
float64[4] electrical_power # V*I, W (not shaft power)
float64[4] rpm_error # measured minus current requested RPM, NaN without active valid command
float64[4] actuator_control # exact current command; telemetry is not time-aligned to this command
float64[4] desired_motor_rpm

# Estimation method 1 = existing CT/CQ + measured RPM + body airflow (zero wind assumption).
# Method reserved 0 = none. Voltage is recorded, not multiplied into thrust without calibration.
uint8 estimation_method
float64 calibration_voltage # V; legacy nominal metadata only, actual bench voltage unverified
bool airflow_valid
uint64 position_timestamp
uint64 attitude_timestamp
float32 position_age_s
float32 attitude_age_s
float64[3] va_b # FLU m/s
float64 feedback_timeout_s
float64 state_timeout_s
bool[4] estimate_valid
float64[4] cts # N/(rad/s)^2, evaluated using measured RPM
float64[4] cms # N m/(rad/s)^2
float64[4] motor_thrust_est # N
float64[4] motor_reaction_torque_est # positive magnitude, N m
float64[4] mechanical_power_est # Q*omega, W; aerodynamic model, not measured shaft power
bool total_estimate_valid # requires all four motor estimates
float64 thrust_est # sum, N
float64[3] tau_est # FLU N m, arm forces + reaction torque signs [+,-,+,-]
# Model inputs allow independent offline reconstruction even when control is inactive.
float64 air_density
float64 propeller_radius
float64 arm_length
float64 arm_angle_rad
float64[3] ct_coefficients
float64[3] cq_coefficients
