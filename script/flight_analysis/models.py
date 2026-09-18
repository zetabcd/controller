"""Derived quantities using the same conventions/formulas as the controller."""
import hashlib
from pathlib import Path
import warnings
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .ulog_data import Signal, hold


def rotation_matrices(q):
    norms = np.linalg.norm(q, axis=1)
    valid = np.isfinite(q).all(axis=1) & (norms > 1e-9)
    result = np.full((len(q), 3, 3), np.nan)
    if valid.any():
        result[valid] = Rotation.from_quat((q[valid] / norms[valid, None])[:, [1, 2, 3, 0]]).as_matrix()
    return result


def derive(log, max_age, params_path=None):
    s = log.signals
    params, provenance = {}, {'parameter_file': None, 'calibration_voltage_v': 16.0, 'parameter_sources': {}}
    if params_path:
        path = Path(params_path).resolve()
        raw = path.read_bytes()
        loaded = yaml.safe_load(raw)
        params = loaded.get('px4ctrl_node', loaded).get('ros__parameters', loaded)
        provenance.update(parameter_file=str(path), sha256=hashlib.sha256(raw).hexdigest(), parameters=params)
        log.notes.append('离线模型使用显式提供的参数文件；需由使用者保证它与本次飞行一致。16 V 标定不进行未经标定的电压补偿。')

    def assign(key, base, values, unit, description):
        s[key] = Signal(base.t.copy(), values, unit, description, 'FLU / controller world')

    for key, target in (('attitude', 'euler'), ('attitude_des', 'euler_des')):
        if key in s:
            base = s[key]
            matrices = rotation_matrices(base.y)
            valid = np.isfinite(matrices).all(axis=(1, 2))
            values = np.full((len(base.t), 3), np.nan)
            if valid.any():
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', UserWarning)
                    values[valid] = Rotation.from_matrix(matrices[valid]).as_euler('xyz', degrees=True)
            assign(target, base, values, 'deg', key + ': roll/pitch/yaw display; singularities possible')

    for key, output in (('thrust_des', 'thrust_delta'), ('rate_des', 'rate_delta')):
        if key in s:
            base = s[key]
            values = np.full_like(base.y, np.nan)
            values[1:] = np.diff(base.y, axis=0)
            values[1:][(np.diff(base.t) <= 0) | (np.diff(base.t) > max_age)] = np.nan
            assign(output, base, values, base.unit, 'sample-to-sample command change; no difference across gaps')
    if 'state' in s:
        base = s['state']
        for key, output in (('position', 'position_age'), ('attitude', 'attitude_age'), ('rate', 'imu_age'),
                            ('position_ref', 'debug_age')):
            if key in s:
                _, age = hold(s[key], base.t, max_age)
                assign(output, base, age, 's', 'receive age at FSM/debug sample; not transport latency')
    if all(key in s for key in ('p_term', 'i_term', 'd_term', 'alpha_ff', 'alpha_des')):
        base = s['alpha_des']
        total = sum(hold(s[key], base.t, max_age)[0] for key in ('p_term', 'i_term', 'd_term', 'alpha_ff'))
        assign('pid_sum', base, total, 'rad/s²', 'P + I_used + D + feedforward')
        assign('pid_identity_residual', base, total - base.y, 'rad/s²', 'PID sum minus recorded angular acceleration command; algebraic consistency')
    if 'effectiveness' in s:
        base = s['effectiveness']
        matrices = base.y.reshape(-1, 4, 4)
        valid = np.isfinite(matrices).all(axis=(1, 2))
        condition = np.full(len(base.t), np.nan)
        if valid.any():
            condition[valid] = np.linalg.cond(matrices[valid])
        assign('allocation_condition', base, condition, 'mixed-unit condition number',
               'raw effectiveness matrix condition number; not dimensionless physical controllability')

    # fb_a is currently never populated; ref_a is also left at zero by MPC.
    # Differentiate the actually published reference velocity, without crossing gaps.
    if 'velocity_ref' in s:
        base = s['velocity_ref']
        values = np.full_like(base.y, np.nan)
        valid = np.flatnonzero(np.isfinite(base.y).all(axis=1))
        if len(valid) > 2:
            cuts = np.flatnonzero((np.diff(valid) > 1) | (np.diff(base.t[valid]) > max_age)
                                 | (np.diff(base.t[valid]) <= 0)) + 1
            for idx in np.split(valid, cuts):
                if len(idx) >= 3:
                    values[idx[1:-1]] = np.gradient(base.y[idx], base.t[idx], axis=0)[1:-1]
        assign('acceleration_ref', base, values, 'm/s²', 'numerical derivative of published reference velocity; endpoints/gaps excluded')
        log.notes.append('fb_a 在当前控制器中未赋值，MPC 的 ref_a 也可能为默认零；不把它们当作有效加速度。实际加速度取 PX4 local_position 的速度导数，参考加速度由 ref_v 分段差分，包含数值微分误差。')

    def parameter(name, count, base, section, key):
        # Prefer recorded per-cycle parameters over a current checkout's YAML.
        for topic in ('/debugPx4/motor_feedback', '/debugPx4/ratectrl'):
            d = log.topic(topic)
            fields = [name] if count == 1 else [f'{name}[{i}]' for i in range(count)]
            values = log.fields(d, fields)
            if values is not None:
                times = log.times(d)
                order = np.argsort(times, kind='stable')
                sig = Signal(times[order], values[order])
                aligned, _ = hold(sig, base.t, max_age)
                provenance['parameter_sources'][name] = {'source': d.name, 'field': name,
                    'first_finite_value': next((row.tolist() for row in aligned if np.isfinite(row).all()), None)}
                return aligned
        values = params.get(section, {}).get(key) if section else params.get(key)
        if values is None:
            return None
        values = np.asarray(values, dtype=float).reshape(1, -1)
        if values.shape[1] != count:
            return None
        provenance['parameter_sources'][name] = {'source': str(params_path), 'parameter': section + '.' + key,
                                                 'value': values[0].tolist()}
        return np.repeat(values, len(base.t), axis=0)

    if 'motor_thrust' in s and 'thrust_sol' not in s:
        assign('thrust_sol', s['motor_thrust'], np.sum(s['motor_thrust'].y, axis=1), 'N', 'sum of allocated motor thrust (model)')
    if 'rate' in s and 'alpha' in s and 'tau_inverse' not in s:
        base = s['alpha']
        inertia = parameter('inertia_diagonal', 3, base, 'uav', '_inertia')
        if inertia is None and all(k in params.get('uav', {}) for k in ('Jvx', 'Jvy', 'Jvz')):
            inertia = np.tile([params['uav'][k] for k in ('Jvx', 'Jvy', 'Jvz')], (len(base.t), 1))
            provenance['parameter_sources']['inertia_diagonal'] = {'source': str(params_path), 'value': inertia[0].tolist() if len(inertia) else []}
        if inertia is not None:
            rate, _ = hold(s['rate'], base.t, max_age)
            assign('tau_inverse', base, inertia * base.y + np.cross(rate, inertia * rate), 'N m (estimate)',
                   'J*angular acceleration + omega cross J*omega; net inverse-dynamics torque, includes disturbances')
    if 'tau_des' in s and 'tau_sol' in s:
        base = s['tau_sol']
        target, _ = hold(s['tau_des'], base.t, max_age)
        assign('tau_residual', base, target - base.y, 'N m', 'desired minus allocated model torque')
        residual = target - base.y
        for name, flag in (('allocation_positive_inferred', residual > 1e-6),
                           ('allocation_negative_inferred', residual < -1e-6)):
            assign(name, base, np.where(np.isfinite(residual), flag.astype(float), np.nan),
                   'flag', 'allocation residual exceeds +/-1e-6 N m; inferred, not logged saturation flags')
    if 'thrust_des' in s and 'thrust_sol' in s:
        base = s['thrust_sol']
        target, _ = hold(s['thrust_des'], base.t, max_age)
        assign('thrust_residual', base, target - base.y, 'N', 'desired minus allocated total thrust')
    if 'cts' not in s and 'rpm_des' in s and 'motor_thrust' in s:
        base = s['motor_thrust']
        rpm, _ = hold(s['rpm_des'], base.t, max_age)
        omega2 = (rpm * 2 * np.pi / 60) ** 2
        ct = np.divide(base.y, omega2, out=np.full_like(base.y, np.nan), where=omega2 > 1e-6)
        assign('cts_reconstructed', base, ct, 'N/(rad/s)²', 'allocated thrust / requested omega²; inferred controller coefficient')
    # Distinguish physical battery measurements from a scalar debug voltage.
    if 'voltage' in s and 'current' in s:
        base = s['current']
        voltage, _ = hold(s['voltage'], base.t, max_age)
        values = voltage * base.y
        values[(voltage <= 0) | (base.y < 0)] = np.nan
        assign('battery_power', base, values, 'W', 'battery V*I; electrical, not shaft power')

    # ESC sequence counts callbacks, not rate-controller iterations. Avoid weighting
    # a cached report hundreds of times in the RPM/electrical statistics.
    if 'esc_sequence' in s:
        seq = s['esc_sequence']
        new = np.r_[True, np.diff(seq.y[:, 0]) != 0] & (seq.y[:, 0] > 0)
        for key in ('rpm_measured', 'motor_voltage', 'motor_current', 'electrical_power'):
            if key in s and np.array_equal(s[key].t, seq.t):
                old = s[key]
                s[key] = Signal(old.t[new], old.y[new], old.unit, old.source + ' (new ESC reports only)', old.frame)
    if 'rpm_measured' not in s:
        return provenance
    base = s['rpm_measured']
    omega = base.y * 2 * np.pi / 60  # mechanical RPM, no pole-pair multiplier
    omega[omega < 0] = np.nan
    air, _ = hold(s.get('feedback_airflow', s.get('airflow')), base.t, max_age)
    if air.shape[1] != 3 or not np.isfinite(air).any():
        if 'velocity' in s and 'attitude' in s:
            vel, _ = hold(s['velocity'], base.t, max_age)
            q, _ = hold(s['attitude'], base.t, max_age)
            air = np.einsum('nji,nj->ni', rotation_matrices(q), vel)
            log.notes.append('离线 ESC 反算来流采用实际姿态旋转地速，假设风速为零；不使用期望姿态代替实际姿态。')
        else:
            log.notes.append('缺少有效实际姿态/来流，不能离线反算 ESC 推力；不会默认使用零来流。')
            return provenance
    coefficients = []
    for field, prefix in (('ct_coefficients', 'Ct'), ('cq_coefficients', 'Cq')):
        values = parameter(field, 3, base, 'motor', '_unused')
        if values is None and all(prefix + '_' + k in params.get('motor', {}) for k in 'abc'):
            values = np.tile([params['motor'][prefix + '_' + k] for k in 'abc'], (len(base.t), 1))
            provenance['parameter_sources'][field] = {'source': str(params_path), 'value': values[0].tolist() if len(values) else []}
        coefficients.append(values)
    rho = parameter('air_density', 1, base, 'aero', 'rho')
    radius = parameter('propeller_radius', 1, base, 'uav', 'rp')
    arm = parameter('arm_length', 1, base, 'uav', 'l')
    beta = parameter('arm_angle_rad', 1, base, 'uav', '_beta_rad')
    if beta is None and 'beta_deg' in params.get('uav', {}):
        beta = np.full((len(base.t), 1), np.deg2rad(params['uav']['beta_deg']))
        provenance['parameter_sources']['arm_angle_rad'] = {'source': str(params_path), 'beta_deg': params['uav']['beta_deg']}
    if any(x is None for x in (*coefficients, rho, radius, arm, beta)):
        log.notes.append('离线反算缺少 CT/CQ 多项式、空气密度或桨/机架尺寸；可提供本次飞行的 --params 参数文件。')
        return provenance
    ct_poly, cq_poly = coefficients
    j = np.clip(np.pi * air[:, 2:3] / (omega * radius + 1e-8), 0, 1e10)
    def coefficient(poly, factor):
        raw = poly[:, :1] * j*j + poly[:, 1:2] * j + poly[:, 2:3]
        return np.clip(raw, poly[:, 2:3] / 10, poly[:, 2:3]) * factor
    ct = coefficient(ct_poly, 4 * rho * radius**4 / np.pi**2)
    cq = coefficient(cq_poly, 8 * rho * radius**5 / np.pi**2)
    valid_model = (rho[:, 0] > 0) & (radius[:, 0] > 0) & (arm[:, 0] > 0)
    ct[~valid_model], cq[~valid_model] = np.nan, np.nan
    thrust, reaction = ct * omega**2, cq * omega**2
    tau = np.column_stack((arm[:, 0] * np.sin(beta[:, 0]) * (thrust @ [1, -1, -1, 1]),
                           arm[:, 0] * np.cos(beta[:, 0]) * (thrust @ [-1, -1, 1, 1]),
                           reaction @ [1, -1, 1, -1]))
    for key, y, unit in (('offline_cts', ct, 'N/(rad/s)²'), ('offline_cms', cq, 'N m/(rad/s)²'),
                         ('offline_motor_thrust', thrust, 'N (model)'), ('offline_reaction', reaction, 'N m (model)'),
                         ('offline_thrust', thrust.sum(axis=1), 'N (model)'), ('offline_tau', tau, 'N m (model)'),
                         ('offline_mechanical_power', reaction * omega, 'W (model)')):
        assign(key, base, y, unit, 'offline CT/CQ + mechanical ESC RPM + measured-state airflow; quasi-steady model')
    for target, key in (('thrust_est_motor', 'offline_motor_thrust'), ('reaction_est', 'offline_reaction'),
                         ('thrust_est', 'offline_thrust'), ('tau_est', 'offline_tau'),
                         ('mechanical_power', 'offline_mechanical_power')):
        if target not in s:
            s[target] = s[key]
    return provenance
