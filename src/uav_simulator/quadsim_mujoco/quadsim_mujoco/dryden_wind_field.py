import numpy as np

# sun: DrydenWindField 在频域按 Dryden 功率谱合成三轴湍流序列，再按仿真步长逐点输出。
# sun: 固定随机种子使同一组控制参数可在完全相同的风扰下重复比较。

class DrydenWindField:
        """
        适配无人机动力学模型的Dryden风场类（支持逐步获取风场噪声）
        
        核心特性：
        1. 预先生成指定时长的风场序列，按步调用
        2. 支持循环模式（风场数据耗尽后自动复用）
        3. 输出与无人机动力学步长匹配的风场扰动（u,v,w）
        """
        def __init__(
            self,
            U0=3.0,          # 平均风速 (m/s)
            height=10.0,    # 飞行高度 (m)
            dt=0.01,          # 无人机动力学计算步长 (s)
            duration=200.0,    # 预生成风场的总时长 (s)
            seed=42,          # 随机种子（保证可复现）
            loop=True         # 是否循环复用风场数据
        ):
            # 基础参数
            self.U0 = U0
            self.height = height
            self.dt = dt
            self.duration = duration
            self.loop = loop
            self.seed = seed
            
            # 初始化状态
            self.current_step = 0  # 当前步数
            self.total_steps = int(duration / dt)  # 预生成总步数
            self.fs = 1 / dt  # 采样频率 (Hz)
            
            # 预先生成风场序列（u/v/w）
            self.t, self.u_series, self.v_series, self.w_series = self._generate_wind_series()
        
        def _generate_wind_series(self):
            """内部方法：预生成风场时间序列"""
            # sun: 整段预生成可把 FFT 开销移出实时仿真循环，代价是占用与 duration/dt 成正比的内存。
            np.random.seed(self.seed)
            
            N = self.total_steps
            t = np.linspace(0, self.duration, N, endpoint=False)
            f = np.fft.fftfreq(N, self.dt)  # 频率轴
            
            # sun: 湍流尺度随高度变化并设置下限，避免近地高度导致相关长度过短。
            L_u = max(0.7 * self.height, 200)   # 纵向湍流尺度
            L_v = L_u                          # 侧向湍流尺度
            L_w = 0.5 * L_u                    # 垂向湍流尺度
            
            # 湍流强度
            sigma_u = 0.1 * self.U0 if self.height > 300 else 0.2 * self.U0
            sigma_v = sigma_u / np.sqrt(3)
            sigma_w = sigma_u / np.sqrt(3)
            
            # 功率谱密度（PSD）函数
            def psd_u(freq):
                freq_safe = np.where(freq == 0, 1e-10, freq)
                return (2 * sigma_u**2 * L_u / self.U0) / (1 + (L_u * freq_safe / self.U0)**2)
            
            def psd_v(freq):
                freq_safe = np.where(freq == 0, 1e-10, freq)
                return (sigma_v**2 * L_v / self.U0) * (1 + 3*(L_v * freq_safe / self.U0)**2) / (1 + (L_v * freq_safe / self.U0)**2)**2
            
            def psd_w(freq):
                freq_safe = np.where(freq == 0, 1e-10, freq)
                return (sigma_w**2 * L_w / self.U0) * (1 + 3*(L_w * freq_safe / self.U0)**2) / (1 + (L_w * freq_safe / self.U0)**2)**2
            
            # sun: 随机相位乘以功率谱幅值构造频域样本，逆 FFT 后再校准到目标标准差。
            S_u = psd_u(f)
            S_v = psd_v(f)
            S_w = psd_w(f)
            
            phi_u = 2 * np.pi * np.random.rand(N)
            phi_v = 2 * np.pi * np.random.rand(N)
            phi_w = 2 * np.pi * np.random.rand(N)
            
            F_u = np.sqrt(S_u) * np.exp(1j * phi_u)
            F_v = np.sqrt(S_v) * np.exp(1j * phi_v)
            F_w = np.sqrt(S_w) * np.exp(1j * phi_w)
            
            # 逆FFT得到时域序列并归一化
            u = np.fft.ifft(F_u).real
            v = np.fft.ifft(F_v).real
            w = np.fft.ifft(F_w).real
            
            u = u * (sigma_u / np.std(u))
            v = v * (sigma_v / np.std(v))
            w = w * (sigma_w / np.std(w))
            
            return t, u, v, w
        
        def get_wind(self):
            """
            获取当前步的风场噪声（核心接口）
            返回：当前步的纵向(u)、侧向(v)、垂向(w)风场扰动 (m/s)
            """
            # sun: 循环模式使用模运算复用序列；非循环模式耗尽后返回静风。
            if self.loop:
                step = self.current_step % self.total_steps
            else:
                # 非循环模式：超出长度后返回0
                if self.current_step >= self.total_steps:
                    return 0.0, 0.0, 0.0
                step = self.current_step
            
            # 获取当前步的风场值
            u = self.u_series[step]
            v = self.v_series[step]
            w = self.w_series[step]
            
            # 步数自增
            self.current_step += 1
            
            return u, v, w
        
        def set_dt(self, new_dt):
            """
            更新动力学计算步长（dt），并重新生成风场序列以匹配新的步长
            参数：new_dt - 新的动力学计算步长 (s)
            """
            self.dt = new_dt
            self.fs = 1 / new_dt
            self.total_steps = int(self.duration / new_dt)
            # sun: 步长改变会同时改变频率轴和样本总数，必须重新生成而不能只调整读取速度。
            self.t, self.u_series, self.v_series, self.w_series = self._generate_wind_series()
        
        def reset_seed(self, new_seed=None):
            """
            重置风场种子（重置步数，可选重新生成风场）
            参数：new_seed - 新的随机种子（None则使用原种子）
            """
            self.current_step = 0
            if new_seed is not None:
                self.seed = new_seed
                # 重新生成风场序列
                self.t, self.u_series, self.v_series, self.w_series = self._generate_wind_series()
                
        def reset(
            self,
            U0=3.0,          # 平均风速 (m/s)
            height=10.0,    # 飞行高度 (m)
            dt=0.01,          # 无人机动力学计算步长 (s)
            duration=200.0,    # 预生成风场的总时长 (s)
            seed=42,          # 随机种子（保证可复现）
            loop=True         # 是否循环复用风场数据
        ):
            """
            重置风场参数并重新生成风场序列
             - 参数与初始化一致，调用后会重置当前步数并生成新的风场数据
             - 可用于在仿真过程中动态调整风场特性
             - 注意：调用后会覆盖原有参数和风场数据
             - 参数：
                U0: 平均风速 (m/s)
                height: 飞行高度 (m)
                dt: 无人机动力学计算步长 (s)
                duration: 预生成风场的总时长 (s)
                seed: 随机种子（保证可复现）
                loop: 是否循环复用风场数据
             - 返回：无（直接修改实例状态）
             - 使用示例：
                wind_field.reset(U0=5.0, height=20.0, dt=0.02, duration=300.0, seed=123, loop=False)
             - 注意事项：
                1. 调用后会立即生效，当前步数重置为0
                2. 如果不需要修改某些参数，可以传入原值或使用默认值
            """
            # 基础参数
            self.U0 = U0
            self.height = height
            self.dt = dt
            self.duration = duration
            self.loop = loop
            self.seed = seed
            
            # 初始化状态
            self.current_step = 0  # 当前步数
            self.total_steps = int(duration / dt)  # 预生成总步数
            self.fs = 1 / dt  # 采样频率 (Hz)
            
            # 预先生成风场序列（u/v/w）
            self.t, self.u_series, self.v_series, self.w_series = self._generate_wind_series()
