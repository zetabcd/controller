#include <cstdint>
#include <cmath>
#include <cstdint>
#include <Eigen/Dense>
#include <iostream>

// sun: 本文件提供可直接在控制循环逐样本调用的 IIR 滤波器。构造参数 fc/fs 均为 Hz，
// sun: reset() 会清空延迟状态，模式切换或传感器重连后应调用以避免旧历史污染输出。

// 二阶巴特沃斯低通滤波器
class SecondOrderButterworthLPF {
public:
    /**
     * @brief 构造函数：初始化滤波器参数
     * @param fc 截止频率 (Hz)
     * @param fs 采样频率 (Hz)
     */
    SecondOrderButterworthLPF(double fc, double fs):fs_(fs) {
        // sun: 先对模拟截止频率做预畸变，再用双线性变换得到数字二阶 Butterworth 系数。
        double wc = 2 * M_PI * fc;
        double T = 1.0 / fs;
        // 预畸变校正
        double wc_prime = (2.0 / T) * tan(wc * T / 2.0);
        // 计算滤波器系数
        double a0_coeff = wc_prime * wc_prime * T * T;
        double a1_coeff = 2.0 * a0_coeff;
        double denom = 4.0 + 2.0 * sqrt(2.0) * wc_prime * T + a0_coeff;
        
        a0 = a0_coeff / denom;
        a1 = a1_coeff / denom;
        a2 = a0_coeff / denom;
        b1 = (2.0 * a0_coeff - 8.0) / denom;
        b2 = (4.0 - 2.0 * sqrt(2.0) * wc_prime * T + a0_coeff) / denom;

        // 初始化历史数据
        reset();
    }

    /**
     * @brief 滤波单次输入
     * @param x 当前输入值
     * @return 滤波后输出值
     */
    double filter(double x) {
        // sun: 直接型差分方程使用两阶输入/输出历史，每次调用后推进一个采样点。
        double y = a0 * x + a1 * x_1 + a2 * x_2 - b1 * y_1 - b2 * y_2;
        // 更新历史数据
        x_2 = x_1;
        x_1 = x;
        y_2 = y_1;
        y_1 = y;
        return y;
    }

    /**
     * @brief 重置滤波器状态
     */
    void reset() {
        x_1 = x_2 = 0.0;
        y_1 = y_2 = 0.0;
    }

    void set_frequency(double new_fc){
        // sun: 在线修改截止频率只重算系数并保留历史，可避免输出瞬间归零但会产生短暂过渡。
        double wc = 2 * M_PI * new_fc;
        double T = 1.0 / fs_;
        // 预畸变校正
        double wc_prime = (2.0 / T) * tan(wc * T / 2.0);
        // 计算滤波器系数
        double a0_coeff = wc_prime * wc_prime * T * T;
        double a1_coeff = 2.0 * a0_coeff;
        double denom = 4.0 + 2.0 * sqrt(2.0) * wc_prime * T + a0_coeff;
        
        a0 = a0_coeff / denom;
        a1 = a1_coeff / denom;
        a2 = a0_coeff / denom;
        b1 = (2.0 * a0_coeff - 8.0) / denom;
        b2 = (4.0 - 2.0 * sqrt(2.0) * wc_prime * T + a0_coeff) / denom;
    }

private:
    // sun: a* 为前馈系数，b* 为反馈系数；x_*、y_* 分别保存两个历史采样。
    double a0, a1, a2;
    double b1, b2;
    // 历史输入输出
    double x_1, x_2;
    double y_1, y_2;
    double fs_;
};

// 二阶巴特沃斯高通滤波器
class SecondOrderButterworthHPF {
public:
    // sun: 高通主要用于去除慢变偏置或重力分量，截止频率必须显著低于奈奎斯特频率。
    /**
     * @brief 构造函数：初始化高通滤波器参数
     * @param fc 截止频率 (Hz)，飞控IMU去零漂建议0.1~1Hz
     * @param fs 采样频率 (Hz)，需与传感器输出频率一致
     */
    SecondOrderButterworthHPF(double fc, double fs) {
        double wc = 2 * M_PI * fc;
        double T = 1.0 / fs;
        // 预畸变校正，消除双线性变换的频率畸变
        double wc_prime = (2.0 / T) * tan(wc * T / 2.0);
        double wc_sq = wc_prime * wc_prime;
        double sqrt2_wc = sqrt(2.0) * wc_prime;

        // 计算滤波器系数
        double denom = 4.0 + sqrt2_wc * T + wc_sq * T * T;
        a0 = 4.0 / denom;
        a1 = -8.0 / denom;
        a2 = 4.0 / denom;
        b1 = (2.0 * wc_sq * T * T - 8.0) / denom;
        b2 = (4.0 - sqrt2_wc * T + wc_sq * T * T) / denom;

        // 初始化历史数据，避免初始值突变
        reset();
    }

    /**
     * @brief 单次滤波计算
     * @param x 当前输入值（如IMU角速度、加速度原始数据）
     * @return 滤波后输出值
     */
    double filter(double x) {
        double y = a0 * x + a1 * x_1 + a2 * x_2 - b1 * y_1 - b2 * y_2;
        // 更新历史输入输出（延迟寄存器）
        x_2 = x_1;
        x_1 = x;
        y_2 = y_1;
        y_1 = y;
        return y;
    }

    /**
     * @brief 重置滤波器状态，适用于传感器重启或数据断连后恢复
     */
    void reset() {
        x_1 = x_2 = 0.0;
        y_1 = y_2 = 0.0;
    }

private:
    // 滤波器系数
    double a0, a1, a2;
    double b1, b2;
    // 历史输入输出缓存（仅需2级延迟，内存占用极小）
    double x_1, x_2;
    double y_1, y_2;
};

class SecondOrderButterworthBPF {
public:
    // sun: 带通由中心频率和带宽定义，适合隔离电机/机架某一频段振动。
    /**
     * @brief 构造函数：初始化带通滤波器参数
     * @param fc_l 低通截止频率 (Hz)
     * @param fc_h 高通截止频率 (Hz)
     * @param fs 采样频率 (Hz)
     */
    SecondOrderButterworthBPF(const double &fc_l, const double &fc_h, const double &fs) 
    : lpf_(fc_l, fs), hpf_(fc_h, fs) {
        reset();
    }

    /**
     * @brief 单次滤波计算
     * @param x 当前输入值（如IMU角速度、加速度原始数据）
     * @return 滤波后输出值
     */
    double filter(const double &x) {
        double y = hpf_.filter(lpf_.filter(x));
        return y;
    }

    /**
     * @brief 重置滤波器状态，适用于传感器重启或数据断连后恢复
     */
    void reset() {
        lpf_.reset();
        hpf_.reset();
    }

private:
    SecondOrderButterworthLPF lpf_;
    SecondOrderButterworthHPF hpf_;
};

// SO3滤波
constexpr double SO3_EPS = 1e-8;
// -------------------------- SO(3) 核心映射函数 --------------------------
/**
 * @brief SO(3)对数映射: 流形→切空间
 * @param q 单位四元数 (Eigen::Quaternionf)，表示SO(3)上的相对旋转
 * @return Vector3f 局部切空间的三维角速度扰动向量 ω ∈ R³ (核心：四元数差异转切空间向量)
 */
inline Eigen::Vector3d SO3_log(const Eigen::Quaterniond& q)
{
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();

    // 计算旋转角 θ，clamp防止浮点精度导致qw超出[-1,1]范围
    double theta = 2.0 * acos( std::clamp(qw, -1.0, 1.0) );
    double sin_half_theta = sqrt(1.0 - qw*qw);

    Eigen::Vector3d omega;
    // 小角度近似：避免sin(θ/2)趋近0时的除零错误，提升数值稳定性
    if (sin_half_theta < SO3_EPS)
    {
        omega = 2.0 * Eigen::Vector3d(qx, qy, qz);
    }
    else
    {
        double scale = theta / sin_half_theta;
        omega = scale * Eigen::Vector3d(qx, qy, qz);
    }
    return omega;
}
/**
 * @brief SO(3)指数映射: 切空间→流形
 * @param omega 切空间的三维角速度向量 (Eigen::Vector3f)，滤波后的扰动向量
 * @return Quaternionf 单位四元数，映射回SO(3)流形的旋转表示
 */
inline Eigen::Quaterniond SO3_exp(const Eigen::Vector3d& omega)
{
    // 计算旋转角的模长
    double theta = omega.norm();
    double half_theta = theta * 0.5;
    double sin_half_theta = sin(half_theta);
    double cos_half_theta = cos(half_theta);

    Eigen::Quaterniond q;
    // 小角度近似：避免θ趋近0时的除零错误
    if (theta < SO3_EPS)
    {
        q.w() = 1.0;
        q.vec() = 0.5 * omega; // vec()直接访问x/y/z虚部组成的Vector3f
    }
    else
    {
        double scale = sin_half_theta / theta;
        q.w() = cos_half_theta;
        q.vec() = scale * omega;
    }
    q.normalize(); // 强制单位化，消除浮点误差
    return q;
}


// -------------------------- 一阶低通滤波器类 (Eigen::Quaternion 核心版) --------------------------
class SO3FirstOrderLowPassFilter
{
    // sun: SO(3) 滤波不直接平均四元数分量，而在李代数切空间更新，保持旋转矩阵正交性。
private:
    float alpha;                // 滤波系数 0<α<1，越小越平滑，越大响应越快
    Eigen::Quaterniond q_filt;         // 滤波器内部缓存的滤波状态四元数，实时更新
    bool is_initialized;        // 初始化标志位：首次输入直接赋值，无滤波运算

public:
    /**
     * @brief 构造函数
     * @param filter_alpha 滤波系数，必须满足 0 < alpha < 1
     */
    SO3FirstOrderLowPassFilter(float filter_alpha)
        : alpha(filter_alpha), is_initialized(false)
    {
        // 滤波系数合法性校验，非法则自动修正为工程经验值0.2
        if (alpha <= 0.0 || alpha >= 1.0)
        {
            std::cerr << "[Warning] 滤波系数错误! 必须0<α<1，已自动修正为0.2" << std::endl;
            alpha = 0.2;
        }
        // 初始化为单位四元数（无旋转姿态）
        q_filt = Eigen::Quaterniond::Identity();
    }

    /**
     * @brief 核心滤波接口：实时更新的核心函数
     * @param q_new 输入的新四元数 (Eigen::Quaternionf，建议单位化)
     * @return Quaternionf 滤波后的单位四元数，内部状态同步更新
     */
    Eigen::Quaterniond filter(const Eigen::Quaterniond& q_new)
    {
        // 第一步：首次初始化，直接将输入作为滤波初始值，无滤波
        if (!is_initialized)
        {
            q_filt = q_new.normalized(); // 强制单位化，兼容非单位输入
            is_initialized = true;
            return q_filt;
        }

        // 第二步：计算 当前滤波姿态 → 新输入姿态 的相对旋转四元数 (表征旋转差异)
        Eigen::Quaterniond delta_q = q_filt.inverse() * q_new;
        delta_q.normalize();

        // 第三步：SO(3)对数映射 → 把四元数差异 转换为 局部切空间的三维向量
        Eigen::Vector3d omega = SO3_log(delta_q);

        // 第四步：一阶低通滤波 → 切空间内的线性加权，乘滤波系数
        Eigen::Vector3d omega_filt = alpha * omega;

        // 第五步：SO(3)指数映射 → 滤波后的切空间向量 映射回SO(3)流形，得到滤波后的相对旋转
        Eigen::Quaterniond delta_q_filt = SO3_exp(omega_filt);

        // 第六步：更新滤波器的当前姿态，回流形复合旋转
        q_filt = q_filt * delta_q_filt;
        q_filt.normalize(); // 强制单位化，消除浮点运算的模长漂移

        return q_filt;
    }

    /**
     * @brief 重置滤波器状态
     * @note 适用于姿态跳变、系统重启、滤波异常时调用
     */
    void reset()
    {
        is_initialized = false;
        q_filt = Eigen::Quaterniond::Identity();
    }

    /**
     * @brief 获取滤波器当前的滤波状态
     */
    Eigen::Quaterniond getCurrentState() const
    {
        return q_filt;
    }

    /**
     * @brief 修改滤波系数
     */
    void setAlpha(float new_alpha)
    {
        if(new_alpha > 0.0 && new_alpha < 1.0)
            alpha = new_alpha;
    }
};
