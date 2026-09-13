#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <functional>
#include <deque>
#include <chrono> // 高精度计时库
#include <suitesparse/umfpack.h>

// sun: 本文件实现滑动窗口总变差正则化（TVR）数值微分。它在噪声角速度上估计
// sun: 角加速度，并用多项式外推补偿窗口方法在最新样本处的边缘效应。
// 静态变量存储开始时间点（单线程安全，多线程需加锁）
static std::chrono::high_resolution_clock::time_point tic_start;

// 开始计时
inline void tic() {
    tic_start = std::chrono::high_resolution_clock::now();
}
// 结束计时，返回耗时（单位：秒，默认打印结果）
inline double toc(bool print = true) {
    auto tic_end = std::chrono::high_resolution_clock::now();
    // 计算耗时（秒）：duration_cast 转换为微秒后转double，避免精度丢失
    double duration = std::chrono::duration_cast<std::chrono::microseconds>(tic_end - tic_start).count() / 1e6;
    
    if (print) {
        std::cout << "耗时：" << duration << " 秒（" << duration * 1e3 << " 毫秒）" << std::endl;
    }
    return duration;
}

class BoundedDeque 
{
// sun: 固定容量双端队列自动丢弃最旧样本，保证求解规模和实时计算开销有上界。
private:
    std::deque<double> deque_;   // 底层 deque
    size_t max_size_;            // 最大长度

    // 内部截断函数：保证 deque 长度 ≤ max_size_
    void truncate() {
        if (max_size_ == 0) {    // 最大长度为0，清空
            deque_.clear();
            return;
        }
        // 超过最大长度时，删除头部元素（FIFO）
        while (deque_.size() > max_size_) {
            deque_.pop_front();
        }
    }

public:
    // 构造函数：指定最大长度
    explicit BoundedDeque(size_t max_size) : max_size_(max_size) {
        if (max_size == 0) {
            std::cout << "警告：最大长度设为0，deque 将始终为空\n";
        }
    }

    // ========== 核心添加方法 ==========
    // 尾部添加元素（超长度则删头部）
    void push_back(double val) {
        deque_.push_back(val);
        truncate();
    }

    // 头部添加元素（超长度则删尾部）
    void push_front(double val) {
        deque_.push_front(val);
        // 头部添加时超长度，删尾部
        while (deque_.size() > max_size_) {
            deque_.pop_back();
        }
    }

    // 原地构造元素（等价 push_back）
    template <typename... Args>
    void emplace_back(Args&&... args) {
        deque_.emplace_back(std::forward<Args>(args)...);
        truncate();
    }

    // ========== 其他常用接口 ==========
    // 获取当前长度
    size_t size() const { return deque_.size(); }

    // 获取最大长度
    size_t max_size() const { return max_size_; }

    // 修改最大长度（修改后自动截断）
    void set_max_size(size_t new_max) {
        max_size_ = new_max;
        truncate();
    }

    // 访问元素（只读）
    const double& operator[](size_t idx) const {
        if (idx >= deque_.size()) {
            throw std::out_of_range("索引越界");
        }
        return deque_[idx];
    }

    // 访问元素（可写）
    double& operator[](size_t idx) {
        if (idx >= deque_.size()) {
            throw std::out_of_range("索引越界");
        }
        return deque_[idx];
    }

    // 清空 deque
    void clear() { deque_.clear(); }

    // 遍历用迭代器（只读）
    auto begin() const { return deque_.cbegin(); }
    auto end() const { return deque_.cend(); }
};

// 垂直拼接：多个列向量 → 拼接后的列向量
template <typename... Vectors>
Eigen::VectorXd concat_vertical(const Vectors&... vecs) {
    // sun: 一次分配目标向量后按段复制，供优化问题拼接多项代价或约束使用。
    // 计算总长度，一次性分配内存
    int total_size = 0;
    (void)std::initializer_list<int>{ (total_size += vecs.size(), 0)... }; // 折叠求和
    Eigen::VectorXd result(total_size);

    // 逐个拷贝输入向量到结果的对应位置
    int current_pos = 0;
    (void)std::initializer_list<int>{ 
        [&]() {
            result.segment(current_pos, vecs.size()) = vecs; // 分段赋值
            current_pos += vecs.size();
            return 0;
        }()... 
    };

    return result;
}

std::function<double(double)> polyFun(const Eigen::VectorXd& coeffs)
{
    return [coeffs](const double &x) -> double {
        int order = coeffs.size() - 1;
        Eigen::ArrayXd powers_arr = Eigen::ArrayXd::LinSpaced(order + 1, 0, order); // 生成幂次数组 [0, 1, 2, ..., n]
        Eigen::ArrayXd x_pows_arr = Eigen::pow(x, powers_arr); // 得到目标数列 (1, x, x², ..., xⁿ)
        return (x_pows_arr * coeffs.array()).sum(); // 点乘并求和
    };
}

/**
 * 加权最小二乘多项式拟合（对应 Python wls_expoly）
 * @param x 输入x向量（std::vector）
 * @param y 输入y向量（std::vector）
 * @param order 多项式阶数（默认5）
 * @param weight_scale 权重缩放系数（默认5）
 * @return 拟合多项式函数
 */
std::function<double(double)> wls_polyfit(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& y, 
    int order = 5, 
    double weight_scale = 5.0) 
{
    // sun: 加权最小二乘拟合窗口尾部多项式，越靠近当前时刻的样本权重越高。
    // ===================== 1. 输入校验 =====================
    int n = x.size();
    if (n != y.size()) {
        throw std::invalid_argument("x和y长度不一致！");
    }
    if (n < order + 1) {
        throw std::invalid_argument("x长度必须 ≥ 多项式阶数+1！");
    }
    if (order < 1) {
        throw std::invalid_argument("多项式阶数必须 ≥ 1！");
    }

    // ===================== 2. 生成Vander矩阵 X =====================
    Eigen::MatrixXd X(n, order + 1);
    // 生成原始 Vander 矩阵：每行 1, x_i, x_i², ..., x_i^order
    for (int i = 0; i < order + 1; ++i) {
        X.col(i) = x.array().pow(i).matrix();
    }
    // ===================== 3. 构造权重矩阵 W =====================
    // 3.1 生成线性权重（对应 np.linspace(1, n, n)）
    Eigen::VectorXd weights_linear = Eigen::VectorXd::LinSpaced(n,1.0,static_cast<double>(n));
    // 3.2 计算权重：(x/n)^weight_scale（对应 weights_fun）
    Eigen::VectorXd weights = (weights_linear.array()/static_cast<double>(n)).pow(weight_scale);
    // 3.3 构造对角权重矩阵 W（对应 np.diag(weights)）
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(n, n);
    W.diagonal() = weights;

    // ===================== 4. 加权最小二乘求解系数 =====================
    // 公式：coefficients = (X^T W X)^-1 X^T W y
    Eigen::MatrixXd X_T = X.transpose();
    Eigen::MatrixXd XTWX = X_T * W * X;
    Eigen::VectorXd XTWy = X_T * W * y;

    // 求解：XTWX * coefficients = XTWy（优先用LDLT，正定矩阵更稳定）
    
    if (XTWX.determinant() < 1e-10) { // 检查矩阵是否奇异
        throw std::runtime_error("XTWX矩阵奇异，无法求逆！");
    }
    // 方法1：LDLT分解（推荐，正定矩阵）
    Eigen::VectorXd coefficients_wls = XTWX.ldlt().solve(XTWy);
    // 方法2：直接求逆（小矩阵可用）
    // coefficients_wls = XTWX.inverse() * XTWy;

    // ===================== 5. 构造拟合多项式函数（替代 np.poly1d） =====================
    // coefficients_wls 顺序：[a0, a1, a2, ..., aorder] → 对应 a0 + a1*x + a2*x² + ... + aorder*x^order
    // 反转系数（对应 np.poly1d(coefficients_wls[::-1])）：[aorder, ..., a1, a0]
    std::vector<double> coeffs_reversed(coefficients_wls.data(), coefficients_wls.data() + coefficients_wls.size());
    std::reverse(coeffs_reversed.begin(), coeffs_reversed.end());
    auto fitted_polynomial_wls_fun = polyFun(coefficients_wls);

    // ===================== 6. 计算导数系数及最后一个点的导数值 =====================
    // 导数系数：[a1, 2*a2, 3*a3, ..., order*aorder]
    // Eigen::ArrayXd k_arr = Eigen::ArrayXd::LinSpaced(order, 1, order);
    // Eigen::VectorXd derivative_coefficients_wls = (coefficients_wls.tail(order).array() * k_arr).matrix();
    // auto fitted_polynomial_derivative_wls_fun = polyFun(derivative_coefficients_wls);

    // // 计算最后一个x点的导数值
    // double x_nth_point = x.tail(1)[0];
    // double derivative_value_at_nth_point_wls = fitted_polynomial_derivative_wls_fun(x_nth_point);

    // ===================== 返回结果 =====================
    return fitted_polynomial_wls_fun;
}

class SlidingWindowTVDerivative 
{
    // sun: update() 每次加入一个 (时间, 信号) 样本；窗口未满时返回安全默认值，
    // sun: 窗口满后求 TV 正则导数并返回指定的延迟补偿位置。
public:
    struct swTVR_params_t
    {
        int window_size;
        double lambda_tv;
        int expend_n;
        int n_for_expoly;
        double atten;
        int order;
        double weight_scale;
    };
    /**
     * 构造函数：初始化滑动窗口TV正则化导数计算器
     * @param window_size 滑动窗口大小
     * @param lambda_tv TV正则化参数
     */
    SlidingWindowTVDerivative(swTVR_params_t params)
    : params_(params),x_window_(params.window_size), y_window_(params.window_size)
    {
        if (params.n_for_expoly > params.window_size) {
            throw std::invalid_argument("n_for_expoly不能大于window_size");
        }
    }

    /**
     * 更新并计算TV正则化导数（对应Python的update_scipy）
     * @param x_current 当前x值
     * @param y_current 当前y值
     * @param dy_base 基准导数（Python中未使用）
     * @param expend_n 扩展点数量
     * @param n_for_expoly 多项式拟合点数
     * @param atten 指数衰减系数
     * @param order 多项式阶数
     * @param weight_scale 加权尺度
     * @return 估计的导数
     */
    double update(const double &x_current, const double &y_current) {
        // sun: 时间与观测必须成对推进，平均时间间隔 h 用于把离散差分缩放回物理单位。
        // std::cout << "INTO UPDATE" << std::endl;

        // 1. 更新滑动窗口
        x_window_.push_back(x_current);
        y_window_.push_back(y_current);

        // 2. 窗口数据不足时返回0
        int cur_n = static_cast<int>(x_window_.size());
        if (cur_n < params_.window_size) {
            return 0.0;
        }

        // 转换为Eigen
        std::vector<double> x_vec(x_window_.begin(), x_window_.end());
        std::vector<double> y_vec(y_window_.begin(), y_window_.end());
        Eigen::VectorXd x_eig = Eigen::Map<Eigen::VectorXd>(x_vec.data(), x_vec.size());
        Eigen::VectorXd y_eig = Eigen::Map<Eigen::VectorXd>(y_vec.data(), y_vec.size());

        // 3. 计算采样步长h（x窗口差分的均值）
        double h = vectorDiff(x_eig).mean();
        if (h < 1e-10) {  // 避免除零
            h = 1e-10;
        }

        // 4. 截取最后n_for_expoly个点用于多项式拟合
        Eigen::VectorXd x_for_expoly = x_eig.tail(params_.n_for_expoly);
        Eigen::VectorXd y_for_expoly = y_eig.tail(params_.n_for_expoly);

        // 5. 加权最小二乘多项式拟合（需用户实现wls_expoly）
        auto fun_polyfit = wls_polyfit(x_for_expoly, y_for_expoly, params_.order, params_.weight_scale);

        // 7. 构造右扩展段（x_smooth_right, y_smooth_right）
        Eigen::VectorXd x_smooth_right = Eigen::VectorXd::LinSpaced(params_.expend_n, x_eig.tail(1)[0], x_eig.tail(1)[0] + (params_.expend_n - 1) * h);
        Eigen::VectorXd y_smooth_right = (
            x_smooth_right.unaryExpr(fun_polyfit).array() * 
            (-((x_smooth_right.array() - x_smooth_right[0])*params_.atten).square()).exp()
        ).matrix();

        // 8. 拼接完整的x和y向量
        Eigen::VectorXd x_full = concat_vertical(
            x_eig.head(cur_n-1),
            x_smooth_right
        );
        Eigen::VectorXd y_full = concat_vertical(
            y_eig.head(cur_n-1),
            y_smooth_right
        );
        int n = x_full.size();
        Eigen::VectorXd u = tv_regularized_derivative_sparse(x_full, y_full);
        // 12. 返回指定位置的导数（u[n - x_smooth_right.size()]）
        int target_idx = n - x_smooth_right.size();
        if (target_idx < 0 || target_idx >= u.size()) {
            throw std::out_of_range("target_idx超出有效范围");
        }

        return u(target_idx);

    }

    // 清空窗口
    void clear() {
        x_window_.clear();
        y_window_.clear();
    }

    Eigen::VectorXd tv_regularized_derivative_sparse(const Eigen::VectorXd &x, const Eigen::VectorXd &y)
    {
        // sun: 正规方程形成三对角稀疏系统，使用 UMFPACK 的 symbolic/numeric/solve 三阶段求解。
        // tic();
        int n = static_cast<int>(y.size());
        // 3. 计算采样步长h（x窗口差分的均值）
        Eigen::VectorXd h = Eigen::VectorXd::Zero(n);
        h.head(n-1) = vectorDiff(x);
        h(n-1) = h(n-2);
        // 9. 计算目标向量b = D(y)（数值微分）
        Eigen::VectorXd D_y = Eigen::VectorXd::Zero(n);
        // D_y.head(n-1) = vectorDiff(y).cwiseQuotient(h.head(n-1));
        D_y.head(n-1) = vectorDiff(y);
        D_y(n-1) = D_y(n-2);
        Eigen::VectorXd b = D_y;

        int nnz = 3 * n - 2;
        std::vector<int> Ap(n + 1, 0), Ai(nnz, 0);    // CSC结构（固定）
        std::vector<double> Ax(nnz, 0.0);     // CSC数值（动态更新）
        void* Symbolic = nullptr;             // 符号分解结果（复用）
        void* Numeric = nullptr;              // 数值分解结果（临时）
        double *control = nullptr;  // 控制参数（NULL使用默认配置）
        double info[UMFPACK_INFO];  // 信息参数（存储分解/求解的统计信息）
        // 填充列指针Ap,Ai
        int idx = 0;
        for (int j = 0; j < n; ++j) {
            Ap[j] = idx;
            if (j > 0) Ai[idx++] = j - 1;   // 次对角线元素（j > 0时，j列有j-1行）
            Ai[idx++] = j;  // 主对角线元素
            if (j < n-1) Ai[idx++] = j + 1; // 次对角线元素（j < n-1时，j列有j+1行）
        }
        Ap[n] = idx;
        
        // 填充Ax：A = diag(h) + lambda_tv * A_tv
        // A_tv结构：主对角线（1/2） + 次对角线（-1），乘以lambda_tv/h后加单位矩阵
        /*
            [ 1. -1.  0.  0.  0.]
            [-1.  2. -1.  0.  0.]
            [ 0. -1.  2. -1.  0.]
            [ 0.  0. -1.  2. -1.]
            [ 0.  0.  0. -1.  1.]
        */
        // const double eps = 1e-10;
        idx = 0;
        for (int i = 0; i < n; ++i) {
            if (idx >= nnz) {
                throw std::runtime_error("CSC矩阵填充越界");
            }
            // double inv_h = (fabs(h[i]) < eps) ? 1e10 : (1.0 / h[i]);
            // 主对角线：I(i,i) + lambda_tv/h[i] * A_tv(i,i)
            double tv_main = (i == 0 || i == n-1) ? 1.0 : 2.0;
            if (i > 0) Ax[idx++] = params_.lambda_tv * (-1.0);// 次对角线元素
            Ax[idx++] = h[i] + params_.lambda_tv * tv_main; 
            if (i < n-1) Ax[idx++] = params_.lambda_tv * (-1.0);// 次对角线元素
        }
        // ===================== 3. 符号分解 =====================
        int status = umfpack_di_symbolic(
            n,          // 矩阵行数
            n,          // 矩阵列数
            Ap.data(),  // 列指针数组
            Ai.data(),  // 行索引数组
            Ax.data(),  // 数值数组
            &Symbolic,  // 输出：符号分解结果
            control,    // 控制参数
            info        // 输出：信息
        );
        if (status != UMFPACK_OK) {
            std::cerr << "符号分解失败，错误码：" << status << std::endl;
            umfpack_di_report_info(control, info);
            // 若符号分解失败，Symbolic可能未初始化，无需释放
            return Eigen::VectorXd::Zero(n); // 返回空结果
        }
        // ===================== 4. 数值分解 =====================
        status = umfpack_di_numeric(
            Ap.data(),  // 列指针数组
            Ai.data(),  // 行索引数组
            Ax.data(),  // 数值数组
            Symbolic,   // 符号分解结果
            &Numeric,   // 输出：数值分解结果
            control,    // 控制参数
            info        // 输出：信息
        );
        // 符号分解完成后可释放内存
        if (Symbolic != nullptr) {
            umfpack_di_free_symbolic(&Symbolic); // 仅释放已成功初始化的Symbolic
        }
        if (status != UMFPACK_OK) {
            std::cerr << "数值分解失败，错误码：" << status << std::endl;
            umfpack_di_report_info(control, info);
            return Eigen::VectorXd::Zero(n);
        }
        // ===================== 5. 求解 A*u = b =====================
        // 解向量u（初始化为0）
        std::vector<double> u(n, 0.0);
        // UMFPACK_A：求解 A*u = b；其他选项：UMFPACK_At（A^T*u=b）、UMFPACK_Aat（A^H*u=b）
        status = umfpack_di_solve(
            UMFPACK_A,  // 求解类型
            Ap.data(),  // 列指针数组
            Ai.data(),  // 行索引数组
            Ax.data(),  // 数值数组
            u.data(),   // 输出：解向量u
            b.data(),   // 输入：右端向量b
            Numeric,    // 数值分解结果
            control,    // 控制参数
            info        // 输出：信息
        );
        // 数值分解完成后可释放内存
        // 确保释放前指针非空
        // umfpack_di_free_numeric(&Numeric);
        if (Numeric != nullptr) {
            // std::cout << "Numeric" << Numeric << std::endl;
            umfpack_di_free_numeric(&Numeric);
        }
        if (status != UMFPACK_OK) {
            std::cerr << "求解失败，错误码：" << status << std::endl;
            umfpack_di_report_info(control, info);
        }
        return Eigen::Map<Eigen::VectorXd>(u.data(), u.size());
    }
private:
    swTVR_params_t params_;
    BoundedDeque x_window_;
    BoundedDeque y_window_;

    /**
    * @brief 计算Eigen向量的差分（相邻元素的差）
    * @tparam Derived Eigen向量的派生类型（支持行/列向量、任意标量类型）
    * @param vec 输入向量（行/列向量均可）
    * @return 差分结果（长度 = 输入长度 - 1）
    */
    Eigen::VectorXd vectorDiff(const Eigen::VectorXd& x) const{
        int n = x.size();
        // 边界处理：向量长度≤1时，返回空向量
        if (n <= 1) {
            return Eigen::VectorXd(0);
        }
        // 核心计算：后n-1个元素 - 前n-1个元素
        return x.tail(n - 1) - x.head(n - 1);
    }

    // 辅助函数：计算deque的差分均值（对应np.mean(np.diff)）
    double computeMeanDiff(const std::deque<double>& vec) const {
        if (vec.size() < 2) return 0.0;
        double sum = 0.0;
        for (size_t i = 1; i < vec.size(); ++i) {
            sum += vec[i] - vec[i-1];
        }
        return sum / (vec.size() - 1);
    }

};
