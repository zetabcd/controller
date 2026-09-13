#ifndef __OTHER_UTILS_H
#define __OTHER_UTILS_H

#include <Eigen/Core>
#include <algorithm>  // 用于 std::min 和 std::max
#include <type_traits> // 用于 std::enable_if, std::is_arithmetic
#include <iostream>
#include <vector>
#include <cmath>      // for std::abs
#include <limits>     // for std::numeric_limits
#include <execinfo.h>
#include <cstdlib>
#include <unistd.h>

// sun: 通用数值工具集中提供单位换算、标量/Eigen 饱和、角度归一化和限幅几何操作；
// sun: 这些函数位于头文件中并以内联或模板形式实现，供两个控制节点共用。

namespace uav_utils {

// 打印调用栈（从当前函数开始，depth 为最大回溯深度）
inline void printCallStack(int depth = 32) {
    std::cout << "\n===== 函数调用栈 =====\n";
    
    // 1. 获取栈帧返回地址列表
    void* callstack[depth];
    int frameCount = backtrace(callstack, depth);
    
    // 2. 将地址转换为符号（函数名+地址）
    char** symbols = backtrace_symbols(callstack, frameCount);
    if (symbols == nullptr) {
        perror("backtrace_symbols failed");
        return;
    }

    // 3. 打印每一层调用（跳过第0层，因为是printCallStack自身）
    for (int i = 1; i < frameCount; ++i) {
        std::cout << "[" << i << "] " << symbols[i] << "\n";
    }

    // 释放内存
    free(symbols);
    std::cout << "======================\n";
}

constexpr double pi = 3.14159265358979323846;
constexpr double rad2deg(double x) {
    return (x / pi * 180.0);
}
constexpr double deg2rad(double x) {
    return (x / 180.0 * pi);
}
constexpr double rad2rpm(double x) {
    return (x / (2*pi) * 60);
}
constexpr double rpm2rad(double x) {
    return (x / 60 * (2*pi));
}

template<typename T> constexpr float sgn(T val) {
    // sun: 返回 -1/0/1，不对零强行指定正号，适合四元数最短路径和方向判断。
    return (T(0) < val) - (val < T(0));
}

// --- 处理 int/double 类型 (第一个参数)，第二个参数也必须是 int/double ---
// 使用 std::enable_if 来限制此模板仅对 int 和 double 生效（SFINAE 规则）
template <
    typename T,
    typename U_MIN,
    typename U_MAX,
    // 约束 T 必须是 int 或 double
    typename Enable_T = std::enable_if_t<std::is_same_v<T, int> || std::is_same_v<T, double>>,
    // 约束 U 必须是 int 或 double
    typename Enable_U_MIN = std::enable_if_t<std::is_same_v<U_MIN, int> || std::is_same_v<U_MIN, double>>,
    typename Enable_U_MAX = std::enable_if_t<std::is_same_v<U_MAX, int> || std::is_same_v<U_MAX, double>>
>
T clip(const T& x, const U_MIN& min_val, const U_MAX& max_val) {
    // sun: 标量版本通过 SFINAE 限定为 int/double，避免与下方 Eigen 重载产生歧义。
    return static_cast<T>(std::max(double(min_val), std::min(double(x), double(max_val))));
}

// --- 处理 Eigen 类型 (第一个参数)，第二个参数必须是 int/double ---
// Eigen::EigenBase<Derived> 是所有 Eigen 数组/矩阵类的基类
template <
    typename Derived,
    typename U_MIN,
    typename U_MAX,
    // 约束 U 必须是 int 或 double
    typename Enable_U_MIN = std::enable_if_t<std::is_same_v<U_MIN, int> || std::is_same_v<U_MIN, double>>,
    typename Enable_U_MAX = std::enable_if_t<std::is_same_v<U_MAX, int> || std::is_same_v<U_MAX, double>>
>
auto clip(const Eigen::EigenBase<Derived>& x, const U_MIN& min_val, const U_MAX& max_val) {
    // sun: Eigen 版本逐元素裁剪并保持原表达式的矩阵/数组形状。
    return x.derived().cwiseMin(max_val).cwiseMax(min_val);
}

/**
 * @brief 三维向量的保方向饱和裁剪函数
 * @param x     输入的待裁剪三维向量
 * @param min   向量模长的下限阈值（饱和下界）
 * @param max   向量模长的上限阈值（饱和上界）
 * @return      裁剪后的三维向量，方向与原向量一致，模长∈[min, max]
 */
inline Eigen::Vector3d clip_gd(const Eigen::Vector3d& vec, const double &min, const double &max)
{
    // sun: 与逐元素 clip 不同，本函数只调整模长，因此不会改变输入向量方向。
    // 1. 计算输入向量的模长(L2范数)
    const double vec_norm = vec.norm();

    // 2. 处理零向量特殊情况：模长趋近于0时，直接返回零向量，避免除以0的非法运算
    const double eps = 1e-9;
    if (vec_norm < eps)
    {
        return Eigen::Vector3d::Zero();
    }

    // 3. 保方向的幅值饱和核心逻辑
    if (vec_norm <= min)
    {
        // 模长小于下限：归一化后放大到min，方向不变
        return vec.normalized() * min;
    }
    else if (vec_norm >= max)
    {
        // 模长大于上限：归一化后缩小到max，方向不变
        return vec.normalized() * max;
    }
    else
    {
        // 模长在区间内：直接返回原向量，无裁剪
        return vec;
    }
}
// 函数：在std::vector<double>向量中找到与 a 差值最小的数
inline double findClosestStd(const std::vector<double>& vec, double a) {
    if (vec.empty()) {
        throw std::invalid_argument("Vector is empty");
        // throw "Vector is empty";
    }

    // 使用 std::min_element 和 lambda 表达式作为比较函数
    auto closest_it = std::min_element(vec.begin(), vec.end(),
        [a](double x, double y) {
            return std::abs(x - a) < std::abs(y - a);
        });

    return *closest_it;
}

inline void printVector(const std::vector<double>& vec) {
    for (double val : vec) {
        std::cout << val << " ";  // 空格分隔元素
    }
    std::cout << std::endl;  // 换行
}

inline Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

inline Eigen::Vector2d constrainXY(const Eigen::Vector2d &v0, const Eigen::Vector2d &v1, const float &max)
{
	if (Eigen::Vector2d(v0 + v1).norm() <= max) {
		// 向量不超过最大幅度
		return v0 + v1;

	} else if (v0.norm() >= max) {
		// v0的大小（优先级较高）已经超过最大值
		return v0.normalized() * max;

	} else if (std::abs(Eigen::Vector2d(v1 - v0).norm()) < 0.001f) {
		// 两向量相等
		return v0.normalized() * max;

	} else if (v0.norm() < 0.001) {
		// v0 = 0
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||v0 + v1|| = ||v0 + s * u1|| <= max
        // 求根公式
		Eigen::Vector2d u1 = v1.normalized();
		double m = u1.dot(v0);
		double c = v0.dot(v0) - max * max;
		double s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

}
#endif
