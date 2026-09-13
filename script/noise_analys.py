import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import pandas as pd
import math
from scipy.optimize import minimize, differential_evolution, basinhopping
from scipy.sparse.linalg import spsolve, cg
from scipy.sparse import diags, eye
from scipy.interpolate import interp1d
from sklearn import metrics  # 官方库计算指标
import scipy.sparse as sp  # 稀疏矩阵核心库
import os
import sys
from matplotlib import font_manager
from matplotlib import rcParams
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    
def estimate_gaussian_params(sensor_data):
    """
    估计传感器数据的高斯分布参数（均值+方差）
    :param sensor_data: 1D 传感器数据列表/数组
    :return: mean: 标准值, var: 无偏方差
    """
    # 转换为numpy数组
    data = np.array(sensor_data, dtype=np.float64)
    # 第一步：计算初始均值和标准差，剔除异常值（3σ准则）
    mean_init = np.mean(data)
    std_init = np.std(data, ddof=1)  # ddof=1 计算无偏标准差
    # 筛选有效数据（剔除异常值）
    valid_data = data[np.abs(data - mean_init) <= 3 * std_init]
    # 第二步：基于有效数据计算最终参数
    mean_final = np.mean(valid_data)
    var_final = np.var(valid_data-mean_final, ddof=1)  # 无偏方差
    return mean_final, var_final

def draw_noise_data(data_cols, xlabel, ylabels):
    
    df = pd.read_csv('./script/rosbag2_2024_11_06-16_47_52.csv')
    filtered_df = df[df['__time'] >= df['__time'][0] + 25]
    filtered_df = df
    
    data_df = filtered_df[data_cols]
    data_df_cleaned = data_df.dropna()
    t = data_df_cleaned.to_numpy()[:,0]
    t = t - t[0]
    
    select = t<(37-25)
    t = t[select]
    data_x = data_df_cleaned.to_numpy()[:,1]
    data_y = data_df_cleaned.to_numpy()[:,2]
    data_z = data_df_cleaned.to_numpy()[:,3]
    data_x = data_x[select]
    data_y = data_y[select]
    data_z = data_z[select]
    mean_x, var_x = estimate_gaussian_params(data_x)
    mean_y, var_y = estimate_gaussian_params(data_y)
    mean_z, var_z = estimate_gaussian_params(data_z)
    print("data_x:",f"N({mean_x:.5f},{var_x:.5f}),std={np.sqrt(var_x):.5f}")
    print("data_y:",f"N({mean_y:.5f},{var_y:.5f}),std={np.sqrt(var_y):.5f}")
    print("data_z:",f"N({mean_z:.5f},{var_z:.5f}),std={np.sqrt(var_z):.5f}")
    
    fig, axs = plt.subplots(3,1,figsize=(10, 15))
    datas = (data_x,data_y,data_z)
    colors = ("r","b","g")
    
    for i in range(3):
        ax = axs[i]
        ax.axhline(0, color='grey', linestyle='--', linewidth=1)
        ax.plot(t,datas[i], color=colors[i],linewidth=.5)
        if i == 2:
            xlabel_ = xlabel
        else:
            xlabel_ = None
        ax.set_xlabel(xlabel_)
        ax.set_ylabel(ylabels[i])
    
    

if __name__ == "__main__":
    gyro_cols = ["__time","/fmu/out/sensor_combined/gyro_rad.0", "/fmu/out/sensor_combined/gyro_rad.1", "/fmu/out/sensor_combined/gyro_rad.2"]
    y_label_chinese = r"角速度"
    y_label_unit = r"$\ (rad/s)$"
    y_labels = (
        y_label_chinese+r"$^\mathcal{B}\omega_x$"+y_label_unit,
        y_label_chinese+r"$^\mathcal{B}\omega_y$"+y_label_unit,
        y_label_chinese+r"$^\mathcal{B}\omega_z$"+y_label_unit)
    xlabel = r"时间$\ t(s)$"
    draw_noise_data(gyro_cols, xlabel, y_labels);
    
    acc_cols = ["__time","/fmu/out/sensor_combined/accelerometer_m_s2.0", "/fmu/out/sensor_combined/accelerometer_m_s2.1", "/fmu/out/sensor_combined/accelerometer_m_s2.2"]
    y_label_chinese = r"加速度"
    y_label_unit = r"$\ (m/s^2)$"
    y_labels = (
        y_label_chinese+r"$^\mathcal{I}a_x$"+y_label_unit,
        y_label_chinese+r"$^\mathcal{I}a_y$"+y_label_unit,
        y_label_chinese+r"$^\mathcal{I}a_z$"+y_label_unit)
    xlabel = r"时间$\ t(s)$"
    draw_noise_data(acc_cols, xlabel, y_labels);
    
    plt.show()
    
    
    
    
    

    