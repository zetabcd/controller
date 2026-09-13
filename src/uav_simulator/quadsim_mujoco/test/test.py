from DrydenWindField import DrydenWindField

if __name__ == "__main__":
    # 1. 初始化风场模型（匹配无人机动力学步长）
    dt = 0.01  # 无人机动力学计算步长（100Hz）
    wind_model = DrydenWindField(
        U0=5.0,        # 平均风速25m/s
        height=10.0,   # 飞行高度800m
        dt=dt,          # 与动力学步长一致
        duration=1.0,  # 预生成60秒的风场数据
        loop=True       # 循环复用风场
    )
    
    # 2. 模拟无人机动力学循环（逐步获取风场）
    total_sim_steps = 200  # 模拟1000步（10秒）
    u_list, v_list, w_list = [], [], []
    time_list = []
    
    for step in range(total_sim_steps):
        # 动力学模型主循环：获取当前步风场
        u, v, w = wind_model.get_wind()
        
        # （此处可添加无人机动力学计算逻辑）
        # 例如：无人机速度 = 标称速度 + 风场扰动
        
        # 保存数据用于可视化
        u_list.append(u)
        v_list.append(v)
        w_list.append(w)
        time_list.append(step * dt)
    
    # 3. 可视化逐步获取的风场数据
    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(time_list, u_list, 'b-*', linewidth=0.8)
    plt.title('无人机动力学逐步风场 - 纵向 (u)')
    plt.ylabel('速度 (m/s)')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(3, 1, 2)
    plt.plot(time_list, v_list, 'g-*', linewidth=0.8)
    plt.title('无人机动力学逐步风场 - 侧向 (v)')
    plt.ylabel('速度 (m/s)')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(3, 1, 3)
    plt.plot(time_list, w_list, 'r-*', linewidth=0.8)
    plt.title('无人机动力学逐步风场 - 垂向 (w)')
    plt.xlabel('时间 (s)')
    plt.ylabel('速度 (m/s)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 4. 重置风场模型（示例）
    wind_model.reset(new_seed=123)
    # 重置后可重新开始获取风场
    u_reset, v_reset, w_reset = wind_model.get_wind()
    print(f"重置后第一步风场：u={u_reset:.4f}, v={v_reset:.4f}, w={w_reset:.4f}")