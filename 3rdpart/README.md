# 第三方依赖

控制器只保留 QuadControl、OmMpcControl 和 acados NMPC。正常 C++ 构建依赖：

| 依赖 | 用途 |
| --- | --- |
| Eigen 3.4.0 | 矩阵与几何 |
| OSQP 1.0.0 / QDLDL 0.1.8 | OmMpc、轨迹优化 |
| acados 4c23274e4 / HPIPM / BLASFEO | NMPC |
| 系统 SuiteSparse/UMFPACK | 角速度内环 |

`build_all.sh` 只构建 Eigen、OSQP 和 acados，不再构建 Ipopt、NLopt、CasADi C++。
原有第三方源码子模块保留，但已不参与当前控制器构建，也不会卸载本机已有库。
Python CasADi 只在重新生成 acados 模型时需要。

## 首次安装

```bash
sudo apt install build-essential cmake pkg-config libsuitesparse-dev python3-colcon-common-extensions
git submodule update --init 3rdpart/src/eigen 3rdpart/src/qdldl 3rdpart/src/osqp 3rdpart/src/acados
git -C 3rdpart/src/acados submodule update --init external/blasfeo external/hpipm
JOBS=2 ./3rdpart/build_all.sh
```

默认安装到 `/usr/local`，可用 `PREFIX=/opt/manycontroller` 覆盖。安装阶段在必要时
调用 sudo；不在仓库提交二进制。新脚本还安装 acados 的 `link_libs.json` 和
`git_commit_hash`，因为当前上游 CMake 不会把这些生成器需要的元数据安装到前缀。

CMake 查找 `Eigen3`、`osqp`、`acados` 和 UMFPACK；工程自身的 generated C 源码
编译为静态目标 `px4ctrl_acados_ocp_solver`，普通编译不用运行 Python。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl --symlink-install
```

若缓存仍指向历史依赖路径，追加 `--cmake-clean-cache`。

## 修复已安装 OSQP 显示为 0.0.0

如果 OSQP 是用旧版脚本安装的，CMake 可能报告：

```text
Could not find a configuration file for package "osqp" that is compatible
with requested version "1.0".
/usr/local/lib/cmake/osqp/osqp-config.cmake, version: 0.0.0
```

源码并没有错装：`3rdpart/src/osqp` 仍是 `v1.0.0`。这是上游 CMake 版本变量没有
赋值造成的包元数据错误。更新本仓库后重新配置、编译和安装 OSQP 即可：

```bash
cmake -S 3rdpart/src/osqp -B 3rdpart/build/osqp \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DOSQP_VERSION=1.0.0 \
  -DOSQP_BUILD_SHARED_LIB=ON \
  -DOSQP_BUILD_STATIC_LIB=OFF \
  -DOSQP_BUILD_UNITTESTS=OFF \
  -DOSQP_BUILD_DEMO_EXE=OFF \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
  -DFETCHCONTENT_SOURCE_DIR_QDLDL="$PWD/3rdpart/src/qdldl"
cmake --build 3rdpart/build/osqp --parallel 2
sudo cmake --install 3rdpart/build/osqp
sudo ldconfig
```

确认安装元数据和工程查找均正常：

```bash
grep 'set(PACKAGE_VERSION' \
  /usr/local/lib/cmake/osqp/osqp-config-version.cmake | head -1
```

第一条命令应该显示 `1.0.0`。之后清除 px4ctrl 的旧 CMake 缓存并重新构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to px4ctrl --cmake-clean-cache
```

## 重新生成 acados 模型代码

数值参数从 `px4ctrl/config/params.yaml` 启动时读取，修改后重启即可；包括预测步数、
步长、重力、代价权重、输入边界、迭代数、容差和时间预算。只有模型、代价维度、
约束结构或算法类型变化才需生成。

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install 'casadi==3.7.2' ./3rdpart/src/acados/interfaces/acados_template
ACADOS_SOURCE_DIR="$PWD/3rdpart/src/acados" \
ACADOS_INSTALL_PREFIX=/usr/local \
python3 script/generate_px4ctrl_acados_nmpc.py
```

模板渲染器默认在 `$ACADOS_SOURCE_DIR/bin/t_renderer`，也可用 `TERA_PATH` 指定已有
兼容版本。`ACADOS_INSTALL_PREFIX` 必须含 libacados.so 和 lib/link_libs.json；旧安装
缺少元数据时重新运行更新后的 build_all.sh，或指定一个完整、同版本的安装前缀。

脚本只生成源码，不再额外编译 .so；然后正常 colcon build。提交生成 C/H/JSON 的
变化，不提交生成目录中的 Makefile、对象或共享库。接口说明和测试见
[ACADOS_NMPC.md](../src/realflight_modules/px4ctrl/ACADOS_NMPC.md)。
