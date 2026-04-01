# 多 AGV 仓储仿真项目

本项目实现了一个基于 MATLAB 的多 AGV 仓储搬运仿真系统，覆盖地图建模、任务建模、A* 全局路径规划、时间窗冲突检测、调度分配、局部避障、主循环仿真与场景测试。

## 项目目标

- 在离散栅格仓储地图上模拟多台 AGV 执行搬运任务。
- 通过任务调度、路径规划和时间窗管理避免冲突。
- 在出现动态障碍时触发局部避障，并在必要时进行重规划。
- 通过自动化测试验证单模块行为与完整场景行为。

## 主要能力

- 地图模型：支持静态栅格、AGV 占用、任务目标点临时开放。
- AGV 模型：支持移动、装货、卸货、状态切换和局部避障。
- 任务模型：支持从 `MAT` / `JSON` 文件加载任务定义。
- 全局规划：使用 4 邻域 A*，采用改进代价函数 `f(n)=g(n)+(1+r/R)*h(n)`。
- 时间窗管理：支持路径预约、同向冲突、对向冲突、失效路径标记。
- 调度器：支持优先级 + FIFO 排序、插队、顺路合并、冲突后等待或重规划。
- 局部避障：基于轻量 DWA 思路采样局部动作并重新接回全局路径。
- 仿真与可视化：支持无界面测试与 MATLAB 图形界面实时展示。

## 目录结构

- `main.m`：项目默认入口。
- `params.m`：默认仿真参数。
- `+map/MapClass.m`：仓储地图模型。
- `+agv/AGVClass.m`：AGV 运行时模型。
- `+task/TaskClass.m`：任务对象定义。
- `+task/TaskParser.m`：任务数据加载器。
- `+pathplan/AStar.m`：A* 路径规划函数。
- `+pathplan/PathLibrary.m`：候选路径库生成器。
- `+pathplan/DWAClass.m`：局部避障规划器。
- `+timewindow/TimeWindowManager.m`：时间窗管理器。
- `+scheduler/SchedulerClass.m`：任务调度器。
- `+sim/Simulation.m`：仿真主循环。
- `+sim/Visualizer.m`：实时可视化。
- `data/`：地图、任务、AGV 池、路径库等数据文件。
- `tests/`：单元测试与场景测试脚本。

## 运行方式

在 MATLAB 当前工作目录切换到项目根目录 `C:\Aneed` 后执行：

```matlab
main
```

如果希望直接创建默认仿真实例并运行：

```matlab
config = params();
simulation = sim.Simulation.fromDefaults(config);
results = simulation.run();
```

## 测试方式

运行单个测试：

```matlab
addpath(pwd);
addpath(fullfile(pwd, 'tests'));
test_astar
```

运行完整测试集：

```matlab
addpath(pwd);
addpath(fullfile(pwd, 'tests'));
test_map;
test_agv;
test_task;
test_astar;
test_timewindow;
test_pathlibrary;
test_scheduler;
test_dwa;
test_simulation;
test_scenario;
test_performance;
```

## 默认场景说明

- 默认地图为仓储栅格地图，包含通道、货架和平台区域。
- 默认任务集包含 5 个任务。
- 默认 AGV 池包含 3 台 AGV。
- `tests/test_scenario.m` 会执行完整场景并输出关键指标。
- `tests/test_performance.m` 会重复运行完整场景并记录 wall-clock 耗时。

## 性能与实现说明

- `AStar` 已从线性扫描开放列表优化为二叉最小堆优先队列，降低大图搜索时的取最小代价开销。
- `TimeWindowManager` 会按边建立有序窗口索引，仅在相同边段上做时间重叠检测，减少无关窗口扫描。
- 默认测试以无界面模式运行，便于持续回归验证。

## 输出结果

仿真结束后可从 `results` 中读取：

- `currentTime`：仿真结束时刻。
- `completedTaskCount`：已完成任务数量。
- `eventLog`：事件日志。
- `metrics.totalTime`：总耗时。
- `metrics.collisionCount`：碰撞次数。
- `metrics.conflictResolutionCount`：冲突解决次数。
- `metrics.replanCount`：重规划次数。
- `metrics.avoidanceCount`：避障次数。
- `metrics.agvDistances`：每台 AGV 行驶距离。
- `metrics.taskCompletionTimes`：每个任务完成时间。

## 开发说明

- 项目遵循 `AGENTS.md` 中的任务执行规则。
- 新增或修改模块时，建议同步补充 `tests/` 中对应测试。
- 若扩展地图或任务数据，优先沿用现有 `MAT/JSON` 数据格式。
