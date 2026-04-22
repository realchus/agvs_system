# Input Data

`sim.Simulation.fromDefaults(config)` reads this folder first.

- `agv_pool.json`: AGV pool. Each item needs `id`, `position`, and `speed`.
- `task_list.json`: task list. Each item needs `id`, `start`, `waypoints`, `priority`, `requestTime`, and `status`.

Default scale:

- 10 AGVs on parking row 1, distributed across columns 21 to 42.
- 30 tasks using shelves, the left picking window, and the right loading window.

Window conventions:

- Left picking window: row 2, columns 1 to 20.
- Right loading window: row 2, columns 43 to 62.

You can edit these JSON files directly to run your own scenarios. Keep positions as `[row, col]`.
