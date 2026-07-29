# Firmware modules

Normal mission code should treat navigation as a service:

```cpp
navigationGoTo(x, y);
NavigationStatus status = navigationGetStatus();
```

Folder ownership:

- `mission/` decides where to go and what to do there.
- `navigation/` decides how to reach one submitted point safely.
- `motion/` applies accepted chassis commands; `MotorControl.cpp` is the only
  periodic motor-output owner.
- `sensors/` publishes range, object, rear, and IMU evidence.
- `operator/` owns Bluetooth commands, telemetry, and diagnostics.
- `core/` owns shared runtime storage, helpers, and top-level scheduling.

The small `Robot.h`, `Navigation.h`, and `RobotTypes.h` files inside subsystem
folders only forward to the visible sketch-root interfaces. Arduino compiles
`src/` recursively but resolves quoted includes from each nested folder.

Do not include `navigation/NavigationInternal.h` outside `navigation/`. Mission
and future pickup mechanisms need only the root `Navigation.h`.
