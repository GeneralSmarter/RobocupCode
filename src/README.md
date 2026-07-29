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

Subsystem sources include sketch-root shared and public headers with explicit
relative paths such as `../../Robot.h` and `../../Navigation.h`. Do not add
duplicate forwarding copies of `Robot.h`, `Navigation.h`, or `RobotTypes.h`
under `src/`.

Do not include `navigation/NavigationInternal.h` outside `navigation/`. Mission
and future pickup mechanisms need only the root `Navigation.h`.
