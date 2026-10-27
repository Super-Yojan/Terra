# Sensor timing experiment

Is the iPhone a usable robot sensor head, and when a `stale_imu` hold fires, is it
iOS or Terra? TerraPhone stamps every sample at each hop and this script splits the
age of the newest IMU sample at each control tick into those hops.

```
CoreMotion sample ──iOS delivery──▶ motion handler ──Terra queue──▶ terra.control block ──push──▶ Rust
  t_sensor                            t_os                             t_queue                     t_done
                                                                            control tick reads `now`; IMU age = now − newest t_sensor
```

The bare logger runs its own `CMMotionManager` on its own queue and only stamps
arrival, so it shows what iOS delivers with none of our code in the way.

## Record

1. TerraPhone → Debug tools → **Sensor timing**.
2. Type the condition code, tap **Start recording**, run 2 minutes, tap **Stop**.

| code | Terra tracking | YOLO | FPV | bare IMU | bare ARKit load | rover |
|---|---|---|---|---|---|---|
| C0 | off | off | off | on | off | still |
| C1 | off | off | off | on | on | still |
| C2 | on | off | off | on | – | still |
| C3 | on | on | off | on | – | still |
| C4 | on | on | on | on | – | still |
| C5 | on | on | on | on | – | driving L2/L3 straight lines |

Run C2 and C4 in both Debug and Release builds (the CSV header records which).

## Fetch

```
xcrun devicectl device copy from --device <id> --domain-type appDataContainer \
  --domain-identifier org.terra.robotics.phone --source Documents/SensorTiming \
  --destination ~/terra-logs/sensor-timing
```

## Analyse

```
python3 tools/sensor_timing/analyze_sensor_timing.py ~/terra-logs/sensor-timing/*.csv --json summary.json --plot plots/
python3 -m unittest discover tools/sensor_timing   # synthetic checks of the attribution
```

Each stale tick gets one cause:

| cause | meaning |
|---|---|
| `ios_sensor_gap` | iOS produced no IMU sample in the window |
| `ios_delivery` | the sample existed but iOS had not called the handler yet |
| `terra_queue` | the handler had it, but `terra.control` was busy (blocker named: `ar_frame`, `floor_apply`, `detector`, `unlogged_block`) |

`bare fresh` counts stale ticks where the bare logger held a sample under 0.1 s old
at the same moment. A high count means iOS had the data and our path lost it.
