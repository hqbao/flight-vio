# `lidar/` — downward rangefinder (VL53L1X over I2C → `lidar.range`)

A standalone, independently-runnable sibling project (`imu_camera`, `depth`, `vio`,
`ba`, `slam`, `ui`, `netbridge`, `fc`, `lidar`). It reads a **downward-facing
VL53L1X** time-of-flight rangefinder (a **bare VL53L1X breakout**) over **I2C** via a
**pure-`smbus2` register-level driver** and publishes each gated reading on the
`lidar.range` IPC topic, served on its own `oak.lidar` endpoint.

The range is **not** a separate flight-controller channel: the [`fc`](../fc/) UART
sender opens a read-only client on `oak.lidar`, keeps the freshest reading, and
**bundles** `range_m` (+ its validity) into the dblink VIO-pose frame (`sky.fc.dblink
pack_vio_pose`: the trailing `range_m` @ offset 38 + the `VIO_FLAG_RANGE_VALID` =
`0x08` flag bit). See [`fc/README.md` → Bundled downward range](../fc/README.md#bundled-downward-range-vl53l1x).

```
                        lidar.main (oak.lidar)
                        ----------------------
  VL53L1X --I2C--> RangeReader.read() -> gate -> WireRange --lidar.range--> fc client
                                                                            (bundles into
                                                                             DB_CMD_VIO_POSE)
```

`lidar` is a **pure producer**: unlike `depth` / `vio` / `slam` it subscribes to
**nothing** — no capture dependency, no calib barrier, no shared-memory rings. It just
opens the sensor and publishes. So a missing / down / `--no-lidar` lidar process simply
means `fc` never sees a range (it sends `range_valid=0`); the VIO send is unaffected.

## Layers

| File | Role |
|------|------|
| `lidar/comms/` | the **FROZEN** vendored comms contract (byte-identical to the other copies); `lidar` consumes only its server API |
| `lidar/io/vl53l1x_reader.py` | the swappable I2C reader: `VL53L1XReader` (a bare VL53L1X driven register-level with `smbus2` only) + `MockRangeReader` (hardware-free, for host tests); the pure `gate_reading` validity rule |
| `lidar/main.py` | the standalone process: read loop → publish `WireRange` on `lidar.range` (a non-blocking `IPCPubSub` server) |
| `lidar/tools/characterize.py` | I2C bench tool: stream dist + `range_status` + signal and, on the ground, print the recommended FC `disarm_range` |
| `lidar/tests/lidar_mock_selftest.py` | mock-sensor read → gate → publish selftest (no I2C) |

`cv2-free`: the only third-party dep is `smbus2` (pure-Python, installs cleanly on the
Pi5 aarch64/py3.13 with no build) — it **is** the whole VL53L1X driver, talking the
chip's register map directly. Nothing here imports OpenCV, so the lean Pi flight image
(`requirements-flight.txt`) stays clean.

## Hardware / wiring

| Item | Value |
|------|-------|
| Sensor | bare VL53L1X breakout, downward-facing, AGL rangefinder |
| Driver | pure-`smbus2` register-level (`VL53L1XReader`): writes the 91-byte ST/Adafruit default config block at init, then reads `RESULT__RANGE_STATUS` + distance directly. Verified on-device (model id `EA CC 10`, status `0x09`, live distance) |
| Bus | **I2C** — Pi `/dev/i2c-1` (the 40-pin header bus; `DEFAULT_I2C_BUS = 1`) |
| Address | **`0x29`** (the VL53L1X default 7-bit address; `DEFAULT_I2C_ADDRESS = 0x29`) |
| Pi pins | **pin 3 (SDA1 / GPIO2)** → sensor SDA, **pin 5 (SCL1 / GPIO3)** → sensor SCL, **3V3** (pin 1) → VIN, **GND** (pin 6/9) → GND |
| Distance mode | **long @ 50 ms** (the bench-proven config the register init writes, ~4 m range; `DIST_MODE_SHORT` ~1.3 m is a future tuning) |
| Timing budget | 50 ms inter-measurement (set by the macro-period registers in `_init_sensor`) → continuous ranging at ~20 Hz |

> ℹ️ **Wiring is overridable.** The bare VL53L1X answers at its factory address `0x29`
> (verified on-device); the reader is behind a tiny `RangeReader` interface, so override
> the bus/address with `--i2c-address` / `--i2c-bus` (or the launcher's
> `--lidar-i2c-address` / `--lidar-i2c-bus`) if you re-strap it. Enable the Pi's I2C bus
> first (`raspi-config` → Interface → I2C) and confirm the device responds:
> `i2cdetect -y 1` should show `0x29`.

## The validity gate (`range_status` + distance band)

`gate_reading(dist_mm, range_status)` is a pure function (unit-testable in isolation):

```
valid = (range_status == 0x09) AND (LIDAR_MIN_MM <= dist_mm <= LIDAR_MAX_MM)
        # RANGE_STATUS_OK = 0x09,  LIDAR_MIN_MM = 30,  LIDAR_MAX_MM = 4000  (millimetres)
```

`range_status == 0x09` is the VL53L1X `RESULT__RANGE_STATUS` (reg `0x0089`) code for a
completed range (verified on-device); **any** other status (sigma fail, signal fail,
wrap-around, out-of-bounds, …) rejects the reading. The distance band additionally
rejects a reading below the sensor's near dead-zone / a spurious zero and above trusted
range **even when** `range_status == 0x09`. The chip reports **millimetres**; the
published / wire value is **metres** (`range_m`). On a reject, `range_m` is forced to
`0.0`.

> ℹ️ **The status gate reads the raw register.** `VL53L1XReader.read()` reads
> `RESULT__RANGE_STATUS` (reg `0x0089`) directly over `smbus2` and compares it to
> `RANGE_STATUS_OK = 0x09` — there is no driver-accessor dependency to verify, and the
> status gate always applies (it never degrades to distance-band-only). The register
> map is confirmed present on-device (model id `EA CC 10`).

`WireRange` (the `lidar.range` POD) carries `{ seq, ts_ns, range_m, valid }` — `seq`
is a monotone reading counter (drop detection), `ts_ns` is the host `monotonic_ns`
capture instant the `fc` side uses for its freshness gate, `range_m` is metres,
`valid` is `0/1` (kept an int so it maps 1:1 onto the FC's `range_valid` flag).

## Run

```bash
# Standalone, on the Pi (serves lidar.range on oak.lidar @ 50 Hz):
python -m lidar.main --endpoint oak.lidar --rate 50

# Deviceless dry-run / host smoke (the hardware-free MOCK reader, no I2C bus):
python -m lidar.main --mock

# Override the (HIL-unknown) wiring once the bench address/bus is known:
python -m lidar.main --i2c-bus 1 --i2c-address 0x29
```

| Flag | Effect |
|------|--------|
| `--endpoint EP` | this process's IPC endpoint (default `oak.lidar`) |
| `--rate HZ` | I2C read + publish cadence, **clamped `[1, 100]`** (default 50). The VL53L1X short-mode budget is ~20 ms, so ~50 Hz is the practical ceiling. |
| `--i2c-bus N` | Linux I2C bus number (default 1 → `/dev/i2c-1`) |
| `--i2c-address A` | VL53L1X 7-bit I2C address (default `0x29`; accepts `0x..`) |
| `--mock` | use the hardware-free MOCK reader (host dry-run / smoke, **not** flight) |
| `--max-reads N` | stop after publishing N readings (0 = run forever) |

The lidar process is **non-fatal**: a real-reader open failure is logged and the
process exits non-zero (the launcher does **not** take the pipeline down — `fc` just
keeps sending `range_valid=0`). A per-read I2C error never raises; it becomes a
`valid=0` sample so a flaky sensor cannot crash the flight loop.

### In the live pipeline (launcher)

The launcher spawns `lidar` **after `slam`, before `fc`** (so its endpoint is up when
`fc` opens its read-only client), and auto-wires `fc --lidar-endpoint`. `--no-lidar` is
a spawn gate (mirror of `--no-slam` / `--no-ba`):

```bash
# Pi flight with the rangefinder bundled into the FC link:
./run.sh --no-ui --fc /dev/ttyAMA0 --width 320 --height 200 --no-ba --no-slam

# Same rig with NO rangefinder (fc sends range_valid=0):
./run.sh --no-ui --fc /dev/ttyAMA0 --no-ba --no-slam --no-lidar

# Deviceless integration dry-run (mock lidar through the whole launcher):
./run.sh --no-ui --fc /dev/ttyUSB0 --no-ba --no-slam --lidar-mock
```

| Launcher flag | Effect |
|---|---|
| `--no-lidar` | don't spawn `lidar`; `fc` is not given `--lidar-endpoint` → the dblink VIO-pose frame carries `range_valid=0`. Use on a rig with no rangefinder. |
| `--lidar-rate HZ` | lidar I2C read + publish cadence (clamped `[1,100]` by `lidar.main`; `0` = the default 50 Hz). |
| `--lidar-i2c-bus N` | lidar Linux I2C bus number (default `lidar.main`'s `1`). |
| `--lidar-i2c-address A` | lidar VL53L1X 7-bit I2C address (default `0x29`, the bare breakout's factory address; override only if re-strapped). |
| `--lidar-mock` | run `lidar` with the hardware-free MOCK reader (no I2C) — for a deviceless integration dry-run. |

## Characterize → FC `disarm_range`

The FC arms / disarms partly on the downward range (it must know "this is the ground"
to refuse a takeoff or cut at touchdown). The ground floor is a property of **this**
rig (sensor mounting height above the skids, the sensor's near bias), so it is
**measured, not guessed**. Run on the Pi with the rig sat **on the ground**:

```bash
python -m lidar.tools.characterize --seconds 5
python -m lidar.tools.characterize --mock        # no hardware (output-format demo)
```

It streams each reading's raw fields (distance, `range_status`, signal), accumulates
the **valid** ground readings, and prints:

```
  >>> recommended FC disarm_range = <floor + margin> m  (ground floor <median> + margin <m>)
```

The floor is the **median** of the valid readings (robust to the odd spurious sample);
the margin (default **0.10 m**, `--margin`) is generous so sensor noise + a slightly
uneven floor never reads as "airborne" while sat on the ground. Set the printed value
on the FC (`PARAM_ID_DISARM_RANGE`) so it treats `≤ disarm_range` as "on the ground".

## Self-test

```bash
.venv/bin/python -m lidar.tests.lidar_mock_selftest
```

Exercises the read → gate → publish path with **no I2C hardware**: (a) the pure
`gate_reading` rule (both reject paths — a non-zero `range_status`; an out-of-band
distance — yield `valid=0`); (b) `MockRangeReader` producing `RangeSample`s with
`range_m` in metres (0.0 on reject) and the status carried through; (c)
`run_lidar(mock=True)` publishing `WireRange` on a real IPC server, a client receiving
both valid + invalid readings round-tripping the exact contract the `fc` sender
consumes.

The bare VL53L1X + `smbus2` register driver is **final and verified on-device** (model
id `EA CC 10`, `RESULT__RANGE_STATUS = 0x09`, live distance). Remaining bring-up items
are rig integration, not the driver:

- **Run `lidar.tools.characterize` on the ground** → set the FC `PARAM_ID_DISARM_RANGE`.
- **Full-rig HIL** — the driver reads on-device; running the whole launcher pipeline
  (lidar → fc → dblink) on the assembled drone is the remaining integration check.
