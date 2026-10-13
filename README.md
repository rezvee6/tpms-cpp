# Indirect TPMS

Tire-pressure monitoring using only the ABS wheel-speed sensors: no pressure sensors in the wheels.
The C++17 detector is written to a Cortex-M4F budget. It runs natively for tests and benchmarks, and in the
browser through WebAssembly together with a vehicle/tire physics simulator.

## How it detects pressure loss

| Method | Signal | Sees |
|---|---|---|
| **Rolling radius** | A soft tire has a smaller rolling radius and spins faster. Diagonal, side and axle wheel-speed ratios are compared with a baseline learned after a TPMS reset. The baseline is regressed on speed² and acceleration, and cornering is removed using the yaw sensor. | One, two or three tires low (relative) |
| **Torsional resonance** | The sidewall acts as a torsional spring. A 256-point FFT of resampled rim speed shows a peak near 40 Hz that drops as pressure falls. | Absolute loss per wheel, including all four low together |

Fusion: the radius method gives precise *relative* differences between wheels. The resonance method gives the *common mode*, which the radius method can't see.

## Embedded constraints (enforced, not aspirational)

- **RAM:** `sizeof(Detector)` = 12.1 KB, checked against a 32 KB budget with `static_assert`
- **No heap:** a test overrides `operator new` and fails if the detector allocates
- **No exceptions or RTTI.** The core builds with `-Werror -Wconversion -Wsign-conversion -Wdouble-promotion` (float only; double math would be software-emulated on an M4F)
- **ISR-safe input:** wait-free SPSC queue from the capture ISR. Overflows are counted, never silent.
- **Compile-time tables:** FFT twiddles, bit reversal and the Hann window are computed with `constexpr` trig into `.rodata`
- **Timer wrap:** wrap-safe 32-bit arithmetic. The simulator starts 20 s before the 10 MHz timer wraps, so every run exercises the wrap.
- **Tone-ring learning:** per-tooth machining errors are learned so they don't create spurious harmonics in the resonance band

## Build and run

```bash
cmake -S . -B build && cmake --build build -j
./build/itpms_tests                       # unit + scenario tests
./build/itpms_bench                       # per-edge cost, task latency, FFT cost
./build/itpms_cli --minutes 24 --set 12:FL:0.7          # FL drops to 70% at minute 12
./build/itpms_cli --scenario city --leak 10:RR:0.05     # slow leak
./build/itpms_cli --set 8:ALL:0.7                        # all four low
```

Web demo (needs Emscripten):

```bash
./web/build.sh
python3 -m http.server 8765 --directory web   # then open http://localhost:8765
```

## Layout

```
core/      the embedded detector (the only code subject to the constraints)
sim/       vehicle + tire physics simulator and the ECU-style harness
tests/     zero-dependency test runner
bench/     CPU benchmark
tools/     headless scenario CLI
web/       embind bindings, dashboard UI
```

## What the simulator models

Driver and road profiles (highway, country, city, mixed). Cornering kinematics. Drive and brake slip from tire load, plus aero drag. Per-tire manufacturing radius spread and ±10% variation in pressure sensitivity (hidden from the detector). Sidewall torsional mode excited by road roughness. Per-wheel road noise. Tone-ring tooth errors. Timer quantisation. A yaw sensor with bias and noise.
