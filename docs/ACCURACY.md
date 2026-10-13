# Accuracy, limits and safety

All numbers here come from `./build/itpms_accuracy 6`. It runs **1,404 simulated drives** in parallel. Each drive uses a different random car: every tire has its own manufacturing radius spread, pressure sensitivity (±10%), resonance frequency (40–45 Hz), damping and frequency exponent, none of which the detector is told. Each drive calibrates at nominal pressure, deflates one tire (or all four), then drives 10 more minutes. The estimate is averaged over the last 3 minutes. Raw output: [accuracy-results.md](accuracy-results.md).

## How accurate is it?

| True loss | Tires | Runs | Mean error | Std dev | Worst error | Warned | Median time to warn |
|---|---|---|---|---|---|---|---|
| 5% | one | 24 | −0.1 pp | 1.7 pp | 3.9 pp | 0/24 | – |
| 5% | all four | 6 | +0.7 pp | 1.4 pp | 2.7 pp | 0/6 | – |
| 10% | one | 24 | +0.7 pp | 1.9 pp | 4.1 pp | 0/24 | – |
| 10% | all four | 6 | +0.8 pp | 1.2 pp | 2.7 pp | 0/6 | – |
| 15% | one | 24 | −0.4 pp | 1.8 pp | 3.4 pp | 3/24 | 7.9 min |
| 15% | all four | 6 | −1.1 pp | 1.6 pp | 3.3 pp | 0/6 | – |
| 20% | one | 24 | −0.1 pp | 1.8 pp | 4.6 pp | 23/24 | 4.2 min |
| 20% | all four | 6 | −0.4 pp | 1.5 pp | 3.0 pp | 5/6 | 2.4 min |
| 25% | one | 24 | −0.5 pp | 1.8 pp | 3.9 pp | 24/24 | 2.2 min |
| 25% | all four | 6 | +0.2 pp | 2.6 pp | 4.8 pp | 6/6 | 1.1 min |
| 30% | one | 24 | +0.0 pp | 1.9 pp | 3.9 pp | 24/24 | 1.4 min |
| 30% | all four | 6 | −0.3 pp | 1.1 pp | 1.4 pp | 6/6 | 0.6 min |
| 40% | one | 24 | +0.2 pp | 2.3 pp | 4.1 pp | 24/24 | 0.8 min |
| 40% | all four | 6 | −0.7 pp | 1.1 pp | 2.3 pp | 6/6 | 0.4 min |

*pp = percentage points: estimating 28% for a true 30% is −2 pp.*

**In short:** the estimate is unbiased, with about **±1.8 pp** of scatter (1σ) and a worst case of about 5 pp. At 35 psi nominal, that's about ±0.6 psi.

## What's the smallest drop it can detect?

There are two answers, because *seeing* a drop and *warning* about it are different things:

- **Seeing it.** A 10% drop is visible in the estimate (10 ± 2%). But a healthy tire's estimate can wander up to **8.3%**, the worst seen in 702 healthy drives. So anything below ~10% can't be told apart from noise.
- **Warning about it.** The warning threshold is set at **20%**, with hysteresis (it clears below 12%). That threshold gives zero false alarms with a wide margin, and at 20% it warns in 23 of 24 runs. 25% and above always warn, in ~1–2 minutes. This matches what regulations ask for: UN R141 requires detecting a 20% loss, and US FMVSS 138 requires a warning at 25% below placard.

The threshold could probably go down to **~12–15%** without false alarms in this simulator. The noise floor is set mainly by the resonance-based common-mode term, which carries about ±5% of jitter into every wheel. That is the clearest next improvement (e.g. longer averaging of the common mode only). The rolling-radius part on its own resolves relative differences far more finely.

## What limits accuracy?

| Source | Effect | Mitigation in the code |
|---|---|---|
| Per-tire pressure sensitivity unknown (±10%) | Proportional error: a 30% loss reads 27–33% | None possible without knowing the tire; real systems tune per tire model |
| Resonance estimate jitter (broad peak, ζ≈0.12) | ±0.5 Hz → ±5% loss, shared by all wheels via the common mode | 32 s spectrum averaging, log-parabola peak fit |
| Drive slip (≈0.3% at 100 km/h, same size as the signal) | Biases the axle indicator | RLS baseline on speed² and acceleration; diagonal indicator cancels it |
| Cornering | Biases the side indicator | Yaw-rate kinematic correction; gate above 2 m/s² lateral acceleration |
| Tone-ring machining errors | Spurious harmonics in the resonance band | Per-tooth correction learning |
| Pressure at calibration | Learned as "normal" | Driver must press TPMS reset at correct pressure (UI warns) |

## Can it handle cat's eyes?

Yes. The measurements show it already did *before* any special handling:

| Road | Impulse defences | Estimate std dev | Worst error | Detected (loss ≥ 20%) | False alarms | Healthy-tire noise (max) |
|---|---|---|---|---|---|---|
| plain road | off | 1.8 pp | 5.0 pp | 118/120 | 0/234 drives | 8.4% |
| plain road | on | 1.8 pp | 4.8 pp | 118/120 | 0/234 drives | 8.3% |
| cat's eyes, lane changes | off | 1.8 pp | 4.8 pp | 117/120 | 0/234 drives | 7.5% |
| cat's eyes, lane changes | on | 1.8 pp | 5.0 pp | 117/120 | 0/234 drives | 7.7% |
| cat's eyes, straddling all trip | off | 1.5 pp | 5.0 pp | 117/120 | 0/234 drives | 7.5% |
| cat's eyes, straddling all trip | on | 1.5 pp | 5.3 pp | 117/120 | 0/234 drives | 7.8% |

Why it's robust: **no decision is ever based on an instant.**

- A rolling-radius sample is 0.25 s of *whole tooth counts*. A stud strike is a ~10 ms event that adds ~0.01 rad of extra rotation, about 0.05% of that window, on one wheel.
- Warnings come from an EWMA over ~100 s of accepted samples (400 samples). One sample hit by a stud moves the estimate by about 1/400 of its tiny error.
- A stud strike kicks the sidewall, and the sidewall rings **at its own resonance**. To the frequency method, a stud strike is just more excitation of exactly the signal it's measuring, so the peak doesn't move.
- Straddling a stud line for a whole trip adds a steady ~0.02% to the left wheels' rotation. That's below 1% equivalent pressure.

Two inexpensive defences were added anyway. They help against harsher impulses (potholes, expansion joints, rumble strips) that the simulator doesn't model as severely:

1. **~2 s running median** (9 samples) on each rolling-radius indicator. It throws away up to 4 outlier samples in any 2.25 s window, where an average would smear them in. A real pressure change passes through with about one second of delay.
2. **Impulse clipping in FFT frames.** Road texture produces Gaussian-like rim-speed noise, with peak/RMS around 3–4. Samples beyond 4.5 × RMS are clipped in two passes. Normal roads trigger it in about 2 of ~9,600 frames; straddling studs triggers it on nearly every frame of that side. Clipping instead of dropping frames means a studded road never starves the estimator. An earlier frame-rejection version did starve it, which is why it was replaced.

Together they cost ~130 bytes of RAM and no measurable CPU.

## Is it safe?

**Not as it stands. It is a well-tested demonstration, not a safety device.** Concretely:

- **Simulator-validated only.** Every number above comes from a simulator I wrote, so the detector and its test world share assumptions. Real tires, wheel bearings, drivetrains, ABS sensor air gaps and roads will have effects that aren't modelled.
- **Not modelled yet:**
  - temperature (pressure changes ~1 psi per 10 °F)
  - load changes and trailers
  - tire wear
  - snow chains, mismatched or spare tires
  - run-flat tires (very small radius change)
  - rough gravel, ABS and ESC interventions beyond simple gating
  - four-wheel drive
  - sensor faults
- **Inherent limits of indirect TPMS:**
  - It can't measure absolute pressure.
  - It learns whatever pressure is present at calibration.
  - It needs minutes of suitable driving to calibrate and to detect.
  - It can't detect a loss while parked.
- **What production would require:**
  - type approval to UN R141 (EU) and/or FMVSS 138 (US), with their test procedures
  - an ISO 26262 functional-safety process with an assigned ASIL
  - fault detection on the sensors themselves (missing teeth, air-gap dropouts, implausible signals)
  - validation over many thousands of kilometres of real driving

Treat a warning from this code as "interesting", and a lack of warning as meaning nothing. Check tire pressures with a gauge.
