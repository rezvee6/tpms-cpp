<!-- 1404 simulated drives, 6 seeds per case -->

### Accuracy by loss level (plain road)

| True loss | Tires | Runs | Mean error | Std dev | Worst error | Warned | Median time to warn |
|---|---|---|---|---|---|---|---|
| 5% | one | 24 | -0.1 pp | 1.7 pp | 3.9 pp | 0/24 | – |
| 5% | all four | 6 | +0.7 pp | 1.4 pp | 2.7 pp | 0/6 | – |
| 10% | one | 24 | +0.7 pp | 1.9 pp | 4.1 pp | 0/24 | – |
| 10% | all four | 6 | +0.8 pp | 1.2 pp | 2.7 pp | 0/6 | – |
| 15% | one | 24 | -0.4 pp | 1.8 pp | 3.4 pp | 3/24 | 7.9 min |
| 15% | all four | 6 | -1.1 pp | 1.6 pp | 3.3 pp | 0/6 | – |
| 20% | one | 24 | -0.1 pp | 1.8 pp | 4.6 pp | 23/24 | 4.2 min |
| 20% | all four | 6 | -0.4 pp | 1.5 pp | 3.0 pp | 5/6 | 2.4 min |
| 25% | one | 24 | -0.5 pp | 1.8 pp | 3.9 pp | 24/24 | 2.2 min |
| 25% | all four | 6 | +0.2 pp | 2.6 pp | 4.8 pp | 6/6 | 1.1 min |
| 30% | one | 24 | +0.0 pp | 1.9 pp | 3.9 pp | 24/24 | 1.4 min |
| 30% | all four | 6 | -0.3 pp | 1.1 pp | 1.4 pp | 6/6 | 0.6 min |
| 40% | one | 24 | +0.2 pp | 2.3 pp | 4.1 pp | 24/24 | 0.8 min |
| 40% | all four | 6 | -0.7 pp | 1.1 pp | 2.3 pp | 6/6 | 0.4 min |

### Cat's eyes: impulse defences off vs on

| Road | Defences | Estimate std dev | Worst error | Detected (loss >= 20%) | False alarms | Healthy-tire noise (max) |
|---|---|---|---|---|---|---|
| plain road | off | 1.8 pp | 5.0 pp | 118/120 | 0/234 drives | 8.4% |
| plain road | on | 1.8 pp | 4.8 pp | 118/120 | 0/234 drives | 8.3% |
| cat's eyes, lane changes | off | 1.8 pp | 4.8 pp | 117/120 | 0/234 drives | 7.5% |
| cat's eyes, lane changes | on | 1.8 pp | 5.0 pp | 117/120 | 0/234 drives | 7.7% |
| cat's eyes, straddling all trip | off | 1.5 pp | 5.0 pp | 117/120 | 0/234 drives | 7.5% |
| cat's eyes, straddling all trip | on | 1.5 pp | 5.3 pp | 117/120 | 0/234 drives | 7.8% |

Healthy-tire estimate never exceeded 8.3% in 702 robust drives (zero-false-alarm floor).
