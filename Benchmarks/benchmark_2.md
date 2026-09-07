# Benchmark 2 — Robust Partial Initialization with Restart

## 1. Objective

The objective of this benchmark is to investigate whether a restart mechanism can make partial initialization robust while preserving the estimation performance of the best two configurations of the standard EKF (full measurements and partial updates).

Benchmark 1 showed that partial updates can be successfully exploited, whereas partial initialization significantly degrades the tracking performance. This benchmark therefore evaluates a restart strategy designed to recover from unreliable partial initializations while preserving the benefits of partial measurements.

Two restart strategies were initially investigated:

- **Full Restart**, where the state is fully reinitialized from the available measurement.
- **Hybrid Restart**, where the measured position is reinitialized while the predicted velocity is preserved.

For both strategies, different restart thresholds are evaluated to identify an appropriate trade-off between availability and estimation accuracy.

---

## 2. Experimental Configurations

The benchmark uses the dataset and evaluation methodology defined in the benchmark README. Values are given as **mean / 95th percentile**.

### 2.1 Reference Configurations

The following configurations from Benchmark 1 are used as references:

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **Standard — Full Measurements** | 90.84% | 1.81 / 5.20 | 2.63 / 8.59 | 12.79 / 49.96 | 9.94 / 28.55 |
| **Standard — Partial Updates** | 91.73% | 1.84 / 5.30 | 2.64 / 8.62 | 13.81 / 51.06 | 10.45 / 29.21 |
| Standard — Partial Init. + Partial Updates | 90.65% | 2.08 / 5.84 | 2.82 / 9.08 | 23.47 / 54.85 | 15.40 / 31.68 |

---

## 3. Global Evaluation of the Restart Strategy

### 3.1 Full Restart

The Full Restart strategy was evaluated using several restart thresholds.

| Restart Threshold | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **1.2** | **92.20%** | **1.96 / 5.50** | **2.68 / 8.78** | 15.20 / 51.10 | 11.22 / 29.25 |
| **0.99** | 96.75% | 2.11 / 5.91 | 3.82 / 11.97 | 8.00 / 23.18 | 12.29 / 17.89 |
| **0.9** | 98.34% | 2.82 / 8.46 | 4.36 / 12.60 | 4.39 / 7.59 | 13.30 / 15.76 |

The results clearly demonstrate the trade-off introduced by the restart threshold.

With a threshold of **1.2**, the Full Restart significantly improves the global tracking performance compared with the standard Partial Init. + Partial Updates configuration. Track availability increases from **90.65% to 92.20%**, while both position and velocity errors are reduced.

More aggressive restart thresholds further increase availability. At **0.99**, availability reaches **96.75%**, while at **0.9** it reaches **98.34%**, close to the proportion of measurements containing usable information in the dataset.

However, this increased availability comes at the cost of progressively degraded position and velocity accuracy. The very low Mahalanobis errors obtained with the most aggressive thresholds should therefore not be interpreted as evidence of better overall tracking performance. The filter increasingly relies on measurement-driven resets and loses the temporal information provided by the EKF.

The threshold of **1.2** therefore appears to provide the most promising operational compromise: it substantially improves availability while preserving the benefits of temporal filtering.

> **Note:** The restart threshold does not affect the results continuously. Since the decision depends on the discrete values encountered by the restart criterion, multiple threshold values can lead to exactly the same behavior. For example, thresholds of **1.1** and **1.2** produce identical results in the current evaluation.

---

### 3.2 Hybrid Restart

The Hybrid Restart strategy was evaluated to determine whether preserving the predicted velocity during a restart could provide a better compromise.

A representative configuration with a threshold of 1.0 is shown below.

| Restart Strategy | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **Hybrid Restart — threshold 1.0** | 92.15% | 2.16 / 6.13 | 3.48 / 10.89 | 16.05 / 44.59 | 14.42 / 26.54 |

The Hybrid Restart does not provide a satisfactory trade-off. Similar behavior is observed for the other tested thresholds, with no configuration approaching the performance of the Full Restart.

This behavior is consistent with the role of the restart mechanism. A restart is triggered when the predicted state is considered unreliable. In such a situation, preserving the predicted velocity may retain an inaccurate representation of the target dynamics. Combining a newly measured position with an unreliable historical velocity can therefore lead to an inconsistent state.

The Hybrid Restart is consequently discarded in favor of the Full Restart strategy.

---

## 4. Benchmark Mode Comparison

The most promising configuration, **Restart — Partial Init. + Partial Updates with a restart threshold of 1.2**, is compared against the two relevant standard configurations using the common set of estimated states.

### 4.1 Comparison with Standard Full Measurements

| Configuration | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL |
|---|---:|---:|---:|---:|
| **Standard — Full Measurements** | 1.81 / 5.20 | 2.61 / 8.52 | 12.78 / 49.99 | 9.93 / 28.53 |
| **Restart — Partial Init. + Partial Updates** | 1.82 / 5.24 | 2.62 / 8.52 | 12.92 / 50.73 | 9.99 / 28.95 |

The Restart configuration achieves **virtually identical estimation performance** to the standard EKF using only complete measurements.

The difference is negligible across all evaluated metrics: position error increases by only **0.01 m** on average, velocity error by **0.01 m/s**, while the Mahalanobis and NLL errors remain extremely close.

This is a strong indication that the use of partial initialization and partial updates does not significantly degrade the tracking quality on states where complete measurements are available.

---

### 4.2 Comparison with Standard Partial Updates

| Configuration | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL |
|---|---:|---:|---:|---:|
| **Standard — Partial Updates** | 1.83 / 5.30 | 2.63 / 8.58 | 13.81 / 51.06 | 10.44 / 29.20 |
| **Restart — Partial Init. + Partial Updates** | 1.83 / 5.29 | 2.63 / 8.58 | 13.80 / 51.02 | 10.44 / 29.15 |

The Restart configuration also provides **essentially identical performance** to the standard configuration with partial updates.

The differences are negligible across all metrics, with the Restart configuration even showing marginally lower 95th-percentile errors for position, Mahalanobis error, and NLL.

This result is particularly important because it shows that introducing partial initialization does **not introduce an additional performance penalty during the subsequent partial-update tracking process**.

The slight degradation observed in the global evaluation of the Restart configuration can therefore be primarily attributed to the more challenging partial-initialization cases themselves, rather than to a degradation of the subsequent tracking process.

---

## 5. Conclusions

The results demonstrate that the proposed **Full Restart strategy successfully stabilizes partial initialization while preserving the tracking performance of the standard EKF**.

A restart threshold of **1.2** provides the most convincing compromise in the current evaluation. It increases global track availability from **90.65% to 92.20%** compared with the standard Partial Init. + Partial Updates configuration, while also improving the global position and velocity errors.

More importantly, the Benchmark Mode results show that the Restart configuration achieves virtually the same estimation performance as both relevant standard configurations when evaluated on common states.

In particular:

- Compared with **Standard — Full Measurements**, the Restart configuration achieves almost identical estimation accuracy.
- Compared with **Standard — Partial Updates**, the Restart configuration also achieves essentially identical performance.
- The additional partial-initialization capability therefore does not appear to degrade the subsequent tracking performance.

These results provide strong evidence that the proposed restart mechanism successfully addresses the main weakness identified in Benchmark 1: **partial initialization can be exploited without significantly compromising the quality of the resulting track**.
