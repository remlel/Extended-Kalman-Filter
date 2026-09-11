# Benchmark 3 — Measured Centered Kalman Filter

## 1. Objective

The objective of this benchmark is to investigate whether a **Measured Centered Kalman Filter (MCKF)** can provide a more robust and principled way of exploiting partial measurements and partial initialization.

Benchmark 2 showed that the Restart strategy can significantly improve the robustness of partial initialization. However, this improvement relies on an explicit heuristic mechanism to detect degraded tracks and reinitialize them.

The MCKF provides an alternative approach by reformulating the measurement update in Cartesian coordinates. The objective is therefore to determine whether this formulation can naturally handle partial measurements and partial initialization while maintaining the estimation performance of the standard EKF.

---

## 2. Experimental Configurations

The benchmark uses the dataset and evaluation methodology defined in the benchmark README. Values are given as **mean / 95th percentile**.

### 2.1 Reference Configurations

The following configurations are used as references to assess the performance of the MCKF:
- **Standard — Full Measurements** configuration provides a reference for the estimation performance achievable with complete measurements.
- **Standard — Partial Updates** configuration provides a reference for the handling of partial measurements during the update stage.
- **Restart — Partial Init.** configuration provides a reference for the handling of partial measurements during the initialization stage.
- **Restart — Partial Init. + Partial Updates** configuration represents the most promising solution obtained in the previous benchmarks globally. It is therefore used as the main reference for evaluating whether the MCKF can provide a more natural alternative to the heuristic restart mechanism.

---

## 3. Global Evaluation of the MCKF

### 3.1 MCKF with Full Measurements

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **MCKF — Full Measurements** | **90.84%** | **1.80 / 5.13** | **2.61 / 8.46** | **12.74 / 46.67** | **9.90 / 26.87** |
| Standard — Full Measurements | 90.84% | 1.81 / 5.20 | 2.63 / 8.59 | 12.79 / 49.96 | 9.94 / 28.55 |

The MCKF achieves the same track availability as the standard EKF, while providing slightly lower errors across all evaluated metrics: position, velocity, Mahalanobis and NLL errors. The magnitude of these improvements is small, and their statistical significance cannot be established from this evaluation alone. Nevertheless, the results show that the MCKF performs at least as well as the standard EKF under nominal measurement conditions, with a consistent improvement across all evaluated criteria.

The MCKF does not fundamentally improve the handling of the dominant complete-measurement cases, and the overall track availability therefore remains limited by the difficult scenarios already observed with the standard EKF.

### 3.2 MCKF with Partial Updates

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **MCKF — Partial Updates** | **91.68%** | **1.82 / 5.22** | **2.62 / 8.47** | **13.61 / 47.58** | **10.33 / 27.44** |
| Standard — Partial Updates | 91.73% | 1.84 / 5.30 | 2.64 / 8.62 | 13.81 / 51.06 | 10.45 / 29.21 |

The MCKF maintains a very similar level of track availability when partial updates are enabled.

Compared with the standard EKF with partial updates, the MCKF provides slightly lower position and velocity errors, together with improved Mahalanobis and NLL metrics.

The MCKF provides performance comparable to the standard EKF for partial updates. No significant degradation is observed, but the results do not show a clear improvement either, which is not surprising given that the standard EKF already handles partial updates effectively.

### 3.3 MCKF with Partial Initialization

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **MCKF — Partial Initialization** | 91.44% | 1.96 / 5.47 | 2.67 / 8.62 | 14.21 / 47.38 | 10.73 / 27.05 |
| Restart — Partial Init. | 91.48% | 1.97 / 5.47 | 2.66 / 8.74 | 14.01 / 49.77 | 10.65 / 28.47 |

When partial initialization is enabled, the standard EKF was previously shown to struggle to exploit the subsequent measurements effectively, with valid measurements frequently rejected after the initialization. The MCKF, in contrast, reaches 91.44% track availability, showing that partial initialization can be effectively exploited without leading to the same loss of track continuity.

The estimation errors remain at a reasonable level despite the large uncertainty introduced by the incomplete initialization. The MCKF achieves a performance level comparable to the previously established Restart approach, which was shown to effectively handle partial initialization.

The additional estimation error remains consistent with the increased uncertainty introduced by the partial initialization and does not indicate any abnormal degradation. Overall, the MCKF therefore provides a viable and consistent way of exploiting partial initialization.

### 3.4 MCKF with Partial Initialization and Partial Updates

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| **MCKF — Partial Init. + Partial Updates** | **92.15%** | **1.96 / 5.46** | **2.67 / 8.65** | **14.96 / 48.32** | **11.07 / 27.60** |
| Restart — Partial Init. + Partial Updates | 92.20% | 1.96 / 5.50 | 2.68 / 8.78 | 15.20 / 51.10 | 11.22 / 29.25 |

The combination of partial initialization and partial updates provides the most significant result of this benchmark.

The MCKF reaches 92.15% track availability, essentially matching the 92.20% obtained with the Restart strategy. The estimation errors are also very similar, with only marginal differences across the evaluated metrics.

These differences are too small to support any meaningful claim of superiority. The results nevertheless show that the MCKF achieves a level of performance at least comparable to the previously established Restart solution.

This is particularly interesting because the MCKF achieves a comparable level of robustness without relying on an explicit restart heuristic. The improvement instead results directly from the formulation of the measurement update.

---

## 4. Benchmark Mode Comparison

The **MCKF — Partial Init. + Partial Updates** configuration is compared with the reference configurations using the common set of estimated states.

### 4.1 Comparison with Standard Full Measurements

| Configuration | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL |
|---|---:|---:|---:|---:|
| **Standard — Full Measurements** | 1.78 / 5.11 | 2.57 / 8.33 | 12.62 / 48.64 | 9.84 / 27.83 |
| **MCKF — Partial Init. + Partial Updates** | 1.80 / 5.12 | 2.59 / 8.31 | 12.57 / 47.29 | 9.77 / 26.98 |

The MCKF achieves **virtually identical estimation performance** to the standard EKF with complete measurements when evaluated on their common survival period.

The differences remain very small across all metrics, those differences are not statistically significant. 

This confirms that enabling partial initialization and partial updates in the MCKF does not significantly degrade the estimation quality compared with the standard EKF under complete measurements.

### 4.2 Comparison with Heuristic Restart

The MCKF is finally compared with the previously established Restart — Partial Init. + Partial Updates configuration using the common set of estimated states.

The two configurations share 18,366 updates, corresponding to 99.7% of the MCKF track life and 99.6% of the Restart track life. This very high overlap ensures that the comparison is performed on essentially the same tracking states.

| Configuration | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL |
|---|---:|---:|---:|---:|
| **MCKF — Partial Init. + Partial Updates** | 1.94 / 5.40 | 2.64 / 8.49 | 14.80 / 47.54 | 10.98 / 27.17 |
| **Restart — Partial Init. + Partial Updates** | 1.94 / 5.42 | 2.63 / 8.50 | 15.04 / 50.05 | 11.13 / 28.46 |

The two configurations exhibit essentially identical estimation performance on their common states. The differences are marginal across all metrics, with no meaningful difference in position or velocity accuracy and only small differences in Mahalanobis error and NLL.

These results confirm that the MCKF reaches the same level of tracking performance as the previously established Restart solution, making it a valid alternative to the heuristic approach.

---

## 5. Interpretation

The main benefit of the MCKF appears when the filter has to handle large state uncertainties during partial initialization. In the standard EKF, the nonlinear position measurement model is locally linearized around the predicted state. When the initial position is highly uncertain, this linearization can become inaccurate, leading to an incorrect projection of the prediction covariance and, consequently, to inconsistent Mahalanobis distances. Valid subsequent measurements may then be rejected during the validation stage.

The MCKF avoids this issue for the position update by expressing the position measurement directly in Cartesian coordinates. The position measurement model is therefore linear, removing the need to evaluate a position Jacobian around the uncertain predicted state. This allows the measurement information and its associated uncertainty to be incorporated without relying on a potentially inaccurate local linearization, providing a more consistent state and covariance estimate after partial initialization.

In contrast, with complete measurements, the MCKF and standard EKF exhibit almost identical behavior. In this case, the measurement uncertainty is much smaller and the predicted state is generally sufficiently close to the measurement for the local linearization of the nonlinear measurement model to remain a good approximation. The different measurement-update formulations therefore lead to essentially the same estimation performance under nominal conditions.

However, the MCKF does not completely eliminate nonlinear effects. The Doppler measurement still depends nonlinearly on the Cartesian position and velocity, and its update therefore remains sensitive to the quality of the intermediate position estimate. This limitation is particularly relevant at short range, where a relatively small position error can produce a significant angular error and consequently a large error in the predicted radial velocity.

The results therefore suggest that the MCKF effectively addresses the linearization issue associated with the position update during partial initialization, while the remaining limitations are mainly related to the nonlinear Doppler update and the underlying prediction model.

| **Successful Partial Initialization with MCKF** | **Failed Partial Initialization with Standard EKF** |
|:---:|:---:|
| <img src="../Upload/CMKF_partial_init.png" height="700"> | <img src="../Upload/Standard_EKF_partial_init.png" height="700"> |

---

## 6. Conclusions

The MCKF provides a significant improvement in robustness when partial initialization is enabled while preserving the estimation performance of the standard EKF.

The main conclusions are:

- With full measurements, the MCKF and standard EKF provide globally identical tracking performance.
- With partial updates, the MCKF maintains comparable availability and slightly improves the estimation metrics.
- With partial initialization, the MCKF provides a clear improvement in track availability and estimation performance.
- With both partial initialization and partial updates, the MCKF reaches **92.15% availability**, essentially matching the **92.20% obtained with the Restart strategy**.

The MCKF therefore appears to provide a more principled and natural solution to the partial-initialization problem than the previously selected Restart strategy.

The remaining limitation is that the MCKF does not address the fundamental weaknesses of the current constant-velocity prediction model or the nonlinear Doppler update. Further improvements should therefore focus on these aspects.