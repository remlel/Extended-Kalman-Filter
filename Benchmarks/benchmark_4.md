# Benchmark 4 — Unscented Kalman Filter

## 1. Objective

The objective of this benchmark is to evaluate the performance of the Unscented Kalman Filter (UKF) and compare it with the standard Extended Kalman Filter (EKF) under the same tracking conditions.

The UKF follows the same general Kalman filtering pipeline as the EKF, but handles the nonlinear measurement model differently. Instead of explicitly computing a measurement Jacobian and locally linearizing the nonlinear transformation, the UKF propagates a set of sigma points through the nonlinear measurement function. The resulting transformed mean and covariance are then computed directly from the propagated sigma points, avoiding the first-order linearization and the associated approximations introduced by the EKF.

The objective is therefore to determine whether this more direct treatment of the nonlinear measurement model provides a measurable improvement in tracking accuracy and/or track availability and robustness.

The comparison is primarily performed against the standard EKF, since both filters implement the same Kalman filtering framework while differing mainly in their treatment of the nonlinear measurement model.

---

## 2. Experimental Configurations

The benchmark uses the same dataset and evaluation methodology as the previous benchmarks. Values are given as mean / 95th percentile.

The following configurations are used as references:

* Standard EKF without partial initialization and without partial updates.

* Standard EKF without partial initialization and with partial updates.

* Full Restart Heuristic (threshold 1.2) with partial initialization.

* Full Restart Heuristic (threshold 1.2) with partial initialization and partial updates.

The Full Restart Heuristic is included as a reference for the configuration handling partial initialization, using the previously selected reset threshold of 1.2.

---

## 3. Global Evaluation of the UKF

### 3.1 Without Partial Initialization and Without Partial Updates

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error |        NLL Error |
| ------------- | -----------------: | -----------------: | -------------------: | ----------------: | ---------------: |
| **UKF**       |         **90.89%** |    **1.79 / 5.17** |          2.63 / 8.62 | **12.48 / 48.44** | **9.81 / 27.95** |
| Standard EKF  |             90.84% |        1.81 / 5.20 |          2.63 / 8.59 |     12.79 / 49.96 |     9.94 / 28.55 |

The UKF and standard EKF exhibit very similar overall tracking performance in this configuration.

Track availability is almost identical, with only a 0.05 percentage-point difference between the two filters. The mean position error is slightly lower for the UKF, while the mean velocity error is identical.

The uncertainty-related metrics show a somewhat larger difference. The UKF achieves lower mean and 95th-percentile Mahalanobis and NLL errors, indicating that its estimates are slightly better aligned with their associated uncertainty.

However, the differences remain relatively small. These results therefore suggest that replacing the EKF linearization with the UKF sigma-point formulation does not fundamentally change the global tracking behaviour under complete measurements, although a modest improvement can be observed for several estimation metrics.

---

### 3.2 Without Partial Initialization and With Partial Updates

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error |         NLL Error |
| ------------- | -----------------: | -----------------: | -------------------: | ----------------: | ----------------: |
| **UKF**       |         **91.73%** |    **1.82 / 5.24** |          2.64 / 8.66 | **13.36 / 49.10** | **10.24 / 28.29** |
| Standard EKF  |             91.73% |        1.84 / 5.30 |          2.64 / 8.62 |     13.81 / 51.06 |     10.45 / 29.21 |

With partial updates enabled, both filters obtain exactly the same track availability of 91.73%.

The UKF nevertheless provides slightly lower position, Mahalanobis, and NLL errors than the standard EKF. The velocity error remains identical in mean value, while the 95th-percentile velocity error is marginally higher for the UKF.

The differences are again relatively limited, but the same general trend as in the previous configuration can be observed: the UKF maintains essentially the same tracking availability while producing slightly lower estimation and uncertainty-related errors.

---

### 3.3 With Partial Initialization and Without Partial Updates

| Configuration                | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error |         NLL Error |
| ---------------------------- | -----------------: | -----------------: | -------------------: | ----------------: | ----------------: |
| **UKF**                      |         **91.14%** |    **2.03 / 5.58** |      **2.80 / 9.09** | **15.13 / 51.38** | **11.22 / 29.61** |
| Full Restart Heuristic (1.2) |             91.48% |        1.97 / 5.47 |          2.66 / 8.75 |     14.14 / 49.77 |     10.72 / 28.46 |

With partial initialization enabled, the UKF increases track availability from the corresponding baseline by only **0.25 percentage points**, which is substantially lower than the **0.64 percentage-point gain** achieved by both the Full Restart Heuristic and the CMKF. Since the latter corresponds to the maximum achievable gain in this configuration, the UKF therefore exploits only a limited fraction of the available partial initializations.

Despite the limited availability gain, the UKF maintains reasonable estimation accuracy when partial initialization is used. However, its errors remain consistently higher than those of the Full Restart Heuristic across all evaluated metrics, with the difference being particularly visible for velocity and the statistical metrics. This shows that the UKF not only exploits fewer partial initializations, but also provides less accurate estimates on the tracks that it successfully maintains.

---

### 3.4 With Partial Initialization and Partial Updates

| Configuration                | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error |         NLL Error |
| ---------------------------- | -----------------: | -----------------: | -------------------: | ----------------: | ----------------: |
| **UKF**                      |         **91.81%** |    **2.01 / 5.54** |      **2.79 / 9.14** | **15.99 / 52.03** | **11.62 / 29.93** |
| Full Restart Heuristic (1.2) |             92.20% |        1.96 / 5.50 |          2.68 / 8.78 |     15.20 / 51.10 |     11.22 / 29.25 |

When partial initialization and partial updates are combined, the UKF achieves a track availability of 91.81%, compared with 92.20% for the Full Restart Heuristic, corresponding to a **0.39 percentage-point difference**.

The 0.39 percentage-point difference is consistent with the separate analyses of partial measurements. Partial updates produced the same availability gain for both approaches, while partial initialization resulted in a 0.39 percentage-point gap between the UKF and the Full Restart Heuristic. The combined result therefore directly reflects the limited exploitation of partial initialization by the UKF.

The UKF nevertheless maintains relatively close estimation performance, although the Full Restart Heuristic achieves lower errors across all evaluated metrics. The main limitation of the UKF in this configuration is therefore its ability to exploit partial initialization, rather than the subsequent processing of partial updates.

---

## 4. Benchmark Mode Comparison

### 4.1 UKF vs Standard EKF with Complete Measurements

The UKF and standard EKF share **99.7% of their track lifetime in both directions**, indicating an almost identical track survival behaviour.

| Filter           | Track survival |        Position error |            Velocity error |   Mahalanobis error |                NLL |
| ---------------- | -------------: | --------------------: | ------------------------: | ------------------: | -----------------: |
| **UKF**          |         18,178 | 1.79 m (95% < 5.16 m) | 2.61 m/s (95% < 8.55 m/s) | 12.41 (95% < 48.00) | 9.76 (95% < 27.73) |
| **Standard EKF** |         18,168 | 1.80 m (95% < 5.19 m) | 2.61 m/s (95% < 8.54 m/s) | 12.75 (95% < 49.48) | 9.91 (95% < 28.39) |

Overall, the two filters exhibit almost identical tracking performance in this configuration. The estimation errors are essentially unchanged between the two approaches, while the statistical metrics show only a slight difference in favour of the UKF.

This close agreement is consistent with the use of complete and relatively accurate measurements. Under these conditions, the nonlinear measurement processing of the UKF provides little practical advantage over the local linearization used by the EKF. 

---

### 4.2 UKF vs Full Restart Heuristic with Partial Initialization

The UKF and Full Restart Heuristic share **99.5% and 99.0% of their respective track lifetimes**, respectively.

| Filter                     | Track survival |        Position error |            Velocity error |   Mahalanobis error |                 NLL |
| -------------------------- | -------------: | --------------------: | ------------------------: | ------------------: | ------------------: |
| **UKF**                    |         18,219 | 2.01 m (95% < 5.48 m) | 2.76 m/s (95% < 8.94 m/s) | 14.87 (95% < 50.31) | 11.07 (95% < 28.99) |
| **Full Restart Heuristic** |         18,297 | 1.96 m (95% < 5.42 m) | 2.64 m/s (95% < 8.58 m/s) | 14.23 (95% < 49.02) | 10.75 (95% < 28.25) |

As seen previously, the Full Restart Heuristic achieves higher track availability than the UKF in this configuration. The benchmark shows that the difference in estimation performance is also consistent across all metrics, with the UKF showing higher position, velocity, Mahalanobis, and NLL errors. The degradation is moderate but clearly present, particularly for velocity and the statistical metrics.

The UKF is therefore able to handle partial initialization and maintain a high level of track availability, but it does so less effectively than the Full Restart Heuristic (and the CMKF). It both exploits fewer partial initializations and provides less accurate estimates on the tracks that it successfully maintains.

---

## 5. Final Interpretation and Conclusion

### 5.1 Summary of UKF Performance

The evaluation of the Unscented Kalman Filter yields a nuanced conclusion regarding its applicability to the radar tracking pipeline. Under nominal conditions with complete measurements, the UKF demonstrates a slight but measurable improvement over the standard EKF. By bypassing the first-order Taylor series approximation (Jacobian) and using the Unscented Transform (UT) to propagate sigma points, the UKF achieves marginally better statistical consistency (lower Mahalanobis and NLL errors) while maintaining equivalent track availability.

However, the UKF performs less effectively when confronted with partial initializations. While it performs slightly better than the standard EKF, the UKF only achieves a **0.25 percentage-point** gain in track availability. In contrast, both the Full Restart Heuristic and the Converted Measurement Kalman Filter (CMKF) achieve a **0.64 percentage-point gain**. Furthermore, the UKF exhibits higher estimation errors on the tracks it manages to maintain compared to the reference heuristic model.

### 5.2 The Gaussian Limitation on Partial Initializations

The main limitation of the UKF with partial initialization lies in the assumptions underlying the Unscented Transform. The UT uses a deterministic set of sigma points to represent a probability distribution through its mean and covariance, with the standard formulation being designed for Gaussian distributions.

When a partial initialization occurs (e.g., missing azimuth and elevation), the true spatial uncertainty of the target is not Gaussian. Since the exact angles are unknown, the target can lie anywhere within the radar's field of view at the measured range, leading to a **uniform distribution** over a portion of a spherical shell.

When the UKF represents this strongly non-Gaussian uncertainty using only a mean and covariance, the resulting sigma points cannot accurately capture the actual shape of the distribution. After propagation through the nonlinear measurement function, the transformed distribution may therefore be poorly represented, potentially leading to an inaccurate predicted state, distorted covariance, and consequently less effective gating or Kalman gains.

Conversely, the Full Restart Heuristic avoids this issue by discarding the previous state information when the uncertainty becomes excessive, while the CMKF addresses the problem differently by applying the appropriate processing directly in the measurement space before filtering.

### 5.3 Proposed Solutions and Future Work

Based on this benchmark, the models exhibit complementary strengths. The UKF provides a slight performance advantage under nominal complete-measurement conditions, while the Full Restart Heuristic is more effective at exploiting partial initializations and maintaining estimation quality under these degraded conditions.

Two main evolutionary paths can therefore be considered for the tracking architecture:

1. **The Hybrid Tracker (Immediate Implementation):**
   The most pragmatic and immediate solution is to combine the strengths of both approaches by implementing a **UKF with a Heuristic Reset**. Since the UKF shares the same general filtering architecture as the EKF, integrating the threshold-based Full Restart logic into the UKF pipeline would be relatively straightforward. This hybrid model could combine the slight accuracy gains provided by the UKF during standard tracking with the robustness of the heuristic reset when the state uncertainty becomes excessive.

2. **Custom Sigma-Point Distribution (Theoretical Evolution):**
   A more mathematically involved alternative would be to modify the generation of sigma points during partial initializations. Instead of relying exclusively on the standard Gaussian-based sigma-point distribution, the generation could be adapted to distinguish between observed and unobserved dimensions. Standard sigma points could be retained for the observable dimensions (range and radial velocity), while a deterministic set of points distributed across the unobserved dimensions (azimuth and elevation) could be used to better represent their uniform uncertainty. This would provide a richer representation of the actual uncertainty while remaining potentially less computationally expensive than a full particle-filter approach.

Ultimately, the UKF appears promising as part of a hybrid architecture rather than as a standalone solution for degraded measurements. Combining it with the Heuristic Reset is a direct and practical avenue for further investigation. In parallel, improving the CMKF transition model beyond the constant-velocity assumption could help relax its current gating limitations. A comparison between these approaches under equivalent gating conditions would also be necessary to determine their relative performance more rigorously.
