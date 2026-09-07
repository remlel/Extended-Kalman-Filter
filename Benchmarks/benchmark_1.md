# Benchmark 1 — Baseline Kalman Filter and Partial Measurements

## 1. Objective

The objective of this benchmark is to characterize the behavior of the current tracking pipeline when using a basic Extened Kalman Filter (EKF) under different levels of measurement completeness.

The current implementation relies on an Extended Kalman Filter with no additional mechanism specifically designed to handle partial initialization or partial measurement updates. The filter parameters, including the process and measurement noise models, have been calibrated using knowledge of the dataset and the characteristics of the simulated measurements. The resulting configuration therefore provides a consistent and representative implementation for the considered tracking problem.

This first benchmark is intended to:

- characterize the current performance of the EKF;
- evaluate the impact of partial measurement updates;
- evaluate the impact of partial initialization;
- identify the current limitations of the tracking pipeline;
- provide quantitative evidence motivating the development of more robust tracking strategies.

The different configurations evaluated in this benchmark should therefore be considered as different operating conditions of the **same EKF**, rather than as independent tracking solutions.

---

## 2. Experimental Protocol

Four configurations of the EKF are evaluated:

| Configuration | Partial Initialization | Partial Update |
|---|---:|---:|
| Configuration A | No | No |
| Configuration B | No | Yes |
| Configuration C | Yes | No |
| Configuration D | Yes | Yes |

The four configurations allow the effects of partial initialization and partial updates to be studied independently as well as jointly.

---

## 3. Evaluation Metrics

The tracking performance is evaluated using the following metrics:

- **Track availability** — proportions of states successfully estimated relative to the total number of states.
- **Mean position error** — average position estimation error.
- **95th percentile position error** — characterizes the upper tail of the position error distribution.
- **Mean velocity error** — average velocity estimation error.
- **95th percentile velocity error** — characterizes the upper tail of the velocity error distribution.
- **Mean Mahalanobis error** — evaluates the consistency between the estimation error and the predicted uncertainty.
- **Mean NLL error** — evaluates the probabilistic quality (confidence level) of the state estimates.

The mean error provides information about the overall estimation accuracy, whereas the 95th percentile provides complementary information about the presence of large or extreme errors.

---

## 5. Results

### 5.1 Global Performance

The following results are obtained in evaluation mode.

| Configuration | Track Availability | Position Error [m] | Velocity Error [m/s] | Mahalanobis Error | NLL Error |
|---|---:|---:|---:|---:|---:|
| No Partial Initialization / No Partial Update | 90.84% | 1.81 / 5.20 | 2.63 / 8.59 | 12.79 / 49.96 | 9.94 / 28.55 |
| No Partial Initialization / Partial Update | 91.73% | 1.84 / 5.30 | 2.64 / 8.62 | 13.81 / 51.06 | 10.45 / 29.21 |
| Partial Initialization / No Partial Update | 89.89% | 2.09 / 5.79 | 2.84 / 9.12 | 15.92 / 55.61 | 11.65 / 32.08 |
| Partial Initialization / Partial Update | 90.65% | 2.08 / 5.84 | 2.82 / 9.08 | 23.47 / 54.85 | 15.40 / 31.68 |

For the error metrics, the two values correspond respectively to the **mean** and the **95th percentile**.

The results show two clearly different behaviors depending on the type of partial measurement considered.

---

### 5.2 Impact of Partial Updates

Introducing partial updates increases track availability from **90.84% to 91.73%**.

This improvement is obtained while maintaining relatively similar estimation accuracy:

- mean position error increases from 1.81 m to 1.84 m;
- mean velocity error increases from 2.63 m/s to 2.64 m/s;
- the 95th percentile errors remain of the same order of magnitude.

The probabilistic metrics also increase moderately, with the mean Mahalanobis error increasing from 12.79 to 13.81 and the mean NLL increasing from 9.94 to 10.45.

These results indicate that the current partial-update mechanism is able to recover additional track states without causing a major degradation of the overall estimation quality.

This behavior is consistent with the expected **availability-versus-accuracy trade-off**. A partial update provides less measurement information than a complete update. Some state components may therefore rely more heavily on prediction, which naturally leads to a less precise estimate.

The observed increase in availability nevertheless appears to outweigh this relatively limited loss in accuracy, making partial updates a promising mechanism for improving track continuity.

---

### 5.3 Impact of Partial Initialization

The effect of partial initialization is substantially different.

When partial initialization is introduced without partial updates, track availability decreases from **90.84% to 89.89%**.

At the same time, all estimation error metrics deteriorate:

- mean position error increases from 1.81 m to 2.09 m;
- the 95th percentile position error increases from 5.20 m to 5.79 m;
- mean velocity error increases from 2.63 m/s to 2.84 m/s;
- the 95th percentile velocity error increases from 8.59 m/s to 9.12 m/s;
- mean Mahalanobis error increases from 12.79 to 15.92;
- mean NLL increases from 9.94 to 11.65.

The degradation is therefore not limited to a small number of extreme cases. Both the average performance and the upper tail of the error distributions are affected.

This indicates that the current EKF is not sufficiently robust to the uncertainty introduced during partial initialization.

The degradation observed with partial initialization is not primarily attributed to an inaccurate estimation of the initial state or covariance. The partial initialization procedure explicitly uses a Monte Carlo approach to estimate both the mean state and the associated covariance, providing a consistent representation of the uncertainty induced by the incomplete measurement.

The limitation instead arises from the local linearization inherent to the Extended Kalman Filter (EKF). When the uncertainty associated with the partial initialization becomes large, the estimated state may be located far from the true state. The Jacobian used by the EKF is then evaluated around this estimated state, rather than around the actual region of the state distribution. In a highly nonlinear system, the resulting local linear approximation may no longer accurately represent the system dynamics over the extent of the uncertainty distribution.

Consequently, the EKF may propagate this uncertainty through an inaccurate local linearization. The resulting predicted state and covariance can therefore become inconsistent with the actual state distribution. This inconsistency can subsequently affect the Mahalanobis distance used for measurement validation, potentially causing valid measurements to fall outside the acceptance region and be rejected.

This behavior highlights a fundamental limitation of the EKF in the presence of highly uncertain initial states: accurately representing the initial uncertainty is not necessarily sufficient when the underlying nonlinear dynamics cannot be adequately approximated by a single local linearization.

In such a situation, the tracker may continue propagating an incorrect track through prediction while rejecting measurements that could otherwise have corrected it. This creates a feedback effect in which an initially poor initialization can lead to the loss of the correct track.

---

### 5.4 Combined Partial Initialization and Partial Updates

When both mechanisms are enabled, track availability reaches **90.65%**.

The partial-update mechanism therefore partially compensates for the loss in availability caused by partial initialization. However, the estimation quality remains significantly degraded compared with the configuration without partial initialization.

In particular, the probabilistic metrics show a substantial increase:

- mean Mahalanobis error: **12.79 → 23.47**;
- mean NLL: **9.94 → 15.40**.

The combined configuration therefore confirms, that improving track availability alone is not sufficient. A tracking strategy must also maintain a sufficiently accurate and statistically consistent state estimate. 

---

## 6. Discussion

Overall, the configuration A standard EKF provides reasonably good tracking performance. The estimation errors remain relatively close to the intrinsic measurement resolution, indicating that the achievable accuracy is already partly constrained by the available measurements.

However, the results also reveal significant room for improvement, particularly in terms of **track availability and robustness in challenging situations**. Given the high proportion of measurements containing usable information in the dataset, the tracking solution could theoretically achieve an availability of at least **98.67%** under ideal conditions, compared with the current availability of around 91%. In addition, while the average estimation accuracy is already reasonably good, the higher errors observed in the tail of the distribution indicate that some edge cases are still not handled robustly. The main challenge is therefore no longer to obtain a generally effective tracker, but to further refine its behavior and improve its robustness in more difficult situations.

The results nevertheless highlight a clear difference between partial updates and partial initialization. **Partial updates improve track availability while maintaining broadly comparable estimation accuracy**, indicating that the EKF can make effective use of incomplete measurements without significantly compromising tracking performance.

In contrast, **partial initialization currently degrades the overall tracking performance**. The resulting initial uncertainty can lead the EKF's local linearization to become unreliable, potentially causing valid subsequent measurements to be rejected. As a result, using a partial measurement for initialization can be more detrimental than simply ignoring it. This highlights the need for a dedicated approach to robust partial initialization.