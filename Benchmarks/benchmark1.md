# Benchmark 01: Partial vs Strict Initialization (Ablation Study)

## 1. Context and Objective
The Extended Kalman Filter (EKF) can theoretically initialize a track using partial measurements (e.g., Range + Doppler only) by generating a very large initial covariance matrix for the missing dimensions using a Monte Carlo method (or UKF).  
The objective of this benchmark is to evaluate the impact of exploiting partial measurements on the filter's overall performance, particularly in terms of Track Availability and Estimated State Error.

## 2. Methodology (Ablation Study)
To avoid having too few partial initialization cases, which could lead to a comparison based on a limited and potentially unrepresentative sample, as well as to avoid survivor bias and ensure a 100% fair comparison, we apply the following procedure:
*   *Artificial Dataset Degradation:* The first radar contact of each scenario has its angles (Azimuth, Elevation) removed.
*   **Partial Mode:** The tracker is allowed to initialize using this incomplete measurement.
*   **Full Mode:** The tracker receives a `NaN` and must wait for the next instant (complete measurement) to initialize.
*   *Evaluation Metric:* The mean Euclidean error is calculated **only** over the valid temporal intersection of the two methods (common mask).

## 3. Baseline Results (Standard EKF)
| Metric | Full Init | Partial Init |
| :--- | :--- | :--- |
| **Position Error** | 1.83 m (95% < 5.30 m) | 1.93 m (95% < 5.54 m) |
| **Velocity Error** | 2.57 m/s (95% < 8.35 m/s) | 2.74 m/s (95% < 8.74 m/s) |

## 4. Analysis and Problem Statement
Contrary to Bayesian intuition (where degraded information is better than no information), partial initialization degrades the final accuracy.  
**Explanation:** The initial covariance matrix ($P_{init}$) generated from the partial measurement is extremely large along the local transverse axes. When the complete measurement arrives at time $t+1$, although the Kalman gain ($K$) assigns a high weight to the new measurement, the update equation retains a residual weighting of this initial covariance. Moreover, the initial covariance matrix is generated using a Monte Carlo method, avoiding any inappropriate linear approximation given the large measurement uncertainties. This confirms that the observed loss in accuracy is inherent to the filter's behavior rather than being caused by a flawed initialization methodology.

## 5. Proposed Solution: Conditional Reset (Track Promotion)
To address this limitation of the EKF's memory, we implement a reset heuristic ("Track Reset").  
During the Update step, we compare the uncertainty of the prediction with that of the incoming measurement:
`If Trace(R) < threshold * Trace(P_pred)` 
Then the filter does not perform a weighted update: it overwrites the current state and resets purely based on the new measurement, promoting the "Tentative" track to a "Confirmed" track.

## 6. Results with Heuristic