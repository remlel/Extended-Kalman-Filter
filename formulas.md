# Extended Kalman Filter (EKF) - Equations Reference

## 1. Vectors & Matrices Definition
* **State Vector $X$ (6x1):** $[x, y, z, v_x, v_y, v_z]^T$
* **Measurement Vector $Z$ (4x1):** $[r, \dot{r}, \theta, \phi]^T$ (Range, Doppler, Azimuth, Elevation)
* **Transition Matrix $F$ (6x6):** Linear kinematic model (State transition)
* **Process Noise Covariance $Q$ (6x6):** Uncertainty in the kinematic model (e.g., accelerations)
* **Measurement Function $h(X)$:** Non-linear mapping from Cartesian state to Spherical measurement
* **Jacobian Matrix $H$ (4x6):** Partial derivatives of $h(X)$ with respect to $X$
* **Measurement Noise Covariance $R$ (4x4):** Radar measurement uncertainties (variances)

---

## 2. Predict Phase (Time Update)
Advances the state and covariance matrices to the current time step.

**State Prediction:**
$$ \hat{X}_{k|k-1} = F \hat{X}_{k-1|k-1} $$

**Covariance Prediction:**
$$ P_{k|k-1} = F P_{k-1|k-1} F^T + Q $$

---

## 3. Update Phase (Measurement Update)
Corrects the prediction using the new radar measurement.

**Innovation (Measurement Residual):**
$$ Y_k = Z_k - h(\hat{X}_{k|k-1}) $$

**Innovation Covariance:**
$$ S_k = H_k P_{k|k-1} H_k^T + R $$

**Kalman Gain:**
$$ K_k = P_{k|k-1} H_k^T S_k^{-1} $$

**State Update:**
$$ \hat{X}_{k|k} = \hat{X}_{k|k-1} + K_k Y_k $$

**Covariance Update:**
$$ P_{k|k} = (I - K_k H_k) P_{k|k-1} $$

---

# Mahalanobis Distance

## Formula

The Mahalanobis distance is defined as

`D² = (z_meas - z_pred)^T S⁻¹ (z_meas - z_pred)`

where

- `z_meas` is the measurement.
- `z_pred` is the predicted measurement.
- `S` is the innovation covariance matrix.

The innovation is

`ν = z_meas - z_pred`

---

## Intuition

Unlike the Euclidean distance, the Mahalanobis distance evaluates the **consistency** of the innovation with respect to its expected uncertainty.

A given innovation may be:
- perfectly acceptable if the measurement uncertainty is large,
- highly unlikely if the measurement uncertainty is small.

Therefore, the Mahalanobis distance is a **relative** distance, whereas the Euclidean distance is an **absolute** distance.

---

## Role of the Innovation Covariance

The innovation covariance matrix

`S = H P H^T + R`

describes the expected uncertainty of the innovation.

Its inverse `S⁻¹` weights each direction according to its uncertainty:

- **large variance** → small penalty,
- **small variance** → large penalty.

If `S = I`, the Mahalanobis distance reduces to the standard Euclidean distance.

---

## Geometric Interpretation

The expression

`D² = ν^T S⁻¹ ν`

is a **quadratic form**.

Since `S⁻¹` is symmetric positive definite,

`D = √(ν^T S⁻¹ ν)`

defines a norm.

Instead of measuring distances with circles (Euclidean geometry), the Mahalanobis distance measures distances with ellipsoids whose shape is determined by the covariance matrix.

---

## Gating

The squared Mahalanobis distance `D²` is compared with a threshold `γ`.

Assuming Gaussian errors, `D²` follows a Chi-square distribution whose degrees of freedom correspond to the measurement dimension:

`D² ~ χ²(n)`

where `n` is the number of measurement components.

For a **4-dimensional measurement** (`Range`, `Radial Speed`, `Azimuth`, `Elevation`), the most common thresholds are:

| Confidence level | Threshold `γ` |
|-----------------:|--------------:|
| 95% | **9.49** |
| 99% | **13.28** |

The threshold therefore defines the size of the **validation gate** (an ellipsoid in the measurement space):

- **Small `γ`** → stricter gate, more measurements are rejected.
- **Large `γ`** → more permissive gate, more measurements are accepted.

If

`D² < γ`

the innovation is considered statistically consistent with the predicted uncertainty, and the measurement is accepted.

Otherwise, the innovation is considered too unlikely under the assumed Gaussian model, and the measurement is rejected.