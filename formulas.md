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

## 1. Formula

The Mahalanobis distance is defined as

`D² = (z_meas - z_pred)^T S⁻¹ (z_meas - z_pred)`

where

- `z_meas` is the measurement.
- `z_pred` is the predicted measurement.
- `S` is the innovation covariance matrix.

The innovation is

`ν = z_meas - z_pred`

---

## 2. Intuition

Unlike the Euclidean distance, the Mahalanobis distance evaluates the **consistency** of the innovation with respect to its expected uncertainty.

A given innovation may be:
- perfectly acceptable if the measurement uncertainty is large,
- highly unlikely if the measurement uncertainty is small.

Therefore, the Mahalanobis distance is a **relative** distance, whereas the Euclidean distance is an **absolute** distance.

---

## 3. Role of the Innovation Covariance

The innovation covariance matrix

`S = H P H^T + R`

describes the expected uncertainty of the innovation.

Its inverse `S⁻¹` weights each direction according to its uncertainty:

- **large variance** → small penalty,
- **small variance** → large penalty.

If `S = I`, the Mahalanobis distance reduces to the standard Euclidean distance.

---

## 4. Geometric Interpretation

The expression

`D² = ν^T S⁻¹ ν`

is a **quadratic form**.

Since `S⁻¹` is symmetric positive definite,

`D = √(ν^T S⁻¹ ν)`

defines a norm.

Instead of measuring distances with circles (Euclidean geometry), the Mahalanobis distance measures distances with ellipsoids whose shape is determined by the covariance matrix.

---

## 5. Gating

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

---

# Converted Measurement Kalman Filter (CMKF) - Equations Reference

## 1. Specific Vectors & Matrices
* **Converted Measurement $Z_{cart}$ (3x1):** $[x_{meas}, y_{meas}, z_{meas}]^T$ obtained via Spherical-to-Cartesian conversion.
* **Doppler Measurement $Z_{vel}$ (1x1):** $[v_{rad}]$
* **Linear Observation Matrix $H_{lin}$ (3x6):** $\begin{bmatrix} I_{3\times3} & 0_{3\times3} \end{bmatrix}$ (Extracts only Cartesian positions).
* **Converted Noise Covariance $R_{cart}$ (3x3):** Cartesian projection of the polar position noise.
* **Conversion Jacobian $J_{s2c}$ (3x3):** Partial derivatives of the Spherical-to-Cartesian transformation evaluated at the measurement.

---

## 2. Stage 1: Linear Position Update
Transforms the measurement to Cartesian space to perform a purely linear Kalman update.

**Converted Measurement & Covariance:**
$$ Z_{cart} = \text{Sphe2Cart}(r, \theta, \phi) $$
$$ R_{cart} = J_{s2c} R_{pos} J_{s2c}^T $$

**Linear Innovation & Gain:**
$$ Y_{cart} = Z_{cart} - H_{lin} \hat{X}_{k|k-1} $$
$$ S_{cart} = H_{lin} P_{k|k-1} H_{lin}^T + R_{cart} $$
$$ K_{cart} = P_{k|k-1} H_{lin}^T S_{cart}^{-1} $$

**Intermediate State & Covariance Update:**
$$ \hat{X}_{int} = \hat{X}_{k|k-1} + K_{cart} Y_{cart} $$
$$ P_{int} = (I - K_{cart} H_{lin}) P_{k|k-1} $$

> **Note - Velocity Correction:** Although $H_{lin}$ zeroes out the velocity components of the measurement, the velocity state is still corrected during this stage thanks to the position-velocity cross-correlations implicitly stored in $P_{k|k-1}$.

---

## 3. Stage 2: Non-Linear Doppler Update
Updates the filter using the radial velocity measurement. Crucially, the non-linear Jacobian is evaluated around the newly corrected intermediate state $\hat{X}_{int}$.

**Non-Linear Innovation (Doppler only):**
$$ Y_{vel} = v_{rad} - h_{vrad}(\hat{X}_{int}) $$

**Jacobian & Innovation Covariance:**
$$ H_{vrad} = \left. \frac{\partial h_{vrad}}{\partial X} \right|_{X = \hat{X}_{int}} \quad \text{(Size: 1x6)} $$
$$ S_{vel} = H_{vrad} P_{int} H_{vrad}^T + R_{vrad} $$

**Final State & Covariance Update:**
$$ K_{vel} = P_{int} H_{vrad}^T S_{vel}^{-1} $$
$$ \hat{X}_{k|k} = \hat{X}_{int} + K_{vel} Y_{vel} $$
$$ P_{k|k} = (I - K_{vel} H_{vrad}) P_{int} $$

> **Note - Partial Measurements Constraint:** The Spherical-to-Cartesian conversion inherently requires angles (Azimuth and Elevation). If the incoming measurement is partial (e.g., Range/Doppler only), the CMKF architecture cannot be applied. The pipeline must dynamically fall back to the Standard EKF update for these specific measurements.

---

# Unscented Kalman Filter (UKF) - Equations Reference

> **Note - Core Concept:** The UKF entirely discards Jacobians and linearization. It uses the Unscented Transform (UT) to deterministically sample the probability distribution (Sigma Points) and pass them through the true non-linear measurement function. This directly overcomes Jensen's inequality ($E[h(X)] \neq h(E[X])$) and accurately captures the posterior mean and covariance to the 3rd order.

## 1. UT Parameters & Sigma Points Generation
The state space of dimension $N$ is represented by $2N+1$ deterministically chosen Sigma Points.

**UT Scaling Parameters:**
* $N$: State dimension (6 for 3D position & velocity)
* $\lambda$: Scaling parameter defining the spread of the points ($\lambda = \alpha^2(N+\kappa)-N$)
* $\gamma = \sqrt{N+\lambda}$: Covariance multiplier

**Weights Formulation:**
$$W_0^{(m)} = \frac{\lambda}{N+\lambda} \quad \text{(Mean weight for the central point)}$$
$$W_0^{(c)} = \frac{\lambda}{N+\lambda} + (1 - \alpha^2 + \beta) \quad \text{(Covariance weight for the central point)}$$
$$W_i^{(m)} = W_i^{(c)} = \frac{1}{2(N+\lambda)} \quad \text{for } i = 1, \dots, 2N \quad \text{(Weights for the peripheral points)}$$

**Sigma Points Creation ($\mathcal{X}_i$):**

A matrix $L$ is derived from the Cholesky decomposition of the predicted covariance: $L = \text{cholesky}(P_{k|k-1})$ with $L L^T = P_{k|k-1}$
$$\mathcal{X}_0 = \hat{X}_{k|k-1}$$
$$\mathcal{X}_i = \hat{X}_{k|k-1} + \gamma L_i \quad \text{for } i = 1, \dots, N$$
$$\mathcal{X}_{i+N} = \hat{X}_{k|k-1} - \gamma L_i \quad \text{for } i = 1, \dots, N$$
*(Where $L_i$ is the $i$-th column of the matrix $L$)*

---

## 2. Measurement Projection (Non-Linear)
Each Sigma Point is projected through the non-linear measurement function $h(X)$.

**Projected Sigma Points ($\mathcal{Z}_i$):**
$$\mathcal{Z}_i = h(\mathcal{X}_i) \quad \text{for } i = 0, \dots, 2N$$

**Predicted Measurement Weighted Mean:**
$$\hat{Z}_{k} = \sum_{i=0}^{2N} W_i^{(m)} \mathcal{Z}_i$$

---

## 3. Update Phase (Jacobian-Free)
Replaces the standard $H P H^T$ and $P H^T$ matrices with cross-correlations of the Sigma Points.

**Measurement Innovation:**
$$Y_k = Z_{meas} - \hat{Z}_{k}$$

**Innovation Covariance ($S_k$):**
$$S_k = \sum_{i=0}^{2N} W_i^{(c)} (\mathcal{Z}_i - \hat{Z}_{k})(\mathcal{Z}_i - \hat{Z}_{k})^T + R$$

**State-Measurement Cross-Covariance ($P_{xz}$):**
$$P_{xz} = \sum_{i=0}^{2N} W_i^{(c)} (\mathcal{X}_i - \hat{X}_{k|k-1})(\mathcal{Z}_i - \hat{Z}_{k})^T$$

**Kalman Gain & Final Update:**
$$K_k = P_{xz} S_k^{-1}$$
$$\hat{X}_{k|k} = \hat{X}_{k|k-1} + K_k Y_k$$
$$P_{k|k} = P_{k|k-1} - K_k S_k K_k^T$$

---

> **Note - Covariance Update Formula:** Since the matrix $H$ is never explicitly computed, the classical EKF covariance update equation $P = (I - KH)P$ cannot be used. The equivalent, stable formulation $P = P - K S K^T$ is utilized instead.