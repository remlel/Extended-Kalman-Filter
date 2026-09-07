# Benchmark

## 1. Objective

The objective of the benchmarks is to evaluate and progressively improve the performance and robustness of the tracking solution.

The performance of a tracking filter is assessed using several complementary metrics:

- **Track availability**
- **Position estimation error**
- **Velocity estimation error**
- **Mahalanobis error**, to assess the consistency between the estimated state and the actual state with respect to the estimated uncertainty
- **Negative Log-Likelihood (NLL)**, to assess the quality of the uncertainty estimation and whether the model is overly confident or overly conservative

The objective is therefore to identify the tracking solution providing the **best overall balance between availability, estimation accuracy, and probabilistic consistency**.

A key working hypothesis explored throughout these benchmarks is that **valid partial measurements can provide useful information even when they carry a high level of uncertainty**. A partial measurement may still constrain the possible state and can therefore, in principle, provide more useful information than performing a pure prediction without any measurement update.

However, effectively exploiting partial measurements is not necessarily possible with a standard EKF without further adaptation. As investigated in the subsequent benchmarks, the use of partial measurements may require specific modifications to the tracking pipeline in order to fully benefit from the available information without degrading the overall tracking performance.

The benchmarks therefore progressively investigate whether and how the exploitation of partial measurements can contribute to improving the overall performance of the tracking solution.

---

## 2. Dataset

The benchmarks are performed on a dataset containing **100 scenarios** of 20 seconds each, with an update rate of 10Hz.

The dataset contains **95.25% complete measurements** (no missing values). Including partial measurements, **98.76% of the expected measurements contain usable information**.
Partial measurements, defined here as measurements containing range and Doppler information but missing azimuth and elevation, account for **1.13% of all measurements**. As a consequence, naturally occurring partial initializations are relatively limited.

To obtain sufficient statistical coverage for evaluating partial initialization, the first measurement of each scenario is deliberately degraded to create a controlled set of partial-initialization cases.

This artificial degradation is intended to provide a representative and controlled evaluation of the initialization mechanism rather than reproduce the exact natural distribution of measurement losses.

> **Note:** These figures characterize the measurement availability in the dataset and should not be interpreted as theoretical upper bounds on tracking availability.

---

## 3. Evaluation Modes

Two complementary evaluation modes are used throughout the benchmarks.

### 3.1 Evaluation Mode

In **Evaluation Mode**, each configuration is evaluated using **all states estimated by that configuration**.

This mode characterizes the overall behavior of each configuration, including its own track availability.

Because configurations may estimate different sets of states, the resulting error metrics are not necessarily directly comparable between configurations.

Evaluation Mode is therefore primarily used to assess the **global behavior of an individual configuration**.

### 3.2 Benchmark Mode

In **Benchmark Mode**, configurations are compared only on the **states commonly estimated by the configurations being compared**.

This provides a controlled comparison of estimation quality, independently of differences in track availability.

Benchmark Mode is therefore used for **fair quantitative comparisons between tracking solutions**.

In summary:

| Mode | Purpose |
|---|---|
| **Evaluation Mode** | Characterize the global performance of each configuration |
| **Benchmark Mode** | Fairly compare different configurations on common estimated states |

---

## 4. Benchmark Structure

The benchmarks are organized as follows:

### Benchmark 1 — Standard EKF Reference

Initial characterization of the current standard EKF and the impact of partial initialization and partial updates.

### Benchmark 2 — 

### Benchmark 3 — 

### Benchmark 4 — 

### ...