
# UKF System Dependency Graph

This diagram outlines the structure of the UKF state estimation system. It captures the core components,
how they interact, and what responsibilities each module holds.

## 📌 Overview

- **main.cpp**: Entry point. Initializes and drives the UKF.
- **UKF.hpp / UKF.cpp**: Core implementation of the Unscented Kalman Filter (UKF). Handles predict/update.
- **estimator_interface.hpp**: Abstract interface all estimators implement.
- **ukf_defs.hpp**: Central types, constants, and dynamics functions.
- **logger.hpp**: Minimal binary logger with CRC-16. Inspired by CAN log streams.
- **logger_conversions.hpp**: Converts Eigen vectors to loggable float arrays.
- **measurement_handler.hpp**: Streams and smooths IMU data.
- **measurement_model.hpp**: Streams ground-truth data as simulated sensor measurements.
- **thread_safe_queue.hpp**: Used by both handlers for buffering.

## 🧭 Rendering the Graph

To render the `.dot` file:

### Option 1: Generate PDF or PNG locally

```bash
dot -Tpdf ukf_dependency_graph.dot -o ukf_graph.pdf
dot -Tpng ukf_dependency_graph.dot -o ukf_graph.png
```

You must have Graphviz installed (`brew install graphviz` or `sudo apt install graphviz`).

### Option 2: Embed in MkDocs (if using Mermaid plugin)

Convert to Mermaid if desired, or host the PNG output and reference via `![UKF Graph](img/ukf_graph.png)`.

## 📂 Files

- [`ukf_dependency_graph.dot`](./ukf_dependency_graph.dot) — Graphviz source
- `ukf_graph.pdf` or `ukf_graph.png` — Optional renderings for visual inspection

---

## 🔗 Relationships Summary

```text
main.cpp
  └─> UKF
        ├─> estimator_interface
        ├─> ukf_defs
        ├─> measurement_handler
        ├─> measurement_model
        ├─> logger
        └─> logger_conversions
measurement_handler + measurement_model
  └─> use thread_safe_queue
```
