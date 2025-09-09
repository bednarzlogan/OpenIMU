#pragma once

#include <atomic>
#include <cstdint>
#include <Eigen/Dense>
#include <string>
#include <sys/types.h>
#include <thread>

#include "estimator_interface.hpp"
#include "IMU_Matrices.hpp"
#include "thread_safe_queue.hpp"
#include "ukf_defs.hpp"


// using N and M for adaptive sizing depending on model without heap allocations
class UKF : public Estimator {
public:
    /// NOTE: use of an 8 bit integer limits the number of sigma points to 255
    static constexpr uint8_t NumSigma = 2 * N + 1; // minimum sample from UKF theory

    // alternate constructor for default initialization
    UKF(const std::string& configs_path);
    ~UKF() {
        stop_filter();
    }

    // sets params for system and gets measurement file path
    void read_configs(std::ifstream& inFile);

    // kicks off processing loop
    void start_filter(std::chrono::milliseconds period);
    void stop_filter();

    // kickoff for sim mode processing
    void read_imu(ImuData imu_measurement);
    void read_gps(Observable observable_measurement);

    /** inherited functions from the estimator class **/
    void initialize(const StateVec& initial_state, const CovMat& initial_covariance);

    void predict(const ControlInput& u, double dt);

    void update(const MeasVec& z, const MeasCov& R);

    // getters for state and covariance
    const StateVec& get_state() const;
    const CovMat& get_covariance() const;

private:
    // config
    UKFParams _params;
    std::string _measurement_file_path;
    std::string _ground_truth_path;

    // UKF scaling parameters
    double _alpha, _beta, _kappa, _lambda, _gamma;

    // weight params
    Eigen::Matrix<double, NumSigma, 1> _Wm;
    Eigen::Matrix<double, NumSigma, 1> _Wc;

    // state variables
    CovMat _P;
    StateVec _x;

    // process noise matrix
    CovMat _Q;

    // current sigma points
    SigmaPointArray _sigma_points;

    // queues for holding incoming measurements
    std::unique_ptr<ThreadQueue<ImuData>> _imu_queue;
    std::unique_ptr<ThreadQueue<Observable>> _gnss_queue;

    // resources for managing the worker thread
    std::atomic<bool> stop_flag_{false};
    std::thread worker_;
    std::condition_variable cv_;
    std::mutex cv_mtx_;

    // for maintaining retrodiction strategy
    struct alignas(32) Snapshot {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        StateVec x;
        CovMat   P;
        ControlInput u_from_prev;
        double t{};
    };
    using HistAlloc = Eigen::aligned_allocator<Snapshot>;
    boost::circular_buffer<Snapshot, HistAlloc> hist;
    uint8_t hist_cap{50}; 

    // time bookkeeping
    double _solution_time{0.0};
    bool _initialized_time{false};
    double _max_wait_time{1}; // max time to wait for a GNSS measurement before just processing IMU

    // helper function to generate sigma points
    void generate_sigma_points(const StateVec& mu, const CovMat& P, SigmaPointArray& sigma_points);

    /**
    * @brief Real-time UKF worker that merges high-rate IMU with delayed GNSS via fixed-lag
    *        retrodiction (rollback -> update -> replay).
    *
    * @param period  Target wall-clock cadence for the loop’s sleep/wake cycle.
    *
    * @details
    * This loop consumes IMU and GNSS from lock-free producer queues without blocking and
    * maintains a short, fixed-capacity history of filter snapshots (`hist`). A GNSS update
    * is only applied once the history *tip* has passed the GNSS timestamp (“rising-edge”
    * eligibility). At that moment we:
    *   1) drain and process all IMU with t ≤ t_gnss,
    *   2) rollback the filter state to the last snapshot at/just before t_gnss,
    *   3) (if needed) propagate exactly to t_gnss using the control that led to the next snapshot,
    *   4) apply the GNSS measurement update at t_gnss, and
    *   5) replay forward to the current history tip, overwriting stored posteriors.
    *
    * ### Core policy (invariants)
    * - **Eligibility:** GNSS is processed iff `hist.front().t ≤ t_gnss ≤ hist.back().t` and there is
    *   no earlier staged IMU (`t_imu < t_gnss`). This guarantees all IMU up to t_gnss have been folded in.
    * - **No forward-only updates:** We never update at t_gnss by “predicting past missing IMU.”
    *   If t_gnss is outside the history window, we do **not** apply the measurement.
    * - **Stale-GNSS handling:** GNSS older than `hist.front().t` is dropped (cannot be retrodicted).
    *
    * ### Arbitration & draining
    * - If a GNSS is eligible, we first drain IMU ≤ t_gnss (using non-blocking pops) to ensure coverage,
    *   then perform rollback→update→replay.
    * - Otherwise, we process the next IMU if available; if nothing is actionable, we sleep until the
    *   next tick or a producer notification.
    *
    * ### Timing / numerics
    * - Timestamps are seconds; comparisons use a small epsilon `TIME_EPS` to avoid jitter issues.
    * - `safe_dt(a,b)` clamps tiny/negative deltas to keep the filter numerically well-posed.
    *
    * ### Concurrency model
    * - Single consumer thread (this worker) + multiple producers (`read_imu`, `read_gps`).
    * - Non-blocking `try_pop` for staging; a condition_variable is used only for sleep/wake.
    *
    * ### Memory & footprint
    * - `hist` is a fixed-capacity circular buffer of `Snapshot{t, x, P, u_from_prev}`; no dynamic
    *   growth during runtime. GNSS is staged in a single optional.
    *
    * ### Result
    * - Low-latency propagation on IMU, measurement-time-correct updates for GNSS, and deterministic
    *   behavior in the presence of delayed/out-of-order GNSS within the retained lag window.
    *
    * @note  This is the minimal “readable first pass.” Timeout/wait-for-post-IMU interpolation,
    *        metrics, and back-pressure policies can be added later without changing the core logic.
    */
    void worker_loop(std::chrono::milliseconds period);
};