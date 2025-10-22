#include <atomic>
#include <cassert>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>

#include "UKF.hpp"
#include "safe_cholesky.hpp"

using json = nlohmann::json;

// TMP
static std::ofstream debug_log("worker_loop_debug.csv", std::ios::app);

void UKF::read_configs(std::ifstream &inFile) {
  UKFParams hold_config;

  // access thre required members
  json j;
  inFile >> j; // streams contents of the config file into 'j'

  // we can now access the features of the configs like a map or dict
  /// NOTE: the UKF only uses a subset of these
  hold_config.gx = j["gx"];
  hold_config.gy = j["gy"];
  hold_config.gz = j["gz"];

  // FOGMP process on biases config
  hold_config.tau_a = j["tau_ax"];
  hold_config.tau_g = j["tau_gx"];
  hold_config.sig_a = j["sig_ax"];
  hold_config.sig_g = j["sig_gx"];

  // scaling params for UKF
  _alpha = j["alpha"];
  _beta = j["beta"];
  _kappa = j["kappa"];

  // example printouts
  std::cout << "Configuration loaded successfully.\n";
  std::cout << "Accelerometer time constant (tau_a): " << hold_config.tau_a
            << "\n";
  std::cout << "Gyro time constant (tau_g): " << hold_config.tau_g << "\n";
  _params = hold_config; // assign configurations
}

UKF::UKF(const std::string &configs_path) {
  // read in configurables
  // load in the system configurations
  std::ifstream inFile(configs_path);
  if (!inFile.is_open()) {
    std::cerr << "Could not open config file: " << configs_path << std::endl;
  }

  // allocate the measurement file path and read in UKF params
  read_configs(inFile);

  // calculate scaling params
  double lambda_raw = _alpha * _alpha * (N + _kappa) - N;
  _lambda = std::max(lambda_raw, -0.95 * N);
  _gamma = std::sqrt(N + _lambda);

  // weights for sigma point mean and covariance
  _Wm.setConstant(0.5 / (N + _lambda));
  _Wc = _Wm; // copy

  _Wm(0) = _lambda / (N + _lambda);
  _Wc(0) = _Wm(0) + (1 - pow(_alpha, 2) + _beta);

  // TMP PROCESS NOISE
  _Q.setIdentity();
  _Q *= 1e-3;

  ukf_log() << "[UKF] _lambda = " << _lambda << ", _gamma = " << _gamma << "\n";

  // create the queues for holding incoming measurements
  _imu_queue = std::make_unique<ThreadQueue<ImuData>>(1000);
  _gnss_queue = std::make_unique<ThreadQueue<Observable>>(1000);

  // retrodiction queue sizing
  hist.set_capacity(hist_cap);
}

void UKF::start_filter(std::chrono::milliseconds period) {
  if (worker_.joinable())
    return; // already running
  stop_flag_.store(false, std::memory_order_relaxed);
  worker_ = std::thread(&UKF::worker_loop, this, period);
}

void UKF::stop_filter() {
  if (!worker_.joinable())
    return;
  stop_flag_.store(true, std::memory_order_relaxed);
  cv_.notify_all(); // wake the worker if it's waiting
  worker_.join();
}

void UKF::read_imu(ImuData imu_measurement) {
  // wake the worker quickly when new data arrives
  _imu_queue->push(imu_measurement);
  cv_.notify_one();
}

void UKF::read_gps(Observable observable_measurement) {
  // signal to the filter that we're ready to process GNSS data
  _gnss_queue->push(observable_measurement);
  cv_.notify_one();
}

void UKF::worker_loop(std::chrono::milliseconds period) {
  /**
   * Here, we manage time-alignment between the solutions from coasting INS and
   * incoming GNSS measurements. The logic is as follows:
   * 1. Check all queues for the oldest available measurement
   * 2. If it's an IMU measurement, add it to a sub queue
   * 3. If it's a GNSS measurement, process all IMU measurements up to that
   * time, propagate to GNSS time, then do a measurement update
   * 4. If too much time has passed without a GNSS measurement, just process all
   * IMU measurements in the queue
   * 5. Repeat
   */
  using clock = std::chrono::steady_clock;
  auto next_wake = clock::now() + period;
  constexpr double TIME_EPS = 1e-6;

  std::optional<ImuData> imu_next;
  std::optional<Observable> gnss_next;

  auto safe_dt = [](double t_now, double t_prev) {
    double dt = t_now - t_prev;
    if (dt < 1e-6)
      dt = 1e-6; // avoid zero/negative
    return dt;
  };

  // --- small helper: find last snapshot with t <= tz (reverse scan; hist is
  // short) ---
  auto find_last_leq = [&](double tz) -> int {
    if (hist.empty())
      return -1;
    for (int i = static_cast<int>(hist.size()) - 1; i >= 0; --i) {
      if (hist.at(static_cast<size_t>(i)).t <= tz)
        return i;
    }
    return -1;
  };

  auto record_snapshot = [&](double t_now, const ControlInput &u) {
    // snapshot current state; assumes _x/_P are UKF’s current posterior after
    // predict/update
    hist.push_back(Snapshot{_x, _P, u, t_now});
  };

  auto process_imu = [&](const ImuData &m) {
    if (!_initialized_time) {
      _solution_time = m.measurement_time;
      _initialized_time = true;
    }
    const double dt = safe_dt(m.measurement_time, _solution_time);
    predict(m.matrix_form_measurement, dt);
    _solution_time = m.measurement_time;
    record_snapshot(_solution_time, m.matrix_form_measurement);
  };

  auto process_gnss = [&](const Observable &z) {
    // if the GNSS time is within our history window, do rollback + replay.
    // (by construction of gnss_is_next, hist is non-empty and front.t <= z.t <=
    // back.t)
    if (z.timestamp <= hist.back().t + TIME_EPS) {
      const int k = find_last_leq(z.timestamp);
      if (k >= 0) {
        // expose extent of retrodiction for testing
        _last_replay_steps = k;

        // rollback to snapshot k
        _x = hist.at(static_cast<size_t>(k)).x;
        _P = hist.at(static_cast<size_t>(k)).P;
        double t = hist.at(static_cast<size_t>(k)).t;

        // expose ms delta between last IMU and GNSS solutions from hist
        _last_retrodict_depth_ms = hist.back().t - z.timestamp;

        // propagate exactly to GNSS time (ZOH with the next step’s control)
        if (z.timestamp > t + TIME_EPS) {
          // k+1 must exist if tz < hist.back().t
          ControlInput u = hist.at(static_cast<size_t>(k + 1)).u_from_prev;
          const double dt = safe_dt(z.timestamp, t);
          predict(u, dt);
          t = z.timestamp;
        }

        // update at GNSS time
        update(z.observation, z.R);

        // replay snapshots k+1..end, overwriting their posteriors
        double t_cur = z.timestamp; // we've just updated at t_z
        for (size_t i = static_cast<size_t>(k + 1); i < hist.size(); ++i) {
          const double dt_step = std::max(1e-6, hist.at(i).t - t_cur);
          predict(hist.at(i).u_from_prev, dt_step);
          t_cur = hist.at(i).t;
          hist.at(i).x = _x;
          hist.at(i).P = _P;
        }

        // bring _solution_time to the tip
        _solution_time = hist.back().t;
        return; // done
      }
      // if tz < hist.front().t, we fall through to “forward-only” handling
      // below.
    }

    // we should never reach here since we've required a valid solution history
    // that surrounds the GNSS measurement
    assert(false && "process_gnss: reached forward-only path unexpectedly");
    return;
  };

  for (;;) {
    // fill holds if empty (non-blocking)
    if (!imu_next) {
      ImuData m;
      if (_imu_queue->try_pop(m))
        imu_next = std::move(m);
    }
    if (!gnss_next) {
      Observable g;
      if (_gnss_queue->try_pop(g))
        gnss_next = std::move(g);
    }

    // nothing to do = wait
    if (!imu_next && !gnss_next) {
      // ensure we're still supposed to be running
      if (stop_flag_.load(std::memory_order_relaxed))
        break;

      // go to sleep until either:
      //    a) the next wake time hits, or
      //    b) someone notifies us because new data was pushed, or
      //    c) stop flag becomes true
      std::unique_lock<std::mutex> lk(cv_mtx_);
      cv_.wait_until(lk, next_wake, [&] {
        return stop_flag_.load(std::memory_order_relaxed) ||
               !_imu_queue->empty() || !_gnss_queue->empty();
      });
      const auto now = clock::now();
      next_wake = now + period;
      continue;
    }

    // decide which timestamp to process next
    bool gnss_is_next = false;
    if (gnss_next) {
      const double tz = gnss_next->timestamp;

      // toss GNSS that fell behind our retained history window
      const bool too_old = !hist.empty() && (tz + TIME_EPS < hist.front().t);
      if (too_old) {
        // TODO - we might want to log this event
        gnss_next.reset(); // drop it and move on
        _stale_gnss_drop_count += 1;
      } else {
        // coverage: can we rollback to tz?  (front <= tz <= back)
        const bool have_history = !hist.empty();
        const bool tz_within_hist = have_history &&
                                    (hist.front().t <= tz + TIME_EPS) &&
                                    (tz <= hist.back().t + TIME_EPS);

        // arbitration: don't skip an IMU that's earlier than tz
        const bool imu_earlier =
            (imu_next && (imu_next->measurement_time + TIME_EPS < tz));

        gnss_is_next = tz_within_hist && !imu_earlier;
      }
    }

    if (gnss_is_next) {
      // first consume all IMU up to the GNSS time
      while (imu_next &&
             imu_next->measurement_time <= gnss_next->timestamp + TIME_EPS) {
        process_imu(*imu_next);
        imu_next.reset();
        ImuData m;
        if (_imu_queue->try_pop(m))
          imu_next = std::move(m);
      }
      // then apply the GNSS update (with ZOH catch-up or rollback+replay as
      // needed)
      process_gnss(*gnss_next);
      _last_gnss_time = gnss_next->timestamp;
      gnss_next.reset();
    } else {
      // IMU is earlier = process it (or try to stage one)
      if (!imu_next) {
        ImuData m;
        if (_imu_queue->try_pop(m))
          imu_next = std::move(m);
      }
      if (imu_next) {
        process_imu(*imu_next);
        imu_next.reset();
      }
    }

    if (stop_flag_.load(std::memory_order_relaxed))
      break;

    // keep a steady cadence
    const auto now = clock::now();
    if (now >= next_wake)
      next_wake = now + period;
    else
      next_wake += period;
  }
}

const StateVec &UKF::get_state() const { return _x; }

const CovMat &UKF::get_covariance() const { return _P; }

void UKF::initialize(const StateVec &initial_state,
                     const CovMat &initial_covariance) {
  _x = initial_state;
  _P = initial_covariance;
  _solution_time = 0;
}

void UKF::generate_sigma_points(const StateVec &mu, const CovMat &P,
                                SigmaPointArray &sigma_points) {

  // tmp logs
  ukf_log() << "Initial covariance diag: " << P.diagonal().transpose() << "\n";
  ukf_log() << "Initial state mu: " << mu.transpose() << "\n";

  // scale the covariance matrix
  CovMat scaled_P = (N + _lambda) * P;

  // ensure the matrix is symmetric and positive definite
  // the function will also perform the cholesky decomp after these steps are
  // taken
  CovMat S;
  bool success = safe_cholesky(scaled_P, S);

  if (!success) {
    std::cerr << "[UKF] ERROR: Failed to compute Cholesky decomposition "
                 "despite stabilization attempts.\n";
    std::cerr << "[UKF] Matrix was:\n" << scaled_P << "\n";
    throw std::runtime_error("UKF: Cholesky decomposition failed");
  }

  // assign first sigma point
  sigma_points[0] = mu;

  // generate symmetric sigma points
  for (uint8_t i = 0; i < N; ++i) {
    sigma_points[i + 1] = mu + _gamma * S.col(i);
    sigma_points[N + i + 1] = mu - _gamma * S.col(i);
  }

  ukf_log() << "[UKF] Raw P:\n" << P << "\n";
  ukf_log() << "[UKF] Scaled P:\n" << scaled_P << "\n";
}

void UKF::predict(const ControlInput &u, double dt) {

  // tmp debug printouts
  // std::cout << "Processing control input:\n"
  //           << u << "\n with dt " << dt << std::endl;

  // use the nomlinear dynamics to propagate the sigma points into predicted
  // states
  SigmaPointArray sigma_points;
  generate_sigma_points(_x, _P, sigma_points); // get new sigma points

  // propagate each sigma point through the nonlinear dynamics using rk4
  SigmaPointArray propagated_sigmas;
  for (uint8_t i = 0; i < NumSigma; ++i) {
    propagated_sigmas[i] = rk4_step(sigma_points[i], u, dt, _params);
  }

  // compute predicted mean and covariance
  StateVec mu_pred;
  mu_pred.setZero();
  for (uint8_t i = 0; i < NumSigma; ++i) {
    mu_pred += _Wm[i] * propagated_sigmas[i];
  }

  CovMat P_pred;
  P_pred = _Q; // covariance update is of the form var(x) + Q
  for (uint8_t i = 0; i < NumSigma; ++i) {
    // "measure" the covariance based on deviations in the sigma points from the
    // mean
    StateVec dx = propagated_sigmas[i] - mu_pred;
    P_pred += _Wc[i] * (dx * dx.transpose());
  }

  // update state & covariance
  _x = mu_pred;
  _P = P_pred;
  _sigma_points = propagated_sigmas;

  // debug logging
  if (debug_log.is_open()) {
    debug_log << "Current state:\n"
              << _x.transpose() << "\nLast control input:\n"
              << u.transpose() << std::endl;
  }
}

void UKF::update(const MeasVec &z, const MeasCov &R) {
  // tmp debug printouts
  std::cout << "Processing measurement input: " << z << std::endl;

  // propagate through measurement model
  std::array<MeasVec, NumSigma> z_sigma;
  for (uint8_t i = 0; i < NumSigma; ++i) {
    z_sigma[i] = _sigma_points[i].head<Z>(); // observing position + velocity
  }

  // predicted measurement mean
  MeasVec z_pred = MeasVec::Zero();
  for (uint8_t i = 0; i < NumSigma; ++i) {
    z_pred += _Wm[i] * z_sigma[i];
  }

  // log resudial in this prediction
  Eigen::Matrix<double, Z, 1> residual = z - z_pred;

  // innovation covariance and cross-covariance
  MeasCov S = R;
  Eigen::Matrix<double, N, Z> P_xz = Eigen::Matrix<double, N, Z>::Zero();
  for (uint8_t i = 0; i < NumSigma; ++i) {
    MeasVec dz = z_sigma[i] - z_pred;
    StateVec dx = _sigma_points[i] - _x;
    S += _Wc[i] * dz * dz.transpose();
    P_xz += _Wc[i] * dx * dz.transpose();
  }

  // Kalman gain
  Eigen::Matrix<double, N, Z> K = P_xz * S.inverse();

  // state update
  _x += K * (z - z_pred);
  _P -= K * S * K.transpose();
}
