#pragma once

#include <array>
#include <atomic>
#include <string>

#include "estimator_interface.hpp"
#include "logger.hpp"
#include "thread_safe_queue.hpp"
#include "ukf_defs.hpp"

enum MeasurementType { IMU = 0, GNSS = 1, NO_MEASUREMENT = 2 };

/**
 * @brief A playback simulator that serves observations based on the timestamps
 * in the provided measurement files.
 */
class TimedPlaybackSim {
public:
  /**
   * @brief Constructs a TimedPlaybackSim object.
   *
   * @param config_path Path to the configurations for the simulator.
   */
  TimedPlaybackSim(const std::string &config_path);

  /**
   * @brief Starts the playback simulation.
   *
   * @param estimator The estimator to which the measurements will be sent.
   */
  void start_simulation(std::shared_ptr<Estimator> estimator,
                        std::chrono::milliseconds period);

  /**
   * @brief Stops the playback simulation.
   */
  void stop_simulation();

  /**
   * @brief Gets the next observation based on the current time.
   *
   * @return The next observation as a string.
   */
  MeasurementType get_next_observation(ImuData &imu_measurement,
                                       Observable &observable_measurement);

  /**
   * @brief Gets the running status of the simulator.
   *
   * @return The running status as a boolean.
   */
  bool is_running() const {
    return _running || !_imu_queue->empty() || !_observable_queue->empty();
  }

  /**
   * @brief Gets the path to the output binary log
   *
   * @return Filesystem path to log
   */
  inline const std::filesystem::path &get_log_path() { return _log_path; }

private:
  // configs and string info
  std::string _config_path;           // path to the configuration file
  std::string _measurement_file_path; // path to the measurement file
  std::string _imu_data_path;         // path to the IMU data file
  float _sample_rate;                 // rate of the fastest sensor

  // logger
  std::filesystem::path _log_path;
  std::unique_ptr<Logger> _logger;

  // we require that the following keys are present in the configuration file
  std::array<std::string, 4> required_keys = {"sample_rate", "imu_data_path",
                                              "measurement_file_path",
                                              "max_measurements"};

  // the reader will dump everything into these as fast as possible
  std::unique_ptr<ThreadQueue<ImuData>>
      _imu_measurements; // array to store IMU measurements
  std::unique_ptr<ThreadQueue<Observable>>
      _observables; // array to store observable measurements

  // the timer will serve measurements from the above into here at the sensor
  // sample rates
  std::unique_ptr<ThreadQueue<ImuData>>
      _imu_queue; // queue for IMU measurements
  std::unique_ptr<ThreadQueue<Observable>>
      _observable_queue; // Queue for observable measurement

  double _current_time;             // current time in the simulation
  std::atomic<bool> _running{true}; // flags simulation is running
  std::atomic<bool> _producer_done{false};

  // early wake vars
  std::mutex cv_mtx_;
  std::condition_variable cv_;

  // declare some diagnostic variables for tracking drop outs
  std::atomic<int16_t> _imu_drops{0};         // # IMU meas dropped
  std::atomic<int16_t> _observable_drops{0};  // # observables dropped
  std::atomic<int16_t> _imu_proc{0};          // # IMU meas processed
  std::atomic<int16_t> _observable_proc{0};   // # observables processed
  std::atomic<int16_t> _last_imu_queue_sz{0}; // tracking processed counts
  std::atomic<int16_t> _last_obs_queue_sz{0}; // tracking processed counts

  void load_configurations(); // load configurations from the file

  void batcher_thread();
  void queue_setter_timer();
  void parse_line(const std::string &line, const std::string &source);
};
