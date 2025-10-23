#pragma once

#include <array>
#include <string>

#include "estimator_interface.hpp"
#include "logger.hpp"
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
   * @brief Gets the running status of the simulator.
   *
   * @return The running status as a boolean.
   */
  bool is_running() const { return _running; }

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

  // logger
  std::filesystem::path _log_path;
  std::unique_ptr<Logger> _logger;

  // we require that the following keys are present in the configuration file
  std::array<std::string, 4> required_keys = {"sample_rate", "imu_data_path",
                                              "measurement_file_path",
                                              "max_measurements"};

  double _current_time; // current time in the simulation
  bool _running{true};  // flags simulation is running

  void load_configurations(); // load configurations from the file

  MeasurementType parse_line(const std::string &line, const std::string &source,
                             ImuData &imu_data, Observable &observable_data);
};
