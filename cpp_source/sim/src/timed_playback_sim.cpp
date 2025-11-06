#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "logger.hpp"
#include "logger_conversions.hpp"
#include "timed_playback_sim.hpp"
#include "ukf_defs.hpp"

using json = nlohmann::json;
using namespace std::chrono;
using clk = std::chrono::steady_clock;

// TMP -- not used right now
static std::ofstream debug_log_playback("playback_debug.csv", std::ios::app);

bool check_key(std::string key, const json &j) {
  // check if the key exists in the JSON object
  if (j.contains(key)) {
    return true;
  } else {
    std::cerr << "Key '" << key << "' not found in configuration file."
              << std::endl;
    return false;
  }
}

// loader for constructor
void TimedPlaybackSim::load_configurations() {
  // open the config file
  std::ifstream config_file;
  config_file.open(_config_path);
  if (!config_file.is_open()) {
    throw std::runtime_error("Failed to open configuration file: " +
                             _config_path);
  }

  // read the configurations
  json j;
  config_file >> j;

  // set the current time to zero
  _current_time = 0.0;

  // set running flag to true
  _running = true;

  // close the config file
  config_file.close();

  // check for required keys
  for (const auto key : required_keys) {
    if (!check_key(key, j)) {
      throw std::runtime_error("Missing required key in configuration file: " +
                               key);
    }
  }

  // set desired fields
  _imu_data_path = j["imu_data_path"];
  _measurement_file_path = j["measurement_file_path"];

  // set up the logger
  const std::string output_dir = j["output_dir"];
  const std::string log_path_str = output_dir + "TimedPlaybackSim.bin";
  _logger = std::make_unique<Logger>(log_path_str);

  // track where the log will be written for the test units
  std::filesystem::path log_path(log_path_str);
  _log_path = log_path;
}

TimedPlaybackSim::TimedPlaybackSim(const std::string &config_path)
    : _config_path(config_path), _current_time(0.0), _running(false) {
  load_configurations();
}

MeasurementType TimedPlaybackSim::parse_line(const std::string &line,
                                             const std::string &source,
                                             ImuData &imu_data,
                                             Observable &observable_data) {
  std::stringstream ss(line);
  std::string value;
  std::array<double, M + 1> measurement_parts;
  std::array<double, 2 * Z + 1> gnss_measurement_parts;
  ImuData imu_measurement;
  Observable measurement;

  // skip empty lines and headers
  if (line.empty())
    return NO_MEASUREMENT;
  if (line.rfind("timestamp", 0) == 0)
    return NO_MEASUREMENT;

  uint8_t i = 0;
  if (source == "imu") {
    while (std::getline(ss, value, ',')) {
      // std::cout << value << std::endl;
      if (i >= M + 1) {
        std::cerr << "Too many elements in IMU CSV row!" << std::endl;
        return NO_MEASUREMENT;
      }
      try {
        measurement_parts[i] = std::stod(value);
      } catch (...) {
        return NO_MEASUREMENT;
      }
      i++;
    }
    if (i != (uint8_t)M + 1) {
      // log out the rejected IMU measurement
      Eigen::Matrix<double, M + 1, 1> out;

      // we'll have to parse out the valid part of the measurement
      const size_t nparsed = static_cast<size_t>(i);
      const size_t ncopy = std::min((int)nparsed, M + 1);

      // assign in the parts that won't overflow the size
      out.setZero();
      for (size_t idx = 0; idx < ncopy; ++idx) {
        out(idx) = measurement_parts[idx];
      }
      log_vector_out(*_logger, out, LoggedVectorType::RejectedIMU);
      return NO_MEASUREMENT;
    }

    measurement.timestamp = measurement_parts[0];

    for (int j = 0; j < M; ++j) {
      imu_measurement.matrix_form_measurement(j) = measurement_parts[j + 1];
    }
    imu_measurement.measurement_time = measurement_parts[0];
    imu_measurement.updateFromMatrix();

    // populate reference so outside functions get a measurement back
    imu_data = imu_measurement;

    // log out correctly formatted IMU
    log_vector_out(*_logger, imu_measurement.matrix_form_measurement,
                   LoggedVectorType::ImuRaw);
    return IMU;

  } else if (source == "gnss") {
    while (std::getline(ss, value, ',')) {
      if (i >= 2 * Z + 1) {
        std::cerr << "Too many elements in GNSS CSV row!" << std::endl;
        return NO_MEASUREMENT;
      }
      try {
        gnss_measurement_parts[i] = std::stod(value);
      } catch (...) {
        return NO_MEASUREMENT;
      }
      i++;
    }
    if (i != (uint8_t)2 * Z + 1) {
      // similar to above, log out the rejected GNSS measurement
      Eigen::Matrix<double, 2 * Z + 1, 1> out;
      out.setZero();

      const size_t nparsed = static_cast<size_t>(i);
      const size_t ncopy = std::min(nparsed, static_cast<size_t>(2 * Z + 1));

      for (size_t idx = 0; idx < ncopy; ++idx) {
        out(static_cast<Eigen::Index>(idx)) = gnss_measurement_parts[idx];
      }

      log_vector_out(*_logger, out, LoggedVectorType::RejectedGNSS);
      return NO_MEASUREMENT;
    }

    // copy in observation info -- ensure that no elements are
    // floating/undefined
    measurement.timestamp = gnss_measurement_parts[0];
    measurement.R.setIdentity();
    for (int j = 0; j < Z; ++j) {
      measurement.observation(j) = gnss_measurement_parts[j + 1];
    }
    for (int j = 0; j < Z; ++j) {
      measurement.R(j, j) = gnss_measurement_parts[j + 1 + Z];
    }

    // populate the reference so that outside functions can access it
    observable_data = measurement;

    // log out correctly formatted GNSS
    Eigen::Matrix<double, Z + 1, 1> gnss;
    gnss(0, 0) = measurement.timestamp;
    gnss.head(Z) = measurement.observation;
    log_vector_out(*_logger, gnss, LoggedVectorType::GNSS);

    return GNSS;
  }

  // bad source case
  return NO_MEASUREMENT;
}

bool TimedPlaybackSim::wait_until_queue_has_space(
    bool imu, std::chrono::milliseconds timeout,
    const std::shared_ptr<Estimator> &est) noexcept {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + timeout;
  auto sleep_us = std::chrono::microseconds(200);

  while (_running) {
    if (!est->queues_full(imu))
      return true; // wait until NOT full
    if (clock::now() >= deadline)
      return false;                        // timeout
    std::this_thread::sleep_for(sleep_us); // backoff
    sleep_us = std::min(sleep_us * 2, std::chrono::microseconds(5000));
  }
  return false; // stopped while waiting
}

void TimedPlaybackSim::pass_measurements(bool &imu_valid, bool &gnss_valid,
                                         const ImuData &imu_data,
                                         const Observable &gnss_data,
                                         std::shared_ptr<Estimator> estimator) {
  // ensure that the queues are not full before passing measurements
  if (imu_valid && !wait_until_queue_has_space(
                       true, std::chrono::milliseconds(2000), estimator))
    return;
  if (gnss_valid && !wait_until_queue_has_space(
                        false, std::chrono::milliseconds(2000), estimator))
    return;

  if (imu_valid && gnss_valid) {
    // process the earlier of the GNSS and IMU data
    bool imu_earlier = imu_data.measurement_time <= gnss_data.timestamp;

    // process the earlier measurement
    // TODO wait unti queues are not full to push a measurement
    if (imu_earlier) {
      // pass imu data to estimator
      if (debug_log_playback.is_open()) {
        debug_log_playback << "Passing IMU data to UKF:\n"
                           << imu_data.matrix_form_measurement.transpose()
                           << std::endl;
      }
      if (!estimator->queues_full(true)) {
        estimator->read_imu(imu_data);
        imu_valid = false;
      }
    } else {
      // process GNSS data
      if (debug_log_playback.is_open()) {
        debug_log_playback << "Passing GNSS data to UKF:\n"
                           << gnss_data.observation.transpose() << std::endl;
      }
      if (!estimator->queues_full(false)) {
        estimator->read_gps(gnss_data);
        gnss_valid = false;
      }
    }
  } else if (imu_valid) {
    // process IMU data
    if (debug_log_playback.is_open()) {
      debug_log_playback << "Passing IMU data to UKF:\n"
                         << imu_data.matrix_form_measurement.transpose()
                         << std::endl;
    }
    if (!estimator->queues_full(true)) {
      estimator->read_imu(imu_data);
      imu_valid = false;
    }
  } else if (gnss_valid) {
    // process GNSS data
    if (!estimator->queues_full(false)) {
      estimator->read_gps(gnss_data);
      gnss_valid = false;
    }
  }
}

void TimedPlaybackSim::start_simulation(std::shared_ptr<Estimator> estimator,
                                        std::chrono::milliseconds period) {
  // The processing loop is like this:
  // 1. Set a timer based on the fastest sensor sample rate
  // 2. Start a thread which will read measurements into a local memory
  // structure while we wait for timer ticks
  // 3. At each timer tick, check the oldest measurement in the local memory
  // structure and see if the sim time
  //    is >= the measurement time; if yes, push the measurement to the
  //    respective queue

  std::ifstream imu_file(_imu_data_path);
  std::ifstream gnss_file(_measurement_file_path);

  if (!imu_file.is_open() || !gnss_file.is_open()) {
    // signal producers are done and bail
    _running = false;
    return;
  }

  // start UKF
  estimator->start_filter(period);
  _running = true;

  std::string imu_line;
  std::string gnss_line;

  // skip headers (ok if empty: getline returns false and we’ll hit *_done
  // below)
  std::getline(imu_file, imu_line);
  std::getline(gnss_file, gnss_line);

  // allocate measurements
  ImuData imu_data;
  Observable gnss_data;
  bool imu_valid = false;
  bool gnss_valid = false;

  bool imu_done = false, gnss_done = false;
  while ((!imu_done || !gnss_done) and _running) {
    // walk through the files if we need a new measurement
    if (!imu_done and !imu_valid) {
      if (std::getline(imu_file, imu_line)) {
        if (parse_line(imu_line, "imu", imu_data, gnss_data) == IMU)
          imu_valid = true;
      } else {
        imu_done = true;
      }
    }
    if (!gnss_done and !gnss_valid) {
      if (std::getline(gnss_file, gnss_line)) {
        if (parse_line(gnss_line, "gnss", imu_data, gnss_data) == GNSS)
          gnss_valid = true;
      } else {
        gnss_done = true;
      }
    }

    // pass any measurement data to the estimator
    pass_measurements(imu_valid, gnss_valid, imu_data, gnss_data, estimator);

    // log out current state
    log_vector_out(*_logger, estimator->get_state(),
                   LoggedVectorType::NominalState);
  }

  // allow time for the estimator to process the last measurement
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  // ensure that the queue is empty before stopping the filter
  auto drained = [&] {
    constexpr double eps = 1e-6;
    const bool q_empty =
        estimator->queues_empty(true) && estimator->queues_empty(false);
    const bool at_tip =
        estimator->solution_time() + eps >= imu_data.measurement_time;

    std::cout << "Drained: " << q_empty << ", At Tip: " << at_tip
              << " with measurement time " << imu_data.measurement_time
              << std::endl;
    return q_empty && at_tip;
  };

  // wait up to ~2 minutes
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(120000);
  while (!drained() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // wait for user interrupt or internal flag to end
  _running = false;
  std::cout << "Simulation stopped." << std::endl;
  estimator->stop_filter();
}

void TimedPlaybackSim::stop_simulation() {
  _running = false;
  std::cout << "Simulation stopped via interrupt." << std::endl;
}
