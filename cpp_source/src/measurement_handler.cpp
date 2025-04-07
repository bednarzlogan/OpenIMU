#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

// writing current date and time to the debug file
#include <ctime>
#include <iostream>

#include "IMU_Matrices.hpp"
#include "measurement_handler.hpp"

using json = nlohmann::json;

std::string formatLogName(std::string base_name, std::string extension = ".txt") {
  auto time = std::time(nullptr);  // returns current time, null tells it we aren't assigning to a timer
  auto tm = *std::localtime(&time);  // time in a struct that works with strftime

  // make the datetime into a string and concatenate with the base name
  char buffer[80];
  std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", &tm);
  return base_name + "_" + std::string(buffer) + extension;
}

std::ofstream diag_log_smoother(formatLogName("measurement_smoother_out"), std::ios::app);

MeasurementHandler::MeasurementHandler(const std::string path_to_configs):
    _oldest_time(0),
    _queue_time(0),
    _measurement_initialized(false),
    _streaming_measurements(true)
{
    // open a file stream to read from
    std::ifstream inFile(path_to_configs);
    if (!inFile.is_open()) {
        std::cerr << "Could not open config file: " << path_to_configs << std::endl;
    }

    // stream the chars into a json object
    json j;
    inFile >> j;

    // find the config for the averaging window
    _average_time = j["averaging_time"];  // TODO - not used until we move to real time

    // find a max queue size argument
    _max_queue_size = j["max_queue_size"];
}

bool MeasurementHandler::getStreamStatus() const { 
    return _streaming_measurements;
}

void MeasurementHandler::_startLoop() {
    if (_reading)
        return; // already running the thread

    _reading = true;
    _loop_thread = std::thread(&MeasurementHandler::_loopTimer, this);
}

void MeasurementHandler::_stopLoop() {
    _reading = false;
    if (_loop_thread.joinable()) {
        _loop_thread.join();  // Wait for thread to finish cleanly
    }

    // clean up variables
    _window_queue.clear();
    _measurement_initialized = false;
    _queue_time = 0;
    _streaming_measurements = false;
}

void MeasurementHandler::_loopTimer() {
    while (_reading || !_measurements_queue.empty()) {
        /* attempt to pull in a measurement and produce a measurement */

        // enforces the checking frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(_check_freq));

        // allocate a placeholder that will be populated by try_pop, if possible
        ImuData result_measurement;

        // try to get a measurement and smooth
        if (_measurements_queue.try_pop(result_measurement)) {
            _doSmoothing(result_measurement);
        }
    }
}

bool MeasurementHandler::pullData(ImuData& return_measurement) {
    // get a measurement when available
    /// NOTE: maybe we can just do try_pop and continue pushing measurements
    /// if we don't have anything to pull, which try_pop will tell us
    bool ready = _measurements_queue.try_pop(return_measurement);

    return ready;
}

void MeasurementHandler::pushData(ImuData new_measurement) { 
    // give the data to the queue in a thread safe way
    _measurements_queue.push(new_measurement);
}

bool MeasurementHandler::getSmoothedData(ImuData& smoothed_measurement){
    if (!_streaming_measurements && _smoothed_measurements.empty())
        return false;
    _smoothed_measurements.wait_and_pop(smoothed_measurement);
    return true;
}

void MeasurementHandler::_doSmoothing(const ImuData& new_measurement) { 
    // push into sliding window queue
    _window_queue.push_front(new_measurement);

    // check for full window or lagging measurements
    // TODO - implement || _queue_time > _average_time into the clause for real time
    if (_window_queue.size() == _max_queue_size) {
        // reset the queue start time -- not used right now
        double newest_time = new_measurement.measurement_time;
        _oldest_time = newest_time;
        _smoothed_measurement.measurement_time = newest_time;

        // if we've never made a smoothed measurement, we have to average the whole window
        if (!_measurement_initialized) { 
            for (auto msmt_iterator = _window_queue.begin(); msmt_iterator != _window_queue.end(); ++msmt_iterator) {
                _smoothed_measurement.matrix_form_measurement += msmt_iterator->matrix_form_measurement;
            }
            _smoothed_measurement.matrix_form_measurement /= _window_queue.size(); // check if this bytes or num elements
            _smoothed_measurement.updateFromMatrix();  // populate the double members

            // pop the oldest element
            _msmt_to_remove = _window_queue.back().matrix_form_measurement / _window_queue.size();
            _window_queue.pop_back();

            // flag that we don't need this operation anymore
            _measurement_initialized = true;
        } 
        else {
            // slide should only require 1 math operation and a pop
            auto new_addition = new_measurement.matrix_form_measurement / _window_queue.size();
            _smoothed_measurement.matrix_form_measurement -= _msmt_to_remove;
            _smoothed_measurement.matrix_form_measurement += new_addition;
            _smoothed_measurement.updateFromMatrix();  // populate the double members

            // get ready for next slide
            _msmt_to_remove = _window_queue.back().matrix_form_measurement / _window_queue.size();
            _window_queue.pop_back();
        }

        // add to queue of ready-to-process measurements
        diag_log_smoother << "\nSmoothed Measurement: \n" 
                        << _smoothed_measurement.matrix_form_measurement.transpose() 
                        << "\nat time: " 
                        << _smoothed_measurement.measurement_time 
                        << std::endl;
        _smoothed_measurements.push(_smoothed_measurement);
    } 
    else {
        // the sliding window won't slow down the measurement dt
        // to maintain this for all time, return the unsmoothed measurements
        // until we have smoothed measurements to give
        _smoothed_measurements.push(new_measurement);
    } 

    // update the dt of the queue -- not used right now
    _queue_time = new_measurement.measurement_time - _oldest_time;
}

int MeasurementHandler::openMeasurementStream(std::string path_to_measurements_file){
    /// NOTE: this will run as fast as the CPU can
    // TODO - make some enumeration for status codes and assign them to this
    int return_code = 0;

    // start consumer
    _startLoop();

    std::ifstream infile(path_to_measurements_file);
    if (!infile.is_open()) {
      std::cerr << "Error: Could not open file: " << path_to_measurements_file << std::endl;
      return -1;
    }
  
    std::string measurement_line;
    std::string line;
  
    // lines one at a time
    while (std::getline(infile, line)) {
      // assign a string stream to break up words
      std::stringstream ss(line);
      std::vector<std::string> row;
      std::string value;
  
      // interact with the stringstream to grab each word between delimiters
      while (std::getline(ss, value, ',')) { // Parse CSV by commas
        row.push_back(value);
      }
      
      // TMP print outputs
      for (const auto& elem : row) {
        std::cout << elem << " ";
      }
      std::cout << std::endl;
  
      // check for a complete 6 elements and convert to doubles
      if (row.size() != 7) {
        std::cerr << "Less than 6 measurements contained in row. CSV format is invalid!" << std::endl;
        return 1;
      }
  
      // allocate the measurement and assign the variables
      ImuData imu_measurements;
      uint count = 0;
  
      // kind of lengthy, but I'm doing this to be explicit about what is what in assigning members in the 
      // state space model
      for (const auto& elem : row) {
        double measurement;
  
        // if the line contains numbers, read in and asign to IMU measurement struct
        try {
          measurement = std::stod(elem);
        } catch (const std::invalid_argument& e) {
          break;
        }
  
        switch (count) {
          case 0:
            imu_measurements.measurement_time = measurement*1e-3;  // timestamps are in msec
            break;
          case 1:
            imu_measurements.dphix = measurement * M_PI/180;
            break;
          case 2:
            imu_measurements.dthetay = measurement * M_PI/180;
            break;
          case 3:
            imu_measurements.dpsiz = measurement * M_PI/180;
            break;
            case 4:
            imu_measurements.accx = measurement * 9.80665;  // convert g's to m/s^2, assuming the input is in g's (standard for IMU)
            break;
          case 5:
            imu_measurements.accy = measurement * 9.80665;
            break;
          case 6:
            imu_measurements.accz = measurement * 9.80665 - 9.80665; // The imu will read 1g when stationary, so we need to zero it out for the z-axis
            break;
          default:
            __builtin_unreachable();
        }
        count++;
      }

      // push into queue
      imu_measurements.updateFromDoubles();  // ensure the matrix form is populated before pushing
      std::cout << "Pushing IMU measurement: " 
                << imu_measurements.matrix_form_measurement.head(3).transpose() 
                << " at time: " << imu_measurements.measurement_time << std::endl;
      pushData(imu_measurements);
    }
  
    // close the file
    infile.close();
    _stopLoop();
    return return_code;
}
