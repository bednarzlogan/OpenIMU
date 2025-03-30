#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

#include "IMU_Matrices.hpp"
#include "measurement_handler.hpp"

using json = nlohmann::json;

void MeasurementHandler::_loopTimer() {
    while (_reading) {
        /* attempt to pull in a measurement and produce a measurmement */

        // enforces the checking frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(_check_freq));

        // allocate a flag that the que has measurements we can grab
        bool m_avail = false; 

        // allocate a placeholder that will be populated by try_pop, if possible
        ImuData result_measurement;

        // try to get a measurement and smooth
        m_avail = _measurements_queue.try_pop(result_measurement);
        if (m_avail) {
            _doSmoothing(result_measurement);
        }
    }
}

MeasurementHandler::MeasurementHandler(const std::string path_to_configs):
    _oldest_time(0),
    _queue_time(0),
    _measurement_initialized(false)
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
    _max_queue_size = j["_max_queue_size"];
}

void MeasurementHandler::resetSmoother() {
    _window_queue.clear();
    _measurement_initialized = false;
    _queue_time = 0;
    _reading = false;
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
    return;
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
        _smoothed_measurements.push_front(_smoothed_measurement);
    }

    // update the dt of the queue -- not used right now
    _queue_time = new_measurement.measurement_time - _oldest_time;
}

int MeasurementHandler::openMeasurementStream(std::string path_to_measurements_file){
    /// NOTE: this will run as fast as the CPU can
    // TODO - make some enumeration for status codes and assign them to this
    int return_code = 0;

    // start consumer
    _reading = true;
    std::thread(&MeasurementHandler::_loopTimer, this).detach();

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
      bool contains_numbers = false;
  
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
  
        contains_numbers = true;
        switch (count) {
          case 0:
            imu_measurements.measurement_time = measurement*1e-3;  // timestamps are in msec
            break;
          case 4:
            imu_measurements.accx = measurement * M_PI/180;
            break;
          case 5:
            imu_measurements.accy = measurement * M_PI/180;
            break;
          case 6:
            imu_measurements.accz = measurement * M_PI/180;
            break;
          case 1:
            imu_measurements.dphix = measurement;
            break;
          case 2:
            imu_measurements.dthetay = measurement;
            break;
          case 3:
            imu_measurements.dpsiz = measurement;
            break;
          default:
            __builtin_unreachable();
        }
        count++;
      }

      // push into queue
      pushData(imu_measurements);
    }
  
    // close the file
    infile.close();
    resetSmoother();
    return return_code;
}
