#pragma once

#include <string>
#include <array>

//#include "logger.hpp"
#include "thread_safe_queue.hpp"
#include "ukf_defs.hpp"
#include "IMU_Matrices.hpp"

/**
 * @brief A playback simulator that serves observations based on the timestamps in
 * the provided measurement files.
 */
 class TimedPlaybackSim {
 public:
     /**
      * @brief Constructs a TimedPlaybackSim object.
      * 
      * @param config_path Path to the configurations for the simulator.
      */
     TimedPlaybackSim(const std::string& config_path);

     /**
      * @brief Starts the playback simulation.
      *
      * @param imu_queue A thread-safe queue containing IMU measurements.
      * @param observable_queue A thread-safe queue for observable measurements.
      */
     void start_simulation(ThreadQueue<ControlInput>& imu_queue, ThreadQueue<Observable>& observable_queue);

     /**
      * @brief Gets the next observation based on the current time.
      * 
      * @return The next observation as a string.
      */
     std::string get_next_observation();

private:
    // configs and string info
     std::string _config_path;  // path to the configuration file
     std::string _measurement_file_path;  // path to the measurement file
     std::string _imu_data_path;  // path to the IMU data file
     float _sample_rate;  // rate of the fastest sensor 

     // we require that the following keys are present in the configuration file
     std::array<std::string, 3> required_keys = {
         "sample_rate", 
         "imu_data_path", 
         "measurement_file_path"
     };

     // structures for storing parsed measurements before queuing
     // and associated size limits
     static const uint16_t _max_measurements = 1000;  // max size for IMU queue

     // the reader will dump everything into these as fast as possible
     ThreadQueue<ImuData> _imu_measurements;  // array to store IMU measurements
     ThreadQueue<Observable> _observables;  // array to store observable measurements

     // the timer will serve measurements from the above into here at the sensor sample rates
     ThreadQueue<ImuData> _imu_queue;  // queue for IMU measurements
     ThreadQueue<Observable> _observable_queue;  // Queue for observable measurement
     
     double _current_time;  // current time in the simulation
     bool _running;  // flag to indicate if the simulation is running

     void load_configurations();  // load configurations from the file

     void batcher_thread();
     void queue_setter_timer(ThreadQueue<ImuData>& imu_queue, ThreadQueue<Observable>& observable_queue);
     void parse_line(const std::string& line, const std::string& source);
 };