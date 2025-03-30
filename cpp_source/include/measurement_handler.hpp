#ifndef MEASUREMENT_HANDLER_HPP
#define MEASUREMENT_HANDLER_HPP

#include <atomic>
#include <cstdint>
#include <Eigen/Dense>

#include "IMU_Matrices.hpp"
#include "thread_safe_queue.hpp"

/* So far, I'm just using this for measurement smoothing, but we may use it more later! */

class MeasurementHandler {
    public:
        // generic constructor : configs will tell us how much smoothing to do
        MeasurementHandler(const std::string path_to_configs);

        // gives a measurement back to the consumer
        bool pullData(ImuData& return_measurement);

        // give it a measurement to queue up
        void pushData(ImuData new_measurement);

        // flushes queues, flags that we're no longer accepting/processing measurements
        void resetSmoother();

        // for now, this plays back data from a file. Later, this will probably
        // be an SPI for interacting with IMU hardware
        int openMeasurementStream(std::string path_to_measurements_file);

    private:
        // queues: 
        ThreadQueue<ImuData> _measurements_queue; // all msmts coming from sensor/file
        std::deque<ImuData> _window_queue; // the measurements we're going to smooth
        std::deque<ImuData> _smoothed_measurements; // ready-to-use measurements

        // the averaging time
        double _average_time;

        // current measurement delta to find how long the buffer is
        double _oldest_time;
        double _queue_time;

        // how often should we check for new measurements
        uint16_t _check_freq = 5;  // check for a meausrement every X ms 
        std::atomic<bool> _reading{false}; // tells us that we're still parsing meausremens

        // have an array of known size to hold the window
        // measurements in. If we can't get enough time
        // before a certain amount of time passes or something like that,
        // just parse the measurement
        uint8_t _max_queue_size = 5;
        

        // smoothed measuremement
        bool _measurement_initialized;
        ImuData _smoothed_measurement;
        Eigen::MatrixXd _msmt_to_remove; // to-be-subtracted element when window slides 

        // perform a windowed average
        void _doSmoothing(const ImuData& new_measurement);
        
        // checks for new measurements
        void _loopTimer();
};

#endif // MEASUREMENT_HANDLER_HPP