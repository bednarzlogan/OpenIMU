#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <Eigen/Dense>

#include "IMU_Matrices.hpp"
#include "thread_safe_queue.hpp"

/**
 * @brief Handles the ingestion, smoothing, and delivery of IMU measurements.
 *
 * The MeasurementHandler class is responsible for:
 * - Receiving IMU data (currently from a file, later possibly from a real-time sensor).
 * - Performing a rolling average to smooth noisy IMU data.
 * - Delivering smoothed data to a consumer (e.g., a Kalman filter).
 *
 * Data ingestion and smoothing run in a dedicated thread. The class supports configuration
 * via JSON for queue length and smoothing parameters.
 */
class MeasurementHandler {
    public:
       /**
        * @brief Construct a new Measurement Handler object.
        * 
        * @param path_to_configs Path to a JSON config file specifying smoothing behavior.
        */
        MeasurementHandler(std::string path_to_configs);

        /**
        * @brief Attempt to retrieve a smoothed IMU measurement.
        * 
        * This is a non-blocking call. It returns false if no smoothed measurements are available.
        * 
        * @param return_measurement Output reference for the retrieved measurement.
        * @return true if a measurement was available, false otherwise.
        */
        bool pullData(ImuData& return_measurement);

        /**
        * @brief Submit a new raw IMU measurement to the handler.
        * 
        * @param new_measurement The new raw IMU data.
        */
        void pushData(ImuData new_measurement);

        /**
        * @brief Attempt to get a new smoothed measurement.
        * 
        * @param smoothed_measurement Will be overwritten with a new measurement once available.
        */
        bool getSmoothedData(ImuData& smoothed_measurement);

        /**
        * @brief tell an outside subscriber if we're still reading measurements
        *
        * @return Status code (false = not reading, true = reading). 
        */
        bool getStreamStatus() const;

        /**
        * @brief Starts reading a stream of IMU measurements from a file.
        * 
        * This function reads and parses a CSV file of IMU data, pushes raw measurements
        * into the smoothing queue, and spawns a background thread to smooth and prepare data.
        * 
        * @param path_to_measurements_file CSV file containing IMU data.
        * @return Status code (0 = success, nonzero = error).
        */
        int openMeasurementStream(std::string path_to_measurements_file);

    private:
        /** Thread-safe queue containing raw IMU data from sensors or files. */
        ThreadQueue<ImuData> _measurements_queue;

        /** Internal buffer holding the most recent measurements for rolling averaging. */
        std::deque<ImuData> _window_queue;

        /** Queue of smoothed measurements that are ready for processing. */
        ThreadQueue<ImuData> _smoothed_measurements;

        /** Duration (in seconds) to average over; set via config file. */
        double _average_time;

        /** Timestamp of the oldest measurement in the current smoothing window. */
        double _oldest_time;

        /** Duration covered by the smoothing window. */
        double _queue_time;

        /** Milliseconds between measurement queue polling attempts. */
        uint16_t _check_freq = 5;

        /** Flag indicating whether the measurement consumer loop is running. */
        std::atomic<bool> _reading{false};

        /** Flag that indicates if the measurement stream is still open */
        std::atomic<bool> _streaming_measurements{false};

        /** Thread running the measurement loop (_loopTimer). */
        std::thread _loop_thread;

        /** Maximum number of measurements to retain in the smoothing window. */
        uint8_t _max_queue_size = 5;

        /** True once enough measurements have been received to start smoothing. */
        bool _measurement_initialized;

        /** The most recent smoothed measurement. */
        ImuData _smoothed_measurement;

        /** Cached value of the to-be-removed element from the last smoothing cycle. */
        Eigen::MatrixXd _msmt_to_remove;

        /**
        * @brief Start the background smoothing thread.
        */
        void _startLoop();

        /**
        * @brief Stop the background smoothing thread and join safely.
        */
        void _stopLoop();

        /**
        * @brief Perform a smoothing pass using the current measurement.
        * 
        * Applies a rolling average using `_max_queue_size` measurements.
        * Updates the smoothed queue with the new result.
        * 
        * @param new_measurement The newest IMU measurement.
        */
        void _doSmoothing(const ImuData& new_measurement);

        /**
        * @brief The background loop that polls for measurements and triggers smoothing.
        * 
        * Runs in its own thread while `_reading == true`. Controlled via `_startLoop` and `_stopLoop`.
        */
        void _loopTimer();
};