#include <array>
#include <algorithm>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <thread>
#include <sstream>
#include <limits>
#include <algorithm>
#include <fstream>
#include <iostream>

#include "timed_playback_sim.hpp"
#include "ukf_defs.hpp"
#include "logger_conversions.hpp"


using json = nlohmann::json;
using namespace std::chrono;
using clk = std::chrono::steady_clock;


bool check_key(std::string key, const json& j) {
    // check if the key exists in the JSON object
    if (j.contains(key)) {
        return true;
    } else {
        std::cerr << "Key '" << key << "' not found in configuration file." << std::endl;
        return false;
    }
}

// loader for constructor
void TimedPlaybackSim::load_configurations() {
    // open the config file
    std::ifstream config_file;
    config_file.open(_config_path);
    if (!config_file.is_open()) {
        throw std::runtime_error("Failed to open configuration file: " + _config_path);
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
    for (const auto key: required_keys) {
        if (!check_key(key, j)) {
            throw std::runtime_error("Missing required key in configuration file: " + key);
        }
    }
    
    // set desired fields
    _sample_rate = j["sample_rate"];
    _imu_data_path = j["imu_data_path"];
    _measurement_file_path = j["measurement_file_path"];
    uint16_t max_measurements = j["max_measurements"];

    // initialize queues

    // these will hold the preprocessed measurements and are only meant to by used in simulations, so 
    // they can be as large as needed
    _imu_measurements = std::make_unique<ThreadQueue<ImuData>>();
    _observables = std::make_unique<ThreadQueue<Observable>>();

    // these are the queues that simulate the real-time measurement streams
    _imu_queue = std::make_unique<ThreadQueue<ImuData>>((size_t) max_measurements);
    _observable_queue = std::make_unique<ThreadQueue<Observable>>((size_t) max_measurements);

    // set up the logger
    const std::string output_dir = j["output_dir"];
    const std::string log_path_str = output_dir + "TimedPlaybackSim.bin";
    _logger = std::make_unique<Logger>(log_path_str);

    // track where the log will be written for the test units
    std::filesystem::path log_path(log_path_str);
    _log_path = log_path;
}


TimedPlaybackSim::TimedPlaybackSim(const std::string& config_path) 
    : _config_path(config_path), _current_time(0.0), _running(false) {
    load_configurations();
}


void TimedPlaybackSim::parse_line(const std::string& line, const std::string& source) {
    std::stringstream ss(line);
    std::string value;
    std::array<double, Z + 1> measurement_parts;
    std::array<double, 2 * Z + 1> gnss_measurement_parts;
    ImuData imu_measurement;
    Observable measurement;

    // skip empty lines and headers
    if (line.empty()) return;
    if (line.rfind("timestamp", 0) == 0) return;

    uint8_t i = 0;
    if (source == "imu") {
        while (std::getline(ss, value, ',')) {
            if (i >= Z + 1) {
                std::cerr << "Too many elements in IMU CSV row!" << std::endl;
                return;
            }
            try {
                measurement_parts[i] = std::stod(value);
            } catch (...) {
                return;
            }
            i++;
        }
        if (i != (uint8_t) M + 1) {
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
            return;
        }

        measurement.timestamp = measurement_parts[0];

        for (int j = 0; j < Z; ++j) {
            imu_measurement.matrix_form_measurement(j) = measurement_parts[j + 1];
        }
        imu_measurement.measurement_time = measurement_parts[0];
        imu_measurement.updateFromMatrix();
        _imu_measurements->push(imu_measurement);

        // log out correctly formatted IMU
        log_vector_out(*_logger, imu_measurement.matrix_form_measurement, LoggedVectorType::ImuRaw);

    }
    else if (source == "gnss") {
        while (std::getline(ss, value, ',')) {
            if (i >= 2 * Z + 1) {
                std::cerr << "Too many elements in GNSS CSV row!" << std::endl;
                return;
            }
            try {
                gnss_measurement_parts[i] = std::stod(value);
            } catch (...) {
                return;
            }
            i++;
        }
        if (i != (uint8_t) 2 * Z + 1) {
            // similar to above, log out the rejected GNSS measurement
            Eigen::Matrix<double, 2 * Z + 1, 1> out;
            out.setZero();

            const size_t nparsed = static_cast<size_t>(i);
            const size_t ncopy   = std::min(nparsed, static_cast<size_t>(2 * Z + 1));

            for (size_t idx = 0; idx < ncopy; ++idx) {
                out(static_cast<Eigen::Index>(idx)) = gnss_measurement_parts[idx];
            }

            log_vector_out(*_logger, out, LoggedVectorType::RejectedGNSS);
            return;
        }

        measurement.timestamp = gnss_measurement_parts[0];
        for (int j = 0; j < Z; ++j) {
            measurement.observation(j) = gnss_measurement_parts[j + 1];
        }
        for (int j = 0; j < Z; ++j) {
            measurement.R(j, j) = gnss_measurement_parts[j + 1 + Z];
        }
        _observables->push(measurement);

        // log out correctly formatted GNSS
        Eigen::Matrix<double, Z + 1, 1> gnss;
        gnss(0, 0) = measurement.timestamp;
        gnss.tail(Z) = measurement.observation;
        log_vector_out(*_logger, gnss, LoggedVectorType::GNSS);
    }
}


void TimedPlaybackSim::batcher_thread() {
    std::ifstream imu_file(_imu_data_path);
    std::ifstream gnss_file(_measurement_file_path);

    std::string line;
    std::getline(imu_file, line); // skip header
    std::getline(gnss_file, line); // skip header

    bool imu_done = false, gnss_done = false;
    while (!imu_done || !gnss_done) {
        if (!imu_done) {
            if (std::getline(imu_file, line)) parse_line(line, "imu");
            else imu_done = true;
        }
        if (!gnss_done) {
            if (std::getline(gnss_file, line)) parse_line(line, "gnss");
            else gnss_done = true;
        }
    }
    _producer_done.store(true, std::memory_order_release);
}


void TimedPlaybackSim::queue_setter_timer() {
    // This function will set a timer based on the sample rate and push measurements to the respective queues
    // It will check the oldest measurement in the local memory structure and see if the sim time is >= the measurement time
    // If yes, it will push the measurement to the respective queue
    // This function will run in a separate thread
    // Define simulation time reference (starts at time of first measurement)
    double sim_start_time = -1.0;

    // system time reference point
    time_point wall_clock_start = clk::now();

    while (_running) {
        ImuData imu_m;
        Observable obs_m;

        // check for available measurements and populate if available
        bool imu_avail = _imu_measurements->front(imu_m);
        bool obs_avail = _observables->front(obs_m);

        if (sim_start_time < 0.0) {
            // If we haven't set the sim start time, set it to the first measurement we have
            if (!imu_avail && !obs_avail) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (imu_avail && obs_avail)
                sim_start_time = std::min(imu_m.measurement_time, obs_m.timestamp);
            else if (imu_avail)
                sim_start_time = imu_m.measurement_time;
            else
                sim_start_time = obs_m.timestamp;

            wall_clock_start = clk::now();  // start pacing once we know sim epoch
        }

        auto now = clk::now();
        double sim_elapsed = std::chrono::duration<double>(now - wall_clock_start).count();
        double sim_now = sim_start_time + sim_elapsed;
        double t_min = std::numeric_limits<double>::infinity();

        // we may have several measurements to push, so drain the queues as much as possible
        for (;;) {
            imu_avail = _imu_measurements->front(imu_m);
            obs_avail = _observables->front(obs_m);
            if (!imu_avail && !obs_avail) {
                // if we've got nothing else in the producer queues and we've 
                // finished reading the files in, we can stop the simulation
                if (_producer_done.load(std::memory_order_acquire))  {
                    _running.store(false, std::memory_order_relaxed);
                    { std::lock_guard<std::mutex> lk(cv_mtx_); }
                    cv_.notify_all();
                }
                break;  // even if not done, nothing to push
            }

            double next_imu_time = imu_avail ? imu_m.measurement_time : std::numeric_limits<double>::infinity();
            double next_obs_time = obs_avail ? obs_m.timestamp : std::numeric_limits<double>::infinity();

            t_min = std::min(next_imu_time, next_obs_time);
            if (t_min > sim_now) break;  // next measurement is in the future
            
            // push the next measurement(s)
            bool new_meas = false;
            if (imu_avail && next_imu_time <= sim_now) {
                _imu_measurements->try_pop(imu_m);   // consume the one we just peeked
                _imu_queue->push(imu_m);               // deliver to live queue
                new_meas = true;
            }
            if (obs_avail && next_obs_time <= sim_now) {
                _observables->try_pop(obs_m);
                _observable_queue->push(obs_m);
                new_meas = true;
            }

            if (new_meas) {
                // notify any waiting threads that new data is available
                { std::unique_lock<std::mutex> lk(cv_mtx_); }
                cv_.notify_all();
            }
        }

        // check t_min to decide how long to sleep
        double dt_s = (t_min > sim_now) ? std::min(t_min - sim_now, 0.05) : 0.010; // cap at 50 ms

        // we need to handle t_min very close to sim_now by using this_thread::yield
        // to allow us to re-loop, but allow other threads to run if needed
        if (dt_s > 0.001) 
            std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
        else 
            std::this_thread::yield();
    }
}


void TimedPlaybackSim::start_simulation(std::shared_ptr<Estimator> estimator, std::chrono::milliseconds period) {
    // The processing loop is like this:
    // 1. Set a timer based on the fastest sensor sample rate
    // 2. Start a thread which will read measurements into a local memory structure while we wait for timer ticks
    // 3. At each timer tick, check the oldest measurement in the local memory structure and see if the sim time 
    //    is >= the measurement time; if yes, push the measurement to the respective queue
    
    // open the thread that batches in the measurements
    std::thread batcher_thread(&TimedPlaybackSim::batcher_thread, this); 

    // start a thread for the timer that will push measurements to the queues
    std::thread queue_setter_thread(&TimedPlaybackSim::queue_setter_timer, this);

    // start the estimator processing loop
    estimator->start_filter(period);

    auto can_act = [&]{
        // wake if: stop requested, or there is any live data to send to the estimator,
        // or sim has finished (done & live queues empty) so we can exit
        const bool stop  = !_running.load(std::memory_order_relaxed);
        const bool data  = !_imu_queue->empty() || !_observable_queue->empty();
        const bool done  = _producer_done.load(std::memory_order_relaxed)
                        && _imu_queue->empty() && _observable_queue->empty();
        return stop || data || done;
    };

    for (;;) {
        // wait until something changed
        std::unique_lock<std::mutex> lk(cv_mtx_);
        cv_.wait(lk, can_act);
        lk.unlock();

        if (!_running.load(std::memory_order_relaxed)) {
            // optional: drain any stragglers before exiting, or just break
            if (_imu_queue->empty() && _observable_queue->empty()) break;
        }

        // repeatedly attempt to fetch a measurement and serve it to the estimator
        ImuData imu_m;
        Observable obs_m;
        MeasurementType m_type = get_next_observation(imu_m, obs_m);
        if (m_type == MeasurementType::IMU) {
            estimator->read_imu(imu_m);
        } else if (m_type == MeasurementType::GNSS) {
            estimator->read_gps(obs_m);
        } else if (m_type == MeasurementType::NO_MEASUREMENT && _producer_done.load(std::memory_order_relaxed)
            && _imu_queue->empty() && _observable_queue->empty()) {
            break; // clean shutdown
        }

        if (m_type != MeasurementType::NO_MEASUREMENT) {
            log_vector_out(*_logger, estimator->get_state(), LoggedVectorType::NominalState);
        }
    }

    // wait for user interrupt or internal flag to end
    batcher_thread.join();
    queue_setter_thread.join();
    
    std::cout << "Simulation stopped." << std::endl;
}


void TimedPlaybackSim::stop_simulation() {
    _running.store(false, std::memory_order_relaxed);
    _producer_done.store(true, std::memory_order_release);
    { std::lock_guard<std::mutex> lk(cv_mtx_); }
    cv_.notify_all();
    std::cout << "Simulation stopped via interrupt." << std::endl;
}


MeasurementType TimedPlaybackSim::get_next_observation(ImuData& imu_measurement, Observable& observable_measurement) {
    // this function will try to pop the next IMU and observable measurements from the queues in chronological order
    // It returns true a measurement is available, false otherwise

    // allocate dummy variables to hold the popped measurements
    bool imu_avail = _imu_queue->front(imu_measurement);
    bool observable_avail = _observable_queue->front(observable_measurement);

    // return the older of the two measurements
    if (imu_avail && observable_avail) {
        if (imu_measurement.measurement_time <= observable_measurement.timestamp) 
            return _imu_queue->try_pop(imu_measurement) ? MeasurementType::IMU : MeasurementType::NO_MEASUREMENT;
        else
            return _observable_queue->try_pop(observable_measurement) ? MeasurementType::GNSS : MeasurementType::NO_MEASUREMENT;
    } 
    else if (imu_avail) 
        return _imu_queue->try_pop(imu_measurement) ? MeasurementType::IMU : MeasurementType::NO_MEASUREMENT;
    else if (observable_avail)
        return _observable_queue->try_pop(observable_measurement) ? MeasurementType::GNSS : MeasurementType::NO_MEASUREMENT;

    return MeasurementType::NO_MEASUREMENT;  // no measurements available
}