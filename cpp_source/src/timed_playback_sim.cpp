#include "timed_playback_sim.hpp"
#include "IMU_Matrices.hpp"
#include "ukf_defs.hpp"
#include <array>
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;
using namespace std::chrono;


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
    _imu_queue = std::make_unique<ThreadQueue<ImuData>>(max_measurements);
    _observable_queue = std::make_unique<ThreadQueue<Observable>>(max_measurements);

}

TimedPlaybackSim::TimedPlaybackSim(const std::string& config_path) 
    : _config_path(config_path), _current_time(0.0), _running(false) {
    load_configurations();
}


void TimedPlaybackSim::parse_line(const std::string& line, const std::string& source) {
    std::stringstream ss(line);
    std::string value;
    std::array<std::string, Z + 1> row;
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
            if (i >= N + 1) {
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
        if (i != N + 1) return;

        measurement.timestamp = measurement_parts[0];

        for (int j = 0; j < N; ++j) {
            imu_measurement.matrix_form_measurement(j) = measurement_parts[j + 1];
        }
        imu_measurement.measurement_time = measurement_parts[0];
        imu_measurement.updateFromMatrix();
        _imu_measurements->push(imu_measurement);
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
        if (i != 2 * Z + 1) return;

        measurement.timestamp = gnss_measurement_parts[0];
        for (int j = 0; j < Z; ++j) {
            measurement.observation(j) = gnss_measurement_parts[j + 1];
        }
        for (int j = 0; j < Z; ++j) {
            measurement.R(j, j) = gnss_measurement_parts[j + 1 + Z];
        }
        _observables->push(measurement);
    }
}

void TimedPlaybackSim::batcher_thread() {
    std::ifstream imu_file(_imu_data_path);
    std::ifstream gnss_file(_measurement_file_path);

    std::string imu_line, gnss_line;
    std::getline(imu_file, imu_line); // skip header
    std::getline(gnss_file, gnss_line); // skip header

    while (true) {
        if (!imu_file.eof() && std::getline(imu_file, imu_line))
            parse_line(imu_line, "imu");

        if (!gnss_file.eof() && std::getline(gnss_file, gnss_line))
            parse_line(gnss_line, "gnss");

        if (imu_file.eof() && gnss_file.eof()) break;
    }
}

void TimedPlaybackSim::queue_setter_timer() {
    // This function will set a timer based on the sample rate and push measurements to the respective queues
    // It will check the oldest measurement in the local memory structure and see if the sim time is >= the measurement time
    // If yes, it will push the measurement to the respective queue
    // This function will run in a separate thread
    // Define simulation time reference (starts at time of first measurement)

    double sim_start_time = -1.0;

    // System time reference point
    auto wall_clock_start = high_resolution_clock::now();

    while (_running) {
        auto now = high_resolution_clock::now();
        double sim_elapsed = duration<double>(now - wall_clock_start).count();
        ImuData imu_measurement;
        Observable observable_measurement;

        // check for available measurements and populate if available
        bool imu_available = _imu_measurements->try_pop(imu_measurement);
        bool observable_available = _observables->try_pop(observable_measurement);

        if (sim_start_time < 0.0) {
            // If we haven't set the sim start time, set it to the first measurement we have
            if (imu_available) {
                sim_start_time = imu_measurement.measurement_time;
            } else if (observable_available) {
                sim_start_time = observable_measurement.timestamp;
            } else {
                std::this_thread::sleep_for(milliseconds(10));  // Wait for data to arrive
                continue;
            }
        }

        double sim_now = sim_start_time + sim_elapsed;

        // push IMU measurements whose timestamp is <= sim_now
        while (imu_available && imu_measurement.measurement_time <= sim_now) {
            _imu_queue->push(imu_measurement);
        }

        // Push GNSS observables whose timestamp is <= sim_now
        while (observable_available && observable_measurement.timestamp <= sim_now) {
            _observable_queue->push(observable_measurement);
        }

        std::this_thread::sleep_for(milliseconds(10));  // avoid tight loop
    }
}


void TimedPlaybackSim::start_simulation() {
    // The processing loop is like this:
    // 1. Set a timer based on the fastest sensor sample rate
    // 2. Start a thread which will read measurements into a local memory structure while we wait for timer ticks
    // 3. At each timer tick, check the oldest measurement in the local memory structure and see if the sim time 
    //    is >= the measurement time; if yes, push the measurement to the respective queue
    
    // open the thread that batches in the measurements
    std::thread batcher_thread(&TimedPlaybackSim::batcher_thread, this); 

    // start a thread for the timer that will push measurements to the queues
    std::thread queue_setter_thread(&TimedPlaybackSim::queue_setter_timer, this);

    // wait for user interrupt or internal flag to end
    batcher_thread.join();
    queue_setter_thread.join();

    _running = false;  // stop the simulation
    std::cout << "Simulation stopped." << std::endl;
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