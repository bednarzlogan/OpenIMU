#include <array>
#include <fstream>
#include <random>

#include "measurement_model.hpp"
#include "ukf_defs.hpp"

void noise_data(Observable& original_measurement) {
    // allocate output data
    MeasVec noised_measurement = original_measurement.observation;

    // in the initial sim, we're hardcoding a constand R
    original_measurement.R = get_simulated_measurement_noise();

    // define a static random number generator for reuse across function calls
    static std::mt19937 gen(std::random_device{}());

    for (uint8_t i = 0; i < Z; ++i) {
        double stddev = std::sqrt(original_measurement.R(i, i));
        std::normal_distribution<double> dist(0.0, stddev);
        noised_measurement(i) += dist(gen);
    }

    original_measurement.observation = noised_measurement;
}

// generic constructor to kickoff
TruthHandler::TruthHandler(): _running(true) {}

TruthHandler::~TruthHandler() {
    _running = false;
}

void TruthHandler::startStream(const std::string& path) {
    // open the path and begin reading the truth data lines
    std::ifstream file(path);
    std::string line;

    while (_running && getline(file, line)) {
        // allocate the truth data and populate the object
        Observable data;

        // if we found a valid measurement, push it!
        if (parse_line(line, data)) {
            // noise the measurement for initial validation
            noise_data(data);
            _observables_queue.push(data);
        }
    } 
    
    // done reading measurements
    _running = false;
}

bool TruthHandler::parse_line(const std::string& line, Observable& measurement) {
    // move to stringstream and perform splitting logic to get the data
    std::stringstream ss(line);
    std::string value;
    std::array<std::string, Z + 1> row;
    std::array<double, Z + 1> measurement_parts; // extra elem for the timestamp

    // check for header row
    if (line.rfind("timestamp", 0) == 0) return false;

    // interact with the stringstream to grab each word between delimiters
    uint8_t i = 0;
    while (std::getline(ss, value, ',')) {
        if (i >= Z + 1) {
            std::cerr << "Too many elements in CSV row!" << std::endl;
            return false;
        }
        try {
            measurement_parts[i] = std::stod(value);
        } catch (...) {
            return false;
        }
        i++;
    }

    // catch too few elements
    if (i != Z + 1) return false;

    measurement.timestamp = measurement_parts[0];  // log format spec
    for (int j = 0; j < Z; ++j)
        measurement.observation(j) = measurement_parts[j + 1];  // skip timestamp

    // TMP until verified
    std::cout << "Parsed truth row at t=" << measurement.timestamp << ", z_truth = " << measurement.observation.transpose() << "\n";

    return true;
}

bool TruthHandler::getNextTruth(Observable& measurement){
    if (!_running && _observables_queue.empty())
        return false;
    _observables_queue.wait_and_pop(measurement);
    return true;
}