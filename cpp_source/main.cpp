// main.cpp
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include "UKF.hpp"
#include "ukf_defs.hpp"          // for N, StateVec, CovMat if not re-exported by UKF.hpp
#include "timed_playback_sim.hpp"

using namespace std::chrono_literals;

static void run_playback(const std::shared_ptr<Estimator>& estimator,
                         TimedPlaybackSim& sim,
                         std::chrono::milliseconds period)
{
    // blocking: returns when sim.stop_simulation() is called
    sim.start_simulation(estimator, period);
}

int main(int argc, char** argv) {
    // defaults (CMake copies config/ next to the binary)
    std::string ukf_config_path = "config/ukf_minimal.json";
    std::string sim_config_path = "config/rt_playback_conf.json";
    std::chrono::milliseconds period{20}; // 50 Hz

    // ultra-light CLI: ukf_playback [ukf_config] [sim_config] [period_ms]
    if (argc > 1) ukf_config_path = argv[1];
    if (argc > 2) sim_config_path = argv[2];
    if (argc > 3) {
        try { period = std::chrono::milliseconds(std::stoi(argv[3])); }
        catch (...) { std::cerr << "WARN: bad period arg; keeping 20 ms.\n"; }
    }

    // build estimator (UKF) and seed a small initial covariance/state
    auto estimator = std::make_shared<UKF>(ukf_config_path);

    StateVec x0 = StateVec::Zero();
    CovMat  P0  = CovMat::Zero();
    // light, readable diagonal (tune however you like)
    // positions, velocities, attitude, accel bias, gyro bias:
    P0.diagonal() <<
        0.5, 0.5, 0.5,
        0.1, 0.1, 0.1,
        0.25, 0.25, 0.25,
        1e-4, 1e-4, 1e-4,
        1e-4, 1e-4, 1e-4;

    estimator->initialize(x0, P0);

    // timed playback sim reads its own JSON (CSV paths, rates, etc.)
    TimedPlaybackSim sim(sim_config_path);

    std::cout << "Starting playback with UKF config: " << ukf_config_path
              << "\nSim config: " << sim_config_path
              << "\nUpdate period: " << period.count() << " ms\n"
              << "Press 'q' + Enter to stop.\n";

    // run the sim in a worker so we can watch stdin here
    std::thread worker(run_playback, estimator, std::ref(sim), period);

    // simple shutdown loop
    for (char c; std::cin.get(c); ) {
        if (c == 'q' || c == 'Q') {
            std::cout << "Stopping...\n";
            // stop producers first, then consumer, then join
            sim.stop_simulation();
            estimator->stop_filter();
            worker.join();
            break;
        }
    }

    return 0;
}
