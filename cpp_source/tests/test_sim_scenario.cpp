// tests/test_sim_scenario.cpp
#include <gtest/gtest.h>
#include <thread>
#include <filesystem>
#include "UKF.hpp"
#include "timed_playback_sim.hpp"

using namespace std::chrono_literals;

static ImuData makeImu(double t) {
  ImuData m{};
  m.measurement_time = t;
  m.matrix_form_measurement.setZero(); // not used here
  m.updateFromMatrix();
  return m;
}

TEST(SimScenario, Fifty_Meter_Loop) {
  // Arrange
  auto ukf = std::make_shared<UKF>("config/system_constants.json");

  // Light, readable initializer; tune however you like
  StateVec x0 = StateVec::Zero();
  CovMat  P0  = CovMat::Zero();
  P0.diagonal() <<
      0.5, 0.5, 0.5,
      0.1, 0.1, 0.1,
      0.25, 0.25, 0.25,
      1e-4, 1e-4, 1e-4,
      1e-4, 1e-4, 1e-4;
  ukf->initialize(x0, P0);

  TimedPlaybackSim sim("config/rt_playback_conf.json");

  const auto period = std::chrono::milliseconds(10); // worker tick; sim uses file timestamps anyway

  // Act: run sim in a worker
  std::thread th([&]{ sim.start_simulation(ukf, period); });

  // Wait for completion (pick one depending on your sim behavior)
  // A) if start_simulation() returns at EOF:

  // Wait up to 5 minutes; abort cleanly if it runs long
  const auto deadline = std::chrono::steady_clock::now() + 300s;
  while (sim.is_running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  if (sim.is_running()) {
    sim.stop_simulation();
    ukf->stop_filter();
    th.join();
    FAIL() << "scenario timed out";
  }
  th.join();
  // B) else poll the finished() flag with timeout:
  // for (int i = 0; i < 500 && !sim.finished(); ++i) std::this_thread::sleep_for(2ms);
  // if (!sim.finished()) { sim.stop_simulation(); FAIL() << "scenario timed out"; }

  // Assert: filter reached end time and ended near expected endpoint (from scenario config)
  // Suppose your scenario doc (JSON) includes "expected_final_enu": [E, N, U] and "pos_tol_m": 1.0
  // If you don't parse JSON here, hardcode the expected for now:
  const Eigen::Vector3d expected_E_N_U(44.15,-0.11, 0.0);  // endpoint of the log for this test
  const double pos_tol = 1.0;  // TODO this should be set from the lower bound of the estimator

  const auto& xN = ukf->get_state();            // N-vector
  const Eigen::Vector3d pos = xN.template segment<3>(0); // assuming first 3 are position

  // make sure we hit the endpoint
  EXPECT_NEAR(pos.x(), expected_E_N_U.x(), pos_tol);
  EXPECT_NEAR(pos.y(), expected_E_N_U.y(), pos_tol);
  EXPECT_NEAR(pos.z(), expected_E_N_U.z(), pos_tol);

  // make sure we mark the right sim end time
  EXPECT_GE(ukf->solution_time(), 882 - 1e-6);

  // ensure that a log larger than 0 bytes was written
  const std::filesystem::path logger_log = sim.get_log_path();  
  EXPECT_TRUE(std::filesystem::is_regular_file(logger_log));
  EXPECT_GT(std::filesystem::file_size(logger_log), 0u);

  // Clean up
  ukf->stop_filter(); // if not already stopped inside sim
}
