// tests/test_retrodict.cpp
#include <gtest/gtest.h>
#include "UKF.hpp"
#include <thread>

static ImuData makeImu(double t,
                       double ax=0,double ay=0,double az=0,
                       double wx=0,double wy=0,double wz=0) {
  ImuData m{};
  m.measurement_time = t;
  m.matrix_form_measurement << ax, ay, az, wx, wy, wz; // assuming M==6
                                                       // may want to just request a control input + timestamp
  m.updateFromMatrix();
  return m;
}

static Observable makeGnss(double t, double sigma=1.0) {
  // since this is a retrodict test, we only care about timing accuracy
  // of the code, not the actual numeric solution
  Observable g{};
  g.timestamp = t;
  g.observation.setZero();
  g.R.setZero();
  for (int i=0;i<Z;++i) g.R(i,i) = sigma*sigma;
  return g;
}

static void wait_until(std::function<bool()> pred, int ms_timeout = 200) {
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  while (!pred()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (clock::now() - t0 > std::chrono::milliseconds(ms_timeout)) break;
  }
}

TEST(UKF_Retrodict, RollbackReplayBasic) {
  UKF ukf("config/system_constants.json");
  ukf.start_filter(std::chrono::milliseconds(2));

  // history capacity + sampling setup
  const uint8_t cap = static_cast<int>(ukf.get_hist_cap());   // e.g., 50
  const double fs = 50.0;                                 // Hz
  const double dt = 1.0 / fs;

  // fill history exactly to capacity: times 0, dt, ..., (cap-1)*dt
  for (int k = 1; k < cap; ++k) {
    ukf.read_imu(makeImu(k * dt));
  }

  // choose tz strictly inside the history window to force replay
  // *** WAIT FIRST *** (give the worker time to turn IMU into snapshots)
  wait_until([&]{
    return ukf.history_size() >= 5u;           // or == cap if you prefer
    // or: return ukf.solution_time() >= (cap-1)*dt - 1e-9;
  }, 5 * 60 * 1000); // up to 5 minutes while you debug

  ASSERT_GT(ukf.history_size(), 0u);
  const double t_front = ukf.hist_front_time();
  const double t_back  = ukf.hist_back_time();
  ASSERT_LT(t_front, t_back); // sanity

  const double tz = t_front + 0.5 * (t_back - t_front);   // midpoint in seconds

  // we'll require that history exists and tz is inside [front, back] before we feed GNSS.
  // (If this ever fails, adjust the number of IMU samples or hist_cap in the UKF.)
  ASSERT_LE(t_front - 1e-9, tz);
  ASSERT_GE(t_back  + 1e-9, tz);

  // debug print
  std::cerr << "front=" << ukf.hist_front_time()
            << " back="  << ukf.hist_back_time()
            << " tz="    << tz
            << " size="  << ukf.history_size() << "\n";


  // GNSS at tz s (inside history)
  ukf.read_gps(makeGnss(tz));

  // wait until GNSS at tz is fused (avoid fixed sleeps)
  wait_until([&]{ return std::abs(ukf.last_gnss_time() - tz) < 1e-9; });

  // after replay, solution should be at the tip (~ last IMU time, 1.00 s)
  EXPECT_NEAR(ukf.solution_time(), ukf.hist_back_time(), 1e-9);
  EXPECT_GE(ukf.last_replay_steps(), 1u);

  ukf.stop_filter();
}
