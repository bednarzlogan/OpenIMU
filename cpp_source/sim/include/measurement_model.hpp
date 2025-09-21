#include <string>
#include <atomic>

#include "logger.hpp"
#include "thread_safe_queue.hpp"
#include "ukf_defs.hpp"

static constexpr double kPosStd = 1.0;
static constexpr double kVelStd = 0.1;
static constexpr double kPosVar = kPosStd * kPosStd;
static constexpr double kVelVar = kVelStd * kVelStd;

inline static MeasCov get_simulated_measurement_noise() {
    MeasCov R = MeasCov::Zero();
    uint8_t size = Z/2;
    for (int i = 0; i < size; ++i) {
        R(i, i) = kPosVar;
        R(i + size, i + size) = kVelVar;
    }
    return R;
}

class TruthHandler {
public:
    TruthHandler(std::shared_ptr<Logger> diag_logger);
    ~TruthHandler();

    void startStream(const std::string& path);
    bool getNextTruth(Observable& out);
private:
    ThreadQueue<Observable> _observables_queue;
    std::shared_ptr<Logger> _diag_logger;
    std::atomic<bool> _running;

    bool parse_line(const std::string& line, Observable& measurement);
};
