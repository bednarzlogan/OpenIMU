#pragma once

#include <fstream>
#include <array>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>

/**
 * @brief Compact log format for binary telemetry capture.
 * 
 * This header-only logger writes telemetry data using a CAN-like header 
 * and a CRC-16 checksum for basic data integrity. It supports templated 
 * logging of std::array<float, N> payloads with timestamping.
 */


/// @brief Supported payload data types.
enum class DataType : uint8_t {
    Float32 = 0b00,   ///< 4-byte IEEE 754 float
    Int16   = 0b01,   ///< 2-byte signed integer
    Float64 = 0b10,   ///< 8-byte double (rare, high-precision)
    Reserved = 0b11   ///< Reserved for future use
};



// ---- Packed Log Header Definition ----
#pragma pack(push, 1)
/**
 * @brief Packed binary header preceding each log entry.
 *
 * The structure is 9 bytes and includes timestamp, message ID, flags, and payload info.
 * No padding is inserted due to packing (verified by static_assert).
 */
struct LogHeader {
    uint32_t timestamp;  ///< Relative timestamp in microseconds since logger start
    uint16_t id;         ///< Message ID indicating message type (user-defined)
    uint8_t flags;       ///< EXTENDED (bit 0), data type (bits 1–2), reserved (3–7)
    uint8_t node_id;     ///< Node ID of message sender (user-defined)
    uint8_t dlc;         ///< Data Length Code: payload size in bytes
};
#pragma pack(pop)


static_assert(sizeof(LogHeader) == 9, "LogHeader must be 9 bytes packed.");


/**
 * @brief Compute a CRC-16-CCITT checksum over a buffer.
 *
 * @param data Pointer to the byte buffer to compute CRC for.
 * @param length Length of the buffer in bytes.
 * @param crc Initial CRC value (default = 0xFFFF).
 * @return uint16_t Final CRC value.
 */
 inline uint16_t crc16_ccitt(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


/**
 * @brief Logger for writing binary telemetry packets to a file.
 */
class Logger {
public:
    /**
     * @brief Construct a new Logger instance.
     * 
     * Opens the output file and writes a human-readable log start time header.
     * 
     * @param filename Path to the binary log file.
     */
    Logger(const std::string& log_file_name = "diag_log.bin")
        : start_time_(std::chrono::steady_clock::now()),
        wall_time_(std::chrono::system_clock::now())
    {
        out_.open(log_file_name, std::ios::binary);
        if (!out_.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }

        // Magic header constants
        const uint32_t magic = 0x46474F4C; // ASCII "LOGF"
        const uint8_t version = 1;
        const uint8_t endianness = 1; // 1 = little-endian, 0 = big-endian

        out_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
        out_.write(reinterpret_cast<const char*>(&version), sizeof(version));
        out_.write(reinterpret_cast<const char*>(&endianness), sizeof(endianness));

        // newline for separation before datetime string for cat output readability
        out_ << "\n";

        // we'll put in a precise UTC time header at the start of the log file
        // all other timestamps will be relative to this start time
        std::time_t time = std::chrono::system_clock::to_time_t(wall_time_);
        std::tm* tm = std::gmtime(&time);
        char buffer[128];
        std::strftime(buffer, sizeof(buffer), "# Log Start Time: %Y-%m-%d %H:%M:%S UTC\n", tm);
        out_ << buffer;
    }

    /**
     * @brief Flush any buffered log data to disk.
     */
     void flush() {
        out_.flush();
    }


    /**
     * @brief Clean up logger and flush all data on destruction.
     */
    ~Logger() {
        flush();
    }


    /**
     * @brief Write a binary telemetry message to the log.
     *
     * @tparam N Number of floats in the payload array.
     * @param id Message ID (used to identify message type).
     * @param type Payload data type (e.g., float32).
     * @param node_id Node that originated the message.
     * @param payload The payload data as a float array.
     */
    template <size_t N>
    void logMessage(uint16_t id, DataType type, uint8_t node_id, const std::array<float, N>& payload) {
        // get timing info 
        auto now = std::chrono::steady_clock::now();
        auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count();

        LogHeader header;
        header.timestamp = static_cast<uint32_t>(timestamp_us);
        header.id = id;
        header.flags = static_cast<uint8_t>(type) << 1; // bits 1-2 hold data type
        header.node_id = node_id;
        header.dlc = static_cast<uint8_t>(sizeof(float) * N);

        // Write header and payload into a buffer to compute CRC
        std::vector<uint8_t> buffer(sizeof(header) + header.dlc);
        std::memcpy(buffer.data(), &header, sizeof(header));
        std::memcpy(buffer.data() + sizeof(header), payload.data(), header.dlc);

        // Compute CRC over header + payload
        uint16_t crc = crc16_ccitt(buffer.data(), buffer.size());

        // Write full record to file
        out_.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
        out_.write(reinterpret_cast<const char*>(&crc), sizeof(crc));
    }


private:
    std::ofstream out_;  ///< Output stream for binary file writing
    std::chrono::steady_clock::time_point start_time_;  ///< Time reference for timestamps
    std::chrono::system_clock::time_point wall_time_;   ///< Real-world time for header
};