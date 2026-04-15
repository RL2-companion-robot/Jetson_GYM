#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

struct CsvLogRecord {
    std::chrono::system_clock::time_point timestamp;
    std::string record_type;
    std::string event_type;
    std::string event_msg;

    std::array<float, 10> q_exp{};
    std::array<float, 10> dq_exp{};
    std::array<float, 10> tau_exp{};

    std::array<float, 10> q{};
    std::array<float, 10> dq{};
    std::array<float, 10> tau{};

    std::array<float, 3> omega{};
    std::array<float, 3> acc{};
    std::array<float, 3> eu_ang_raw{};
    std::array<float, 3> eu_ang{};
    std::array<float, 4> quat{};
};

class AsyncCsvLogger {
public:
    AsyncCsvLogger();
    ~AsyncCsvLogger();

    bool start(const std::string& file_path,
               std::chrono::milliseconds flush_interval = std::chrono::milliseconds(500));
    void enqueue(CsvLogRecord record);
    void stop();

    const std::string& filePath() const;

private:
    void workerLoop();
    void writeHeader();
    void writeRecord(const CsvLogRecord& record);
    std::string formatTimestamp(const std::chrono::system_clock::time_point& timestamp) const;
    std::string escapeCsv(const std::string& value) const;

    std::ofstream file_;
    std::string file_path_;
    std::chrono::milliseconds flush_interval_;
    std::deque<CsvLogRecord> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    bool running_;
    bool stop_requested_;
};

#endif // CSV_LOGGER_H
