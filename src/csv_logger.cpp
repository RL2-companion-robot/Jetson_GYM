#include "csv_logger.h"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

AsyncCsvLogger::AsyncCsvLogger()
    : flush_interval_(std::chrono::milliseconds(500))
    , running_(false)
    , stop_requested_(false) {
}

AsyncCsvLogger::~AsyncCsvLogger() {
    stop();
}

bool AsyncCsvLogger::start(const std::string& file_path, std::chrono::milliseconds flush_interval) {
    stop();

    file_.open(file_path);
    if (!file_.is_open()) {
        return false;
    }

    file_path_ = file_path;
    flush_interval_ = flush_interval;
    running_ = true;
    stop_requested_ = false;
    writeHeader();
    worker_ = std::thread(&AsyncCsvLogger::workerLoop, this);
    return true;
}

void AsyncCsvLogger::enqueue(CsvLogRecord record) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return;
    }
    queue_.push_back(std::move(record));
    cv_.notify_one();
}

void AsyncCsvLogger::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        stop_requested_ = true;
        cv_.notify_all();
    }

    if (worker_.joinable()) {
        worker_.join();
    }

    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }

    running_ = false;
    stop_requested_ = false;
    queue_.clear();
}

const std::string& AsyncCsvLogger::filePath() const {
    return file_path_;
}

void AsyncCsvLogger::workerLoop() {
    auto next_flush_deadline = std::chrono::steady_clock::now() + flush_interval_;

    while (true) {
        std::vector<CsvLogRecord> batch;
        bool should_stop = false;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_until(lock, next_flush_deadline, [&]() {
                return stop_requested_ || !queue_.empty();
            });

            if (!queue_.empty()) {
                batch.reserve(queue_.size());
                while (!queue_.empty()) {
                    batch.push_back(std::move(queue_.front()));
                    queue_.pop_front();
                }
            }

            should_stop = stop_requested_ && queue_.empty() && batch.empty();
        }

        for (const auto& record : batch) {
            writeRecord(record);
        }

        auto now = std::chrono::steady_clock::now();
        if (now >= next_flush_deadline || (!batch.empty() && stop_requested_)) {
            file_.flush();
            next_flush_deadline = now + flush_interval_;
        }

        if (should_stop) {
            break;
        }
    }

    file_.flush();
}

void AsyncCsvLogger::writeHeader() {
    file_ << "timestamp_local"
          << ",record_type"
          << ",event_type"
          << ",event_msg";

    for (int i = 0; i < 10; ++i) file_ << ",q_exp_" << i;
    for (int i = 0; i < 10; ++i) file_ << ",dq_exp_" << i;
    for (int i = 0; i < 10; ++i) file_ << ",tau_exp_" << i;

    for (int i = 0; i < 10; ++i) file_ << ",q_" << i;
    for (int i = 0; i < 10; ++i) file_ << ",dq_" << i;
    for (int i = 0; i < 10; ++i) file_ << ",tau_" << i;

    file_ << ",omega_x,omega_y,omega_z"
          << ",acc_x,acc_y,acc_z"
          << ",roll,pitch,yaw"
          << ",quat_w,quat_x,quat_y,quat_z"
          << "\n";
}

void AsyncCsvLogger::writeRecord(const CsvLogRecord& record) {
    file_ << escapeCsv(formatTimestamp(record.timestamp))
          << "," << escapeCsv(record.record_type)
          << "," << escapeCsv(record.event_type)
          << "," << escapeCsv(record.event_msg);

    for (float value : record.q_exp) file_ << "," << value;
    for (float value : record.dq_exp) file_ << "," << value;
    for (float value : record.tau_exp) file_ << "," << value;

    for (float value : record.q) file_ << "," << value;
    for (float value : record.dq) file_ << "," << value;
    for (float value : record.tau) file_ << "," << value;

    for (float value : record.omega) file_ << "," << value;
    for (float value : record.acc) file_ << "," << value;
    for (float value : record.eu_ang) file_ << "," << value;
    for (float value : record.quat) file_ << "," << value;

    file_ << "\n";
}

std::string AsyncCsvLogger::formatTimestamp(const std::chrono::system_clock::time_point& timestamp) const {
    const auto time_t_value = std::chrono::system_clock::to_time_t(timestamp);
    std::tm local_tm;
    localtime_r(&time_t_value, &local_tm);

    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp.time_since_epoch()) % std::chrono::seconds(1);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0') << micros.count();
    return oss.str();
}

std::string AsyncCsvLogger::escapeCsv(const std::string& value) const {
    if (value.find_first_of(",\"\n") == std::string::npos) {
        return value;
    }

    std::string escaped = "\"";
    for (char ch : value) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += "\"";
    return escaped;
}
