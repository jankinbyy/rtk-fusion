#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
static std::string GetCurrentTimeStamp(int time_stamp_type = 0) {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time_t);

    char buffer[128];
    strftime(buffer, sizeof(buffer), "%F %T", now_tm);

    std::ostringstream ss;
    ss.fill('0');

    std::chrono::milliseconds ms;
    std::chrono::microseconds cs;
    std::chrono::nanoseconds ns;

    switch (time_stamp_type) {
    case 0:
        ss << buffer;
        break;
    case 1:
        ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        ss << buffer << ":" << ms.count();
        break;
    case 2:
        ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000;
        break;
    case 3:
        ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000000000;
        ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000 << ":" << ns.count() % 1000;
        break;
    default:
        ss << buffer;
        break;
    }

    return ss.str();
}
class FileWriter {
private:
    bool use_file = false;

public:
    std::ofstream file_stream;
    FileWriter() {
    }
    void Init(const std::string &filename) {
        use_file = true;
        if (use_file) {
            file_stream.open(filename, std::ios::out);
            file_stream.close();
            file_stream.open(filename, std::ios::app);
            if (!file_stream.is_open()) {
                std::cerr << "错误：无法打开文件 " << filename << std::endl;
                use_file = false;
            } else {
                file_stream.setf(std::ios::fixed, std::ios::floatfield);
                file_stream.precision(9);
            }
        }
    }
    void Init(const std::string &filename, bool use) {
        use_file = use;
        if (use_file) {
            file_stream.open(filename, std::ios::out);
            file_stream.close();
            file_stream.open(filename, std::ios::app);
            if (!file_stream.is_open()) {
                std::cerr << "错误：无法打开文件 " << filename << std::endl;
                use_file = false;
            } else {
                file_stream.setf(std::ios::fixed, std::ios::floatfield);
                file_stream.precision(9);
            }
        }
    }

    ~FileWriter() {
        if (use_file && file_stream.is_open()) {
            file_stream.close();
        }
    }

    template <typename T>
    friend FileWriter &operator<<(FileWriter &file_wrt, const T &data) {
        if (file_wrt.use_file && file_wrt.file_stream.is_open()) {
            file_wrt.file_stream << data;
        }
        return file_wrt;
    }
};

class DebugMode {
public:
    DebugMode() {
    }
    void Init(const std::string &_topic, int _level,
              const std::string &_file_name) {
        topic = _topic;
        level = _level;
        if (level < MAX_FILE_LEVEL) {
            file_write.Init(_file_name, true);
        }
    }

    void operator()(int in_level, const std::string &sub) {
        if (level != -1 && in_level > level - 1) {
            std::cout << GetCurrentTimeStamp(1) << " [" + topic + "]   " << sub << std::endl;
        }
    }

    template <typename T>
    friend DebugMode &operator<<(DebugMode &debug_mode, const T &data) {
        debug_mode.file_write << data;
        return debug_mode;
    }
    // Overload for std::endl
    friend DebugMode &operator<<(DebugMode &debug_mode,
                                 std::ostream &(*manipulator)(std::ostream &)) {
        manipulator(debug_mode.file_write.file_stream);
        // manipulator(std::cout);
        return debug_mode;
    }

private:
    std::string topic;
    int level = -1;
    FileWriter file_write;
    static const int MAX_FILE_LEVEL = 4;
};