#pragma once
#include <fstream>
#include <iostream>
#include <string>

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
            std::cout << "[" + topic + "]   " << sub << std::endl;
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