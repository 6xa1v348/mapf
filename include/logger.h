#pragma once
#include "common.h"


enum class LogLevel {DEBUG, INFO, WARN, ERROR};

class Logger {
    private:
        bool verbose;
        bool fileLogging;
        std::ofstream logFile;
        std::mutex mtx;

        Logger() : verbose(true), fileLogging(false) {}
        ~Logger() {
            if (logFile.is_open()) logFile.close();
        }
        Logger(const Logger&) = delete;
        Logger& operator=(const Logger&) = delete;

        std::string levelToStr(LogLevel lvl) {
            switch (lvl) {
                case LogLevel::DEBUG : return "DEBUG";
                case LogLevel::INFO : return "INFO";
                case LogLevel::WARN : return "WARN";
                case LogLevel::ERROR: return "ERROR";
            }
            return "";
        }

        void write(LogLevel lvl, const std::string& name, const std::string& msg) {
            std::string output = "[" + levelToStr(lvl) + "] " + name + " : ";
            output += msg + "\n";

            if (verbose || lvl == LogLevel::ERROR) {
                std::cout << output;
                std::cout.flush();
            }
            if (fileLogging && logFile.is_open()) {
                logFile << output;
                logFile.flush();
            }
            if (lvl == LogLevel::ERROR) std::exit(1);
        }

    public:
        static Logger& get() {
            static Logger instance;
            return instance;
        }

        void setVerbose(bool enabled) {
            verbose = enabled;
        }

        void enableFileLogging(const std::string& filename = "output.log") {
            std::lock_guard<std::mutex> lock(mtx);
            logFile.open(filename, std::ios::out | std::ios::app);
            fileLogging = logFile.is_open();
        }

        void disableFileLogging() {
            std::lock_guard<std::mutex> lock(mtx);
            if (logFile.is_open()) logFile.close();
            fileLogging = false;
        }

        void log(LogLevel lvl, const std::string& name, const std::string& msg) {
            std::lock_guard<std::mutex> lock(mtx);
            write(lvl, name, msg);
        }

        void log(LogLevel lvl, const std::string& name, const std::string& msg, const Time::time_point& t) {
            std::lock_guard<std::mutex> lock(mtx);
            int elapsed = getElapsedTime(t);
            write(lvl, name, msg + " (" + std::to_string(elapsed) + " ms)");
        }
};


#define MAKE_LOG_FN(CLASSNAME, FN_NAME, LEVEL) \
    template <typename... Args> \
    void FN_NAME(const std::string& msg, Args&&... args) const { \
        Logger::get().log(LogLevel::LEVEL, #CLASSNAME, msg, std::forward<Args>(args)...); \
    }

#define LOGGER(CLASSNAME) \
    MAKE_LOG_FN(CLASSNAME, debug, DEBUG) \
    MAKE_LOG_FN(CLASSNAME, info, INFO) \
    MAKE_LOG_FN(CLASSNAME, warn, WARN) \
    MAKE_LOG_FN(CLASSNAME, error, ERROR)
