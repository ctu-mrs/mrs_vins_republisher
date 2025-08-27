#pragma once
#include <chrono>
#include <atomic>
#include <iostream>

/**
 * Simple rate limiter for logging/printing messages.
 * Thread-safe and header-only.
 */
class RateLimiter {
private:
    mutable std::atomic<std::chrono::steady_clock::time_point> last_print_;
    mutable std::atomic<int> suppressed_count_;
    const std::chrono::milliseconds interval_;

    // ANSI color codes
    static constexpr const char* RED = "\033[31m";
    static constexpr const char* YELLOW = "\033[33m";
    static constexpr const char* RESET = "\033[0m";

public:
    /**
     * Constructor
     * @param interval_ms Minimum interval between prints in milliseconds (default: 5000ms = 5s)
     */
    explicit RateLimiter(int interval_ms = 5000)
        : last_print_(std::chrono::steady_clock::time_point{})
        , suppressed_count_(0)
        , interval_(interval_ms) {}

    /**
     * Attempt to print. Returns true if enough time has passed.
     * @return true if should print, false if rate limited
     */
    bool shouldPrint() const {
        auto now = std::chrono::steady_clock::now();
        auto last = last_print_.load();

        if (now - last >= interval_) {
            // Try to update the time atomically
            if (last_print_.compare_exchange_strong(last, now)) {
                return true;
            }
        }

        // Rate limited - increment suppressed counter
        suppressed_count_.fetch_add(1);
        return false;
    }

    /**
     * Get and reset the count of suppressed messages.
     * @return Number of messages that were suppressed since last print
     */
    int getSuppressedCount() const {
        return suppressed_count_.exchange(0);
    }

    /**
     * Template function to print with automatic rate limiting and suppression info.
     * Usage: limiter.print("Error at position", x, y);
     */
    template<typename... Args>
    void print(Args&&... args) const {
        printTo(std::clog, nullptr, std::forward<Args>(args)...);
    }

    /**
     * Template function to print to a specific stream with rate limiting.
     * Usage: limiter.printTo(std::cerr, "Error at position", x, y);
     */
    template<typename... Args>
    void printTo(std::ostream& stream, Args&&... args) const {
        printTo(stream, nullptr, std::forward<Args>(args)...);
    }

    /**
     * Template function to print with color and rate limiting.
     * Usage: limiter.printWithColor(std::cerr, RED, "Error at position", x, y);
     */
    template<typename... Args>
    void printWithColor(std::ostream& stream, const char* color, Args&&... args) const {
        printTo(stream, color, std::forward<Args>(args)...);
    }

private:
    template<typename... Args>
    void printTo(std::ostream& stream, const char* color, Args&&... args) const {
        if (shouldPrint()) {
            int suppressed = getSuppressedCount();

            // Print color code if specified
            if (color) stream << color;

            // Print the message
            ((stream << args << " "), ...);

            // // Add suppression info if any
            // if (suppressed > 0) {
            //     stream << "(" << suppressed << " similar messages suppressed)";
            // }

            // Reset color if specified
            if (color) stream << RESET;

            stream << std::endl;
        }
    }

public:

    /**
     * Reset the rate limiter (force next message to print)
     */
    void reset() {
        last_print_.store(std::chrono::steady_clock::time_point{});
        suppressed_count_.store(0);
    }
};

/**
 * Macro for easy static rate limiter usage.
 * Usage: RATE_LIMITED_LOG(5000, "Error:", value, "at", x, y);
 */
#define RATE_LIMITED_LOG(interval_ms, ...) \
    do { \
        static RateLimiter limiter(interval_ms); \
        limiter.print(__VA_ARGS__); \
    } while(0)

/**
 * Macro for error messages to stderr in red.
 * Usage: RATE_LIMITED_ERROR(5000, "Error:", value, "at", x, y);
 */
#define RATE_LIMITED_ERROR(interval_ms, ...) \
    do { \
        static RateLimiter limiter(interval_ms); \
        limiter.printWithColor(std::cerr, "\033[31m", __VA_ARGS__); \
    } while(0)

/**
 * Macro for warning messages in yellow.
 * Usage: RATE_LIMITED_WARNING(5000, "Warning:", value, "at", x, y);
 */
#define RATE_LIMITED_WARNING(interval_ms, ...) \
    do { \
        static RateLimiter limiter(interval_ms); \
        limiter.printWithColor(std::clog, "\033[33m", __VA_ARGS__); \
    } while(0)
