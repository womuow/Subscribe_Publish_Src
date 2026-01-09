#include <queue>
#include <mutex>

template<typename T>
class DataBuffer {
private:
    std::queue<T> buffer_;
    std::mutex mutex_;
    T default_value_;
    size_t max_size_;

public:
    DataBuffer(const T& default_val, size_t max_size = 10) 
        : default_value_(default_val), max_size_(max_size) {}

    void push(const T& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() >= max_size_) {
            buffer_.pop();
        }
        buffer_.push(data);
    }

    bool try_pop(T& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            data = default_value_;
            return false;
        }
        data = buffer_.front();
        buffer_.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }
};