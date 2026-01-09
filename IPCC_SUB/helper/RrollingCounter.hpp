#include <iostream>
#include <stdexcept>

namespace HBC_HELPER
{

class SteppingRollingCounter {
private:
    uint32_t current_;
    uint32_t min_;
    uint32_t max_;
    uint32_t step_;
    
public:
    SteppingRollingCounter(uint32_t min = 0, uint32_t max = 99, uint32_t start = 0, uint32_t step = 1) 
        : min_(min), max_(max), step_(step) {
        if (min >= max) {
            throw std::invalid_argument("min must be less than max");
        }
        if (step <= 0) {
            throw std::invalid_argument("step must be positive");
        }
        current_ = (start >= min && start <= max) ? start : min;
    }
    
    SteppingRollingCounter& operator++() {
        current_ += step_;
        if (current_ > max_) {
            current_ = min_ + (current_ - max_ - 1) % (max_ - min_ + 1);
        }
        return *this;
    }
    
    SteppingRollingCounter operator++(int) {
        SteppingRollingCounter temp = *this;
        ++(*this);
        return temp;
    }
    
    SteppingRollingCounter& operator--() {
        current_ -= step_;
        if (current_ < min_) {
            uint32_t overflow = min_ - current_ - 1;
            current_ = max_ - (overflow % (max_ - min_ + 1));
        }
        return *this;
    }
    
    SteppingRollingCounter operator--(int) {
        SteppingRollingCounter temp = *this;
        --(*this);
        return temp;
    }
    
    SteppingRollingCounter& add(uint32_t value) {
        if (value > 0) {
            for (uint32_t i = 0; i < value; ++i) {
                ++(*this);
            }
        } else if (value < 0) {
            for (uint32_t i = 0; i < -value; ++i) {
                --(*this);
            }
        }
        return *this;
    }
    
    operator uint32_t() const { return current_; }
    uint32_t value() const { return current_; }
    uint32_t min() const { return min_; }
    uint32_t max() const { return max_; }
    uint32_t step() const { return step_; }
    
    void reset(uint32_t value = 0) {
        current_ = (value >= min_ && value <= max_) ? value : min_;
    }
};

}