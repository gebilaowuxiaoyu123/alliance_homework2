#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rmcs_core::filter {

class MedianFilter {
public:
    // window 须为奇数(3/5/7…)
    explicit MedianFilter(std::size_t window)
        : window_(window) {
        buffer_.reserve(window);
    }

    // 返回窗口内样本的中值(窗口未满时取已收样本的中值)
    double update(double sample) {
        if (buffer_.size() < window_) {
            buffer_.push_back(sample);
        } else {
            buffer_.erase(buffer_.begin());
            buffer_.push_back(sample);
        }
        ++count_;

        std::vector<double> sorted = buffer_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    void reset() {
        buffer_.clear();
        count_ = 0;
    }

private:
    std::vector<double> buffer_;
    std::size_t window_;
    std::size_t count_ = 0;
};

} // namespace rmcs_core::filter
