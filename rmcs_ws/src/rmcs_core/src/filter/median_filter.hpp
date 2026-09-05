// filter/median_filter.hpp
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rmcs_core::filter {

// 中值滤波：对最近 window 个样本取排序中值输出。
// ---------------------------------------------------------------------------
// 适用：偶发尖刺/离群点(如编码器差分测速的跳变)。尖刺是孤立离群值，
//       取中值会直接把它丢掉；不像低通会把尖刺摊开成一段拖尾。
// window 建议奇数(3/5/7)：越大越钝、对真实跳变响应越慢。
// 启动前几拍窗口未满时，取"已收样本"的中值，避免输出突然跳 0。
// ---------------------------------------------------------------------------
class MedianFilter {
public:
    // window 必须是奇数(3/5/7…)
    explicit MedianFilter(std::size_t window)
        : window_(window) {
        buffer_.reserve(window);
    }

    // 推一个新样本 → 返回窗口的中值
    double update(double sample) {
        if (buffer_.size() < window_) {
            buffer_.push_back(sample);              // 窗口未满：直接收
        } else {
            buffer_.erase(buffer_.begin());         // 已满：丢最旧
            buffer_.push_back(sample);              // 收最新(保持到达序)
        }
        ++count_;

        // 复制排序取中值；窗口未满时 size<window → 取已收样本的中值(不跳0)
        std::vector<double> sorted = buffer_;
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    void reset() {
        buffer_.clear();
        count_ = 0;
    }

private:
    std::vector<double> buffer_;   // 存最近 min(count_, window) 个样本(到达序)
    std::size_t window_;
    std::size_t count_ = 0;        // 已收样本数(窗口未满时用)
};
} // namespace rmcs_core::filter
