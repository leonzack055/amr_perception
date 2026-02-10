#ifndef AMR_REFLECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP
#define AMR_REFLECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP
#include <algorithm>
#include <deque>
#include <vector>
#include <optional>

namespace amr_reflector_noise_handling {

template <typename T> class TimestampedData {
public:
  TimestampedData(int64_t ts = 0) : timestamp(ts) {}
  TimestampedData(const T &d, int64_t ts) : data(d), timestamp(ts) {}
  T data;
  int64_t timestamp;
  // Comparison operators for sorting
  bool operator<(const TimestampedData &other) const {
    return timestamp < other.timestamp;
  }

  bool operator<=(const TimestampedData &other) const {
    return timestamp <= other.timestamp;
  }

  bool operator>(const TimestampedData &other) const {
    return timestamp > other.timestamp;
  }

  bool operator>=(const TimestampedData &other) const {
    return timestamp >= other.timestamp;
  }

  bool operator==(const TimestampedData &other) const {
    return timestamp == other.timestamp;
  }
};

template <typename T> class TimeOrderQueue {
private:
  std::deque<TimestampedData<T>> queue; // 升序插入双端队列

public:
  // 有序插入数据，对于按顺序持续插入为O(N)
  void push(const TimestampedData<T> &item) {
    auto it = std::lower_bound(queue.begin(), queue.end(), item);
    queue.insert(it, item);
  }

  // 将指定时间戳前的数据全部弹出，弹出的数据按时间排序返回
  std::vector<TimestampedData<T>> popBefore(int64_t cutoff_time) {
    std::vector<TimestampedData<T>> popped_items;
    auto it = std::lower_bound(queue.begin(), queue.end(),
                               TimestampedData<T>(cutoff_time));
    if (it != queue.begin()) {
      popped_items.assign(queue.begin(), it);
      queue.erase(queue.begin(), it);
    }
    return popped_items;
  }

  // 获取指定时间段内的元素 [start_time, end_time]
  std::vector<TimestampedData<T>> getRange(int64_t start_time,
                                           int64_t end_time) const {
    std::vector<TimestampedData<T>> result;
    // Find first element >= start_time
    auto start_it = std::lower_bound(queue.begin(), queue.end(),
                                     TimestampedData<T>(start_time));
    // 没有元素大于startTime，没有办法提取有效数据
    if (start_it == queue.end())
      return result;
    // Find first element > end_time
    // 没有元素大于end_time可能有数据缓冲时刻没到，没法形成严格的end_time
    auto end_it = std::upper_bound(queue.begin(), queue.end(),
                                   TimestampedData<T>(end_time));
    if (end_it == queue.end())
      return result;
    // STL 规则左开右闭，这个队列在[start_time, end_time]的元素
    result.assign(start_it, end_it);

    return result;
  }

  // Get elements as a new queue within time range
  TimeOrderQueue<T> getRangeQueue(int64_t start_time, int64_t end_time) const {
    TimeOrderQueue<T> result;
    // Find first element >= start_time
    auto start_it = std::lower_bound(queue.begin(), queue.end(),
                                     TimestampedData<T>(start_time));
    if (start_it == queue.end())
      return result;
    if (start_it == queue.begin())
      return result;
    // Find first element > end_time
    auto end_it = std::upper_bound(queue.begin(), queue.end(),
                                   TimestampedData<T>(end_time));
    if (end_it == queue.begin())
      return result;
    // Add elements in range [start_it, end_it)
    for (auto it = start_it; it != end_it; ++it) {
      result.push(*it);
    }
    return result;
  }

  TimestampedData<T> pop_front() {
    TimestampedData<T> front_data = std::move(queue.front());
    queue.pop_front(); // deque/list的头部删除接口
    return front_data;
  }
  TimestampedData<T> pop_back() {
    TimestampedData<T> back_data = std::move(queue.back());
    queue.pop_back(); // deque/list的头部删除接口
    return back_data;
  }

  // 查找队列中第一个元素 (oldest)
  const TimestampedData<T> &front() const {
    if (queue.empty()) {
      throw std::runtime_error("Queue is empty");
    }
    return queue.front();
  }

  // 查找尾端元素 (newest)
  const TimestampedData<T> &back() const {
    if (queue.empty()) {
      throw std::runtime_error("Queue is empty");
    }
    return queue.back();
  }

  std::optional<TimestampedData<T>>
  findClosestTime(int target_time // 目标时间（int类型）
  ) {
    // 边界1：vector为空，返回空
    if (queue.empty()) {
      std::cerr << "错误：vector为空，无元素可查找！" << std::endl;
      return std::nullopt;
    }

    // 边界2：vector只有1个元素，直接返回
    if (queue.size() == 1) {
      return queue.front();
    }

    // 3. 遍历查找最接近的元素
    int min_diff = abs(queue[0].timestamp - target_time); // 初始化最小差值
    size_t closest_idx = 0; // 初始化最接近元素的索引

    for (size_t i = 1; i < queue.size(); ++i) {
      // 计算当前元素与目标的绝对差值
      int current_diff = abs(queue[i].timestamp - target_time);

      // 找到更小的差值：更新最小差值和索引
      if (current_diff < min_diff) {
        min_diff = current_diff;
        closest_idx = i;
      }
    }

    // 返回最接近的元素
    return queue[closest_idx];
  }

  // Check if queue is empty
  bool empty() const { return queue.empty(); }

  // Get size of queue
  size_t size() const { return queue.size(); }

  // Clear queue
  void clear() { queue.clear(); }

  // Get all elements (for debugging/inspection)
  std::vector<TimestampedData<T>> getAll() const {
    return std::vector<TimestampedData<T>>(queue.begin(), queue.end());
  }
};

} // namespace amr_reflector_noise_handling

#endif //! ECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP