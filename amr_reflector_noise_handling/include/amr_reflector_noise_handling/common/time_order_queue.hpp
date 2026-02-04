#ifndef AMR_REFLECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP
#define AMR_REFLECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP
#include <algorithm>
#include <deque>
#include <vector>

namespace amr_reflector_noise_handling {

   template<typename T>
   class TimestampedData {
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

   template <typename T>
   class TimeOrderQueue {
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
       if(start_it == queue.end()) return result;
       if(start_it == queue.begin()) return result;
       // Find first element > end_time
       auto end_it = std::upper_bound(queue.begin(), queue.end(),
                                      TimestampedData<T>(end_time));
       if (end_it == queue.end()) return result;

       // Copy elements in range [start_it - 1, end_it)
       // start_it - 1 是因为 lower_bound 会返回第一个 >= start_time 的元素，而我们想要的是第一个 <= start_time 的元素
       result.assign(start_it -1 , end_it);

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
       for (auto it = start_it - 1; it != end_it; ++it) {
         result.push(*it);
       }
       return result;
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

} // ns amr_reflector_noise_handling

#endif //!ECTOR_NOISE_HANDLING_COMMON_TIME_ORDER_QUEUE_HPP