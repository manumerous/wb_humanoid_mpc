#include <mutex>

namespace robot {

template <typename T>
class ThreadSafe {
 public:
  // Constructor
  explicit ThreadSafe(const T& initial = T{}) : value_(initial) {}

  ThreadSafe(T&& initial) : value_(std::move(initial)) {}

  template <typename Func>
  auto call_with_lock(Func&& func) {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::forward<Func>(func)(value_);
  }

  // Const version for read-only access
  template <typename Func>
  auto call_with_lock(Func&& func) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::forward<Func>(func)(value_);
  }

  void copy_value(T& value) const {
    std::lock_guard<std::mutex> lock(mutex_);
    value = value_;
  }

  void set(const T& new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = new_value;
  }

  void set(T&& new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = new_value;
  }

 private:
  mutable std::mutex mutex_;
  T value_;
};

}  // namespace robot