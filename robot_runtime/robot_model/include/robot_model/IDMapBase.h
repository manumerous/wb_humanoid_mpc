#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <concepts>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace robot::model {

// Creating a custom map container that maps ids to elements. It allocates a
// continuous space in memory with one element for each value between min_id and
// max_id even if the original iterator did not contain all of them. This way
// the map can be initialized with a non continuous space of iterator values
// while only resulting in a tiny memory overhead due to the use of
// std::optional.

// Define a concept for the extractor
template <typename E, typename T, typename ScalarType>
concept IDMapExtractor = requires(E e, const T& val) {
  { e(val) } -> std::same_as<ScalarType>;
};

template <typename T>
class IDMapBase {
 public:
  std::optional<T>& at(size_t element_id) { return map_elements_.at(element_id); }

  const std::optional<T>& at(size_t element_id) const { return map_elements_.at(element_id); }

  std::optional<T>& operator[](size_t element_id) { return at(element_id); }

  const std::optional<T>& operator[](size_t element_id) const { return at(element_id); }

  template <typename It, typename ScalarType>
  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> toEigenVector(It begin,
                                                             It end,
                                                             IDMapExtractor<T, ScalarType> auto extractor,
                                                             ScalarType defaultValue) const {
    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> vector(std::distance(begin, end));
    int index = 0;
    for (const auto& id : std::ranges::subrange(begin, end)) {
      const std::optional<T>& val = this->at(id);
      if (val.has_value()) {
        vector(index) = extractor(*val);
      } else {
        vector(index) = defaultValue;
      }
      index++;
    }
    return vector;
  }

  bool inRange(size_t id) const noexcept {
    if (id > map_elements_.size()) {
      return false;
    } else {
      return true;
    }
  }

  size_t size() const {
    return std::count_if(begin(), end(), [](const std::optional<T>& opt) { return opt.has_value(); });
  }

  size_t capacity() const { return map_elements_.size(); }

  // iterator class to skip uninitialized elements

  class const_iterator;
  class iterator;

  // Base iterator template with common functionality
  template <typename MapType, typename RefType>
  class iterator_base {
   public:
    iterator_base(MapType map, size_t it) : map_(map), it_(it) {
      // Skip nullopt elements
      skipNullopt();
    }

    // Increase the iterator
    iterator_base& operator++() {
      ++it_;
      skipNullopt();
      return *this;
    }

    bool operator==(const iterator_base& other) const { return it_ == other.it_; }

    bool operator!=(const iterator_base& other) const { return it_ != other.it_; }

   protected:
    MapType map_;
    size_t it_;

    // Helper function to skip over elements that are not null optionals
    void skipNullopt() {
      while (it_ != map_->map_elements_.size() && !map_->map_elements_[it_].has_value()) {
        ++it_;  // skip while current element is std::nullopt
      }
    }

    // Allow the derived classes to access each other's private members
    friend class iterator;
    friend class const_iterator;
  };

  class iterator : public iterator_base<IDMapBase<T>*, T&> {
   public:
    iterator(IDMapBase<T>* map, size_t it) : iterator_base<IDMapBase<T>*, T&>(map, it) {}

    // Dereference to get underlying data
    T& operator*() { return this->map_->map_elements_[this->it_].value(); }

    // Conversion to const iterator
    operator const_iterator() const { return const_iterator(this->map_, this->it_); }
  };

  class const_iterator : public iterator_base<const IDMapBase<T>*, const T&> {
   public:
    const_iterator(const IDMapBase<T>* map, size_t it) : iterator_base<const IDMapBase<T>*, const T&>(map, it) {}

    // Constructor from non-const iterator
    const_iterator(const iterator& other) : iterator_base<const IDMapBase<T>*, const T&>(other.map_, other.it_) {}

    // Dereference to get underlying data
    const T& operator*() const { return this->map_->map_elements_[this->it_].value(); }
  };

  iterator begin() { return iterator(this, 0); }
  iterator end() { return iterator(this, map_elements_.size()); }
  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const { return const_iterator(this, map_elements_.size()); }
  const_iterator cbegin() const { return const_iterator(this, 0); }
  const_iterator cend() const { return const_iterator(this, map_elements_.size()); }

 protected:
  explicit IDMapBase(size_t size) : map_elements_(size) {
    // Limit map size
    if (size > 255) {
      throw std::runtime_error("Too many elements in ID map. max supported size is 255");
    }
    if (size < 2) {
      throw std::runtime_error("Initialized map with only one id!");
    }
  }

  IDMapBase() = delete;

  IDMapBase(const IDMapBase<T>& other) : map_elements_(other.map_elements_){};

  IDMapBase& operator=(const IDMapBase<T>& other) {
    for (size_t i = 0; i < map_elements_.size(); ++i) {
      // Enable bounds checking to prevent other not containing all
      // elements
      map_elements_[i] = other.map_elements_.at(i);
    }
    return *this;
  }

 private:
  std::vector<std::optional<T>> map_elements_;
};

}  // namespace robot::model
