#pragma once

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

namespace fairspace {
namespace common {

class MeanFilter 
{
 public:
  explicit MeanFilter(const std::uint_fast8_t window_size);
  
  MeanFilter() = default;
 
  ~MeanFilter() = default;

  double update(const double measurement);

 private:
  void remove_earliest();

  void insert(const double value);

  double get_min() const;

  double get_max() const;

  bool should_pop_oldest_candidate(const std::uint_fast8_t old_time) const;

  std::uint_fast8_t window_size_ = 0;

  double sum_ = 0.0;

  std::uint_fast8_t time_ = 0;

  // front = earliest
  std::deque<double> values_;

  // front = min
  std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_;

  // front = max
  std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_;

  bool initialized_ = false;
};

}  // namespace common
}  // namespace fairspace