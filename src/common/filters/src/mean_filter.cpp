#include "mean_filter.h"

#include <limits>

#include <ros/console.h>
// #include "cyber/common/log.h"

namespace fairspace {
namespace common {

using uint8 = std::uint_fast8_t;
using TimedValue = std::pair<uint8, double>;

const uint8 kMaxWindowSize = std::numeric_limits<uint8>::max() / 2;

MeanFilter::MeanFilter(const uint8 window_size) : window_size_(window_size) 
{
  //TODO: check the use of gtest
  CHECK_GT(window_size_, 0);
  CHECK_LE(window_size_, kMaxWindowSize);
  initialized_ = true;
}

double MeanFilter::get_min() 
const {
  if (min_candidates_.empty()) 
  {
    return std::numeric_limits<double>::infinity();
  } 
  else 
  {
    return min_candidates_.front().second;
  }
}

double MeanFilter::get_max() 
const {
  if (max_candidates_.empty()) 
  {
    return -std::numeric_limits<double>::infinity();
  } 
  else 
  {
    return max_candidates_.front().second;
  }
}

double MeanFilter::update(const double measurement) 
{
  ACHECK(initialized_);
  CHECK_LE(values_.size(), window_size_);
  CHECK_LE(min_candidates_.size(), window_size_);
  CHECK_LE(max_candidates_.size(), window_size_);
  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) 
  {
    remove_earliest();
  }
  insert(measurement);
  if (values_.size() > 2) 
  {
    return (sum_ - get_min() - get_max()) /
           static_cast<double>(values_.size() - 2);
  } 
  else 
  {
    return sum_ / static_cast<double>(values_.size());
  }
}

bool MeanFilter::should_pop_oldest_candidate(const uint8 old_time) 
const {
  if (old_time < window_size_) 
  {
    CHECK_LE(time_, old_time + window_size_);
    return old_time + window_size_ == time_;
  } 
  else if (time_ < window_size_) 
  {
    CHECK_GE(old_time, time_ + window_size_);
    return old_time == time_ + window_size_;
  } 
  else 
  {
    return false;
  }
}

void MeanFilter::remove_earliest() 
{
  CHECK_EQ(values_.size(), window_size_);
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (should_pop_oldest_candidate(min_candidates_.front().first)) 
  {
    min_candidates_.pop_front();
  }

  if (should_pop_oldest_candidate(max_candidates_.front().first)) 
  {
    max_candidates_.pop_front();
  }
}

void MeanFilter::insert(const double value) 
{
  values_.push_back(value);
  sum_ += value;

  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) 
  {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));

  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) 
  {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

}  // namespace common
}  // namespace fairspace