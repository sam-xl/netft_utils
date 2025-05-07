#ifndef LP_FILTER_H
#define LP_FILTER_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

/**
 * @brief Implements a multi-element low-pass filter.
 *
 * The LPFilter class applies a configurable low-pass filter to input vectors, supporting multiple elements in parallel. It maintains internal state for each element and allows repeated filtering of sequential data.
 *
 * @param deltaT Sampling interval in seconds.
 * @param cutoffFrequency Cutoff frequency of the filter in Hz.
 * @param numElements Number of elements to filter in each input vector.
 */
class LPFilter
{
public:
  LPFilter(double deltaT, double cutoffFrequency, int numElements);
  bool update(std::vector<double> input, std::vector<double>& output);
  
private:
  rclcpp::Logger logger = rclcpp::get_logger("LPFilter");
  bool initialized;
  int noElements;
  std::vector<double> in1, in2, out1, out2;
  double omega_a;
  double a0, a1, a2, b1, b2;
};

#endif