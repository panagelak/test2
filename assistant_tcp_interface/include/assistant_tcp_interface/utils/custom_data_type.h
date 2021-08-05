/**
 * @file custom_data_type.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 30-04-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace assistant_tcp_interface {


//class RobotCommand {
//public:
//  inline static const char INITIALIZE = 'I';
//  inline static const char SENSOR_CONFIGURATION = 'S';
//  inline static const char IO = 'O';
//  inline static const char RESET = 'R';
//  inline static const char PAUSE = 'P';
//  inline static const char TERMINATE = 'T';
//  inline static const char CANCEL = 'X';
//  inline static const char ARM_PAUSE = 'K';
//  inline static const char ARM_PAUSE_WITH_CANCEL = 'L';
//  inline static const char ARM_RESUME = 'M';
//};


namespace utils {

const bool False = false;
const bool True = false;

using vectorstr_t = std::vector<std::string>;
using vector6f_t = std::array<float, 6>;

using vector6i_t = std::array<uint8_t, 6>;
using vectorImageData = std::array<uint8_t, 1200>;

/**
 * @brief Operator << overload for arrays
 *
 * @tparam T
 * @tparam N
 * @param out
 * @param item
 * @return std::ostream&
 */
template <class T, std::size_t N> std::ostream &operator<<(std::ostream &out, const std::array<T, N> &item) {
  out << "[";
  for (size_t i = 0; i < item.size(); ++i) {
    out << item[i];
    if (i != item.size() - 1) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

/**
 * @brief Operator << overload for vectors
 *
 * @tparam T
 * @param os
 * @param v
 * @return std::ostream&
 */
template <typename T> std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "{";
  for (size_t i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i != v.size() - 1)
      os << ", ";
  }
  os << "}";
  return os;
}

/**
 * @brief Degree to rad conversion function for arrays
 *
 * @tparam T
 * @tparam N
 * @param degree_vec
 * @return std::array<T, N>
 */
template <class T, std::size_t N> std::array<T, N> toRad(const std::array<T, N> &degree_vec) {
  std::array<T, N> rad_vec;
  for (size_t i = 0; i < degree_vec.size(); ++i) {
    rad_vec.at(i) = degree_vec.at(i) * M_PI / 180;
  }
  return rad_vec;
}

template <class T> std::vector<T> toRad(const std::vector<T> &degree_vec) {
  std::vector<T> rad_vec;
  for (size_t i = 0; i < degree_vec.size(); ++i) {
    rad_vec.at(i) = degree_vec.at(i) * M_PI / 180;
  }
  return rad_vec;
}


/**
 * @brief  Rad to degree conversion for arrays
 *
 * @tparam T
 * @tparam N
 * @param rad_vec
 * @return std::array<T, N>
 */
template <class T, std::size_t N> std::array<T, N> toDegree(const std::array<T, N> &rad_vec) {
  std::array<T, N> degree_vec;
  for (size_t i = 0; i < rad_vec.size(); ++i) {
    degree_vec.at(i) = rad_vec.at(i) * 180 / M_PI;
  }
  return degree_vec;
}

template <class T> std::vector<T> toDegree(const std::vector<T> &rad_vec) {
  std::vector<T> degree_vec;
  for (size_t i = 0; i < rad_vec.size(); ++i) {
    degree_vec.at(i) = rad_vec.at(i) * 180 / M_PI;
  }
  return degree_vec;
}



} // namespace utils
} // namespace assistant_tcp_interface
