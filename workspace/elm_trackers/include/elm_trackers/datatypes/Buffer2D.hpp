/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

#pragma once

// standard library
#include <cstdint>

// third-party
#include <eigen3/Eigen/Dense>

namespace elm_trackers {

class Buffer2D {
 public:
  Buffer2D(const std::size_t width, const std::size_t height);

  void reset();

  double &cell(const std::size_t row, const std::size_t col);
  const double &cell(const std::size_t row, const std::size_t col) const;

  Eigen::MatrixXd &buffer() { return buffer_; }
  const Eigen::MatrixXd &buffer() const { return buffer_; }

  std::size_t width() const { return width_; }

  std::size_t height() const { return height_; }

  Buffer2D &operator+=(const double rhs);
  Buffer2D &operator-=(const double rhs);
  Buffer2D &operator*=(const double rhs);
  Buffer2D &operator/=(const double rhs);

  Buffer2D(const Buffer2D &) = default;
  Buffer2D(Buffer2D &&) = default;
  Buffer2D &operator=(const Buffer2D &) = default;
  Buffer2D &operator=(Buffer2D &&) = default;

 private:
  std::size_t width_;
  std::size_t height_;

  Eigen::MatrixXd buffer_;

  bool outOfRange(const std::size_t row, const std::size_t col) const;
};

}  // namespace elm_trackers
