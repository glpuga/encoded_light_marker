/* Copyright [2022] <Gerardo Puga>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT) */

// standard library
#include <math.h>

#include <stdexcept>

// third-party

// project
#include <elm_trackers/datatypes/Buffer2D.hpp>

namespace elm_trackers {

Buffer2D::Buffer2D(const std::size_t width, const std::size_t height)
    : width_{width}, height_{height}, buffer_{height_, width_} {
  buffer_ *= 0.0;
}

void Buffer2D::reset() { buffer_ *= 0.0; }

double &Buffer2D::cell(const std::size_t row, const std::size_t col) {
  if (outOfRange(row, col)) {
    throw std::out_of_range("Out of range access to buffer");
  }
  return buffer_(row, col);
}

const double &Buffer2D::cell(const std::size_t row,
                             const std::size_t col) const {
  if (outOfRange(row, col)) {
    throw std::out_of_range("Out of range access to buffer");
  }
  return buffer_(row, col);
}

Buffer2D &Buffer2D::operator+=(const double rhs) {
  buffer_.array() += rhs;
  return *this;
}

Buffer2D &Buffer2D::operator-=(const double rhs) {
  buffer_.array() -= rhs;
  return *this;
}

Buffer2D &Buffer2D::operator*=(const double rhs) {
  buffer_ *= rhs;
  return *this;
}

Buffer2D &Buffer2D::operator/=(const double rhs) {
  buffer_ /= rhs;
  return *this;
}

Buffer2D &Buffer2D::operator+=(const Buffer2D &rhs) {
  buffer_ += rhs.buffer();
  return *this;
}

Buffer2D &Buffer2D::operator-=(const Buffer2D &rhs) {
  buffer_ -= rhs.buffer();
  return *this;
}

Buffer2D &Buffer2D::operator*=(const Buffer2D &rhs) {
  buffer_.array() *= rhs.buffer().array();
  return *this;
}

bool Buffer2D::outOfRange(const std::size_t row, const std::size_t col) const {
  if ((row >= height_) || (col >= width_)) {
    return true;
  }
  return false;
}

} // namespace elm_trackers
