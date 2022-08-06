// Copyright (2022) Gerardo Puga
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

// gtest
#include "gtest/gtest.h"

// project
#include <elm_trackers/datatypes/Buffer2D.hpp>

namespace elm_trackers {

namespace test {

namespace {

using ::testing::Test;

struct Buffer2DTests : public Test {};

TEST_F(Buffer2DTests, ConstructionTest) {
  Buffer2D uut(3, 4);

  ASSERT_EQ(3u, uut.width());
  ASSERT_EQ(4u, uut.height());

  ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 0));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(3, 2));
}

TEST_F(Buffer2DTests, CellAssignments) {
  Buffer2D uut(3, 3);

  uut.cell(0, 0) = 1.0;
  uut.cell(2, 0) = 3.0;
  uut.cell(2, 2) = 5.0;

  ASSERT_DOUBLE_EQ(1.0, uut.cell(0, 0));
  ASSERT_DOUBLE_EQ(3.0, uut.cell(2, 0));
  ASSERT_DOUBLE_EQ(5.0, uut.cell(2, 2));
}

TEST_F(Buffer2DTests, ResetOperation) {
  Buffer2D uut(2, 2);

  uut.cell(0, 0) = 1.0;
  uut.cell(1, 1) = 1.0;

  ASSERT_DOUBLE_EQ(1.0, uut.cell(0, 0));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
  ASSERT_DOUBLE_EQ(1.0, uut.cell(1, 1));

  uut.reset();

  ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 0));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
  ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 1));
}

TEST_F(Buffer2DTests, CopyConstructor) {
  Buffer2D uut(3, 3);

  uut.cell(0, 0) = 1.0;
  uut.cell(2, 0) = 3.0;
  uut.cell(2, 2) = 5.0;

  const Buffer2D uut_copy(uut);

  ASSERT_DOUBLE_EQ(1.0, uut_copy.cell(0, 0));
  ASSERT_DOUBLE_EQ(3.0, uut_copy.cell(2, 0));
  ASSERT_DOUBLE_EQ(5.0, uut_copy.cell(2, 2));
}

TEST_F(Buffer2DTests, MoveConstructor) {
  Buffer2D uut(3, 3);

  uut.cell(0, 0) = 1.0;
  uut.cell(2, 0) = 3.0;
  uut.cell(2, 2) = 5.0;

  const Buffer2D uut_copy(std::move(uut));

  ASSERT_DOUBLE_EQ(1.0, uut_copy.cell(0, 0));
  ASSERT_DOUBLE_EQ(3.0, uut_copy.cell(2, 0));
  ASSERT_DOUBLE_EQ(5.0, uut_copy.cell(2, 2));
}

TEST_F(Buffer2DTests, ScalarOperations) {
  Buffer2D base(2, 2);
  base.cell(0, 0) = 1.0;
  base.cell(1, 1) = 1.0;

  {
    auto uut = base;
    uut += 1.0;
    ASSERT_DOUBLE_EQ(2.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(1.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(1.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(2.0, uut.cell(1, 1));
  }

  {
    auto uut = base;
    uut -= 1.0;
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(-1.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(-1.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 1));
  }

  {
    auto uut = base;
    uut *= 3.0;
    ASSERT_DOUBLE_EQ(3.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(3.0, uut.cell(1, 1));
  }

  {
    auto uut = base;
    uut /= 0.1;
    ASSERT_DOUBLE_EQ(10.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(10.0, uut.cell(1, 1));
  }
}

TEST_F(Buffer2DTests, ArrayOperations) {
  Buffer2D base(2, 2);
  base.cell(0, 0) = 1.0;
  base.cell(1, 1) = 1.0;

  Buffer2D rhs(2, 2);
  rhs.cell(0, 0) = 2.0;
  rhs.cell(1, 1) = 3.0;

  const auto crhs = rhs;

  {
    auto uut = base;
    uut += crhs;
    ASSERT_DOUBLE_EQ(3.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(4.0, uut.cell(1, 1));
  }

  {
    auto uut = base;
    uut -= crhs;
    ASSERT_DOUBLE_EQ(-1.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(-2.0, uut.cell(1, 1));
  }

  {
    auto uut = base;
    uut *= crhs;
    ASSERT_DOUBLE_EQ(2.0, uut.cell(0, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(1, 0));
    ASSERT_DOUBLE_EQ(0.0, uut.cell(0, 1));
    ASSERT_DOUBLE_EQ(3.0, uut.cell(1, 1));
  }
}

TEST_F(Buffer2DTests, RawBufferAccess) {
  Buffer2D uut(2, 2);

  auto &internal_buffer = uut.buffer();

  internal_buffer(0, 0) = 1.0;
  ASSERT_DOUBLE_EQ(1.0, uut.cell(0, 0));

  uut.cell(1, 1) = 2.0;
  ASSERT_DOUBLE_EQ(2.0, internal_buffer(1, 1));

  const auto const_uut = uut;
  const auto &const_internal_buffer = const_uut.buffer();
  ASSERT_DOUBLE_EQ(2.0, const_internal_buffer(1, 1));
}

TEST_F(Buffer2DTests, OutOfRangeBuffer) {
  Buffer2D uut(3, 2);

  ASSERT_THROW(uut.cell(2, 0), std::out_of_range);
  ASSERT_THROW(uut.cell(0, 3), std::out_of_range);

  ASSERT_NO_THROW(uut.cell(0, 0));
  ASSERT_NO_THROW(uut.cell(0, 0));
  ASSERT_NO_THROW(uut.cell(0, 2));
  ASSERT_NO_THROW(uut.cell(1, 2));
}

TEST_F(Buffer2DTests, ArrayOperations2) {
  Buffer2D uut(2, 3);
  uut.cell(0, 0) = 1.0;
  uut.cell(0, 1) = 2.0;
  uut.cell(1, 0) = 3.0;
  uut.cell(1, 1) = 4.0;
  uut.cell(2, 0) = 5.0;
  uut.cell(2, 1) = 6.0;

  uut *= uut;

  ASSERT_DOUBLE_EQ(1.0, uut.cell(0, 0));
  ASSERT_DOUBLE_EQ(4.0, uut.cell(0, 1));
  ASSERT_DOUBLE_EQ(9.0, uut.cell(1, 0));
  ASSERT_DOUBLE_EQ(16.0, uut.cell(1, 1));
  ASSERT_DOUBLE_EQ(25.0, uut.cell(2, 0));
  ASSERT_DOUBLE_EQ(36.0, uut.cell(2, 1));
}

}  // namespace

}  // namespace test

}  // namespace elm_trackers
