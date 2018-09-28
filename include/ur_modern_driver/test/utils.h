#pragma once

#define ASSERT_DOUBLE_ARRAY_EQ(fn, name)                                                                               \
  for (auto const& v : name)                                                                                           \
  {                                                                                                                    \
    ASSERT_EQ(fn, v) << #name " failed parsing";                                                                       \
  }
