# DefineCXX14CompilerFlag
# -----------------------
#
# Tries to find the compiler flag which can be used to enable C++14 on the current compiler.
# If the flag is not found, the macro will issue an cmake error.
#
# DEFINE_CXX_14_COMPILER_FLAG(<var>)
#
# ::
#
#   <var>  - variable to store the resulting flag
#
# Use this to find the compiler option to enable C++14 compilation. This is usefull
# on cmake versions / use cases which do not support CXX_STANDARD.
# Use this in conjunction with target_compiler_option():
#
# include(DefineCXX14CompilerFlag)
# DEFINE_CXX_14_COMPILER_FLAG(CXX14_FLAG)
# ...
# target_compile_options(topt PUBLIC ${CXX14_FLAG})
# ...
#
# The following variables may be set before calling this macro to modify
# the way the check is run:
#
# ::
#
#   CMAKE_REQUIRED_QUIET = execute quietly without messages
#

include(CheckCXXCompilerFlag)

macro (DEFINE_CXX_14_COMPILER_FLAG _RESULT)
  if(NOT DEFINED "${_RESULT}")

    if(NOT CMAKE_REQUIRED_QUIET)
      message(STATUS "Performing C++14 Test")
    endif()

    # Check for default argument (all newer compilers)
    CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
    if(COMPILER_SUPPORTS_CXX14)
      set(${_RESULT} "-std=c++14" CACHE INTERNAL "C++14 flag")
    else()
      # Check for older version (like gcc-4.8.4)
      CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
      if(COMPILER_SUPPORTS_CXX1Y)
	set(${_RESULT} "-std=c++1y" CACHE INTERNAL "C++14 flag")
      else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
      endif()
    endif()

  endif()
endmacro()
