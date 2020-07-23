# DefineCXX11CompilerFlag
# -----------------------
#
# Tries to find the compiler flag which can be used to enable C++11 on the current compiler.
# If the flag is not found, the macro will issue an cmake error.
#
# DEFINE_CXX_11_COMPILER_FLAG(<var>)
#
# ::
#
#   <var>  - variable to store the resulting flag
#
# Use this to find the compiler option to enable C++11 compilation. This is usefull
# on cmake versions / use cases which do not support CXX_STANDARD.
# Use this in conjunction with target_compiler_option():
#
# include(DefineCXX11CompilerFlag)
# DEFINE_CXX_11_COMPILER_FLAG(CXX11_FLAG)
# ...
# target_compile_options(topt PUBLIC ${CXX11_FLAG})
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

macro (DEFINE_CXX_11_COMPILER_FLAG _RESULT)
  if(NOT DEFINED "${_RESULT}")

    if(NOT CMAKE_REQUIRED_QUIET)
      message(STATUS "Performing C++11 Test")
    endif()

    # Check for default argument (all newer compilers)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
      set(${_RESULT} "-std=c++11" CACHE INTERNAL "C++11 flag")
    else()
      # Check for older version (before 2011)
      CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
      if(COMPILER_SUPPORTS_CXX0X)
	set(${_RESULT} "-std=c++0x" CACHE INTERNAL "C++11 flag")
      else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
      endif()
    endif()

  endif()
endmacro()
