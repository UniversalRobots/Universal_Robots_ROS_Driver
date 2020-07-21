// ©2013-2016 Cameron Desrochers.
// Distributed under the simplified BSD license (see the license file that
// should have come with this header).
// Uses Jeff Preshing's semaphore implementation (under the terms of its
// separate zlib license, embedded below).

#pragma once

// Provides portable (VC++2010+, Intel ICC 13, GCC 4.7+, and anything C++11 compliant) implementation
// of low-level memory barriers, plus a few semi-portable utility macros (for inlining and alignment).
// Also has a basic atomic type (limited to hardware-supported atomics with no memory ordering guarantees).
// Uses the AE_* prefix for macros (historical reasons), and the "moodycamel" namespace for symbols.

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <type_traits>

// Platform detection
#if defined(__INTEL_COMPILER)
#define AE_ICC
#elif defined(_MSC_VER)
#define AE_VCPP
#elif defined(__GNUC__)
#define AE_GCC
#endif

#if defined(_M_IA64) || defined(__ia64__)
#define AE_ARCH_IA64
#elif defined(_WIN64) || defined(__amd64__) || defined(_M_X64) || defined(__x86_64__)
#define AE_ARCH_X64
#elif defined(_M_IX86) || defined(__i386__)
#define AE_ARCH_X86
#elif defined(_M_PPC) || defined(__powerpc__)
#define AE_ARCH_PPC
#else
#define AE_ARCH_UNKNOWN
#endif

// AE_UNUSED
#define AE_UNUSED(x) ((void)x)

// AE_FORCEINLINE
#if defined(AE_VCPP) || defined(AE_ICC)
#define AE_FORCEINLINE __forceinline
#elif defined(AE_GCC)
//#define AE_FORCEINLINE __attribute__((always_inline))
#define AE_FORCEINLINE inline
#else
#define AE_FORCEINLINE inline
#endif

// AE_ALIGN
#if defined(AE_VCPP) || defined(AE_ICC)
#define AE_ALIGN(x) __declspec(align(x))
#elif defined(AE_GCC)
#define AE_ALIGN(x) __attribute__((aligned(x)))
#else
// Assume GCC compliant syntax...
#define AE_ALIGN(x) __attribute__((aligned(x)))
#endif

// Portable atomic fences implemented below:

namespace moodycamel
{
enum memory_order
{
  memory_order_relaxed,
  memory_order_acquire,
  memory_order_release,
  memory_order_acq_rel,
  memory_order_seq_cst,

  // memory_order_sync: Forces a full sync:
  // #LoadLoad, #LoadStore, #StoreStore, and most significantly, #StoreLoad
  memory_order_sync = memory_order_seq_cst
};

}  // end namespace moodycamel

#if (defined(AE_VCPP) && (_MSC_VER < 1700 || defined(__cplusplus_cli))) || defined(AE_ICC)
// VS2010 and ICC13 don't support std::atomic_*_fence, implement our own fences

#include <intrin.h>

#if defined(AE_ARCH_X64) || defined(AE_ARCH_X86)
#define AeFullSync _mm_mfence
#define AeLiteSync _mm_mfence
#elif defined(AE_ARCH_IA64)
#define AeFullSync __mf
#define AeLiteSync __mf
#elif defined(AE_ARCH_PPC)
#include <ppcintrinsics.h>
#define AeFullSync __sync
#define AeLiteSync __lwsync
#endif

#ifdef AE_VCPP
#pragma warning(push)
#pragma warning(disable : 4365)  // Disable erroneous 'conversion from long to unsigned int, signed/unsigned mismatch'
                                 // error when using `assert`
#ifdef __cplusplus_cli
#pragma managed(push, off)
#endif
#endif

namespace moodycamel
{
AE_FORCEINLINE void compilerFence(memory_order order)
{
  switch (order)
  {
    case memory_order_relaxed:
      break;
    case memory_order_acquire:
      _ReadBarrier();
      break;
    case memory_order_release:
      _WriteBarrier();
      break;
    case memory_order_acq_rel:
      _ReadWriteBarrier();
      break;
    case memory_order_seq_cst:
      _ReadWriteBarrier();
      break;
    default:
      assert(false);
  }
}

// x86/x64 have a strong memory model -- all loads and stores have
// acquire and release semantics automatically (so only need compiler
// barriers for those).
#if defined(AE_ARCH_X86) || defined(AE_ARCH_X64)
AE_FORCEINLINE void fence(memory_order order)
{
  switch (order)
  {
    case memory_order_relaxed:
      break;
    case memory_order_acquire:
      _ReadBarrier();
      break;
    case memory_order_release:
      _WriteBarrier();
      break;
    case memory_order_acq_rel:
      _ReadWriteBarrier();
      break;
    case memory_order_seq_cst:
      _ReadWriteBarrier();
      AeFullSync();
      _ReadWriteBarrier();
      break;
    default:
      assert(false);
  }
}
#else
AE_FORCEINLINE void fence(memory_order order)
{
  // Non-specialized arch, use heavier memory barriers everywhere just in case :-(
  switch (order)
  {
    case memory_order_relaxed:
      break;
    case memory_order_acquire:
      _ReadBarrier();
      AeLiteSync();
      _ReadBarrier();
      break;
    case memory_order_release:
      _WriteBarrier();
      AeLiteSync();
      _WriteBarrier();
      break;
    case memory_order_acq_rel:
      _ReadWriteBarrier();
      AeLiteSync();
      _ReadWriteBarrier();
      break;
    case memory_order_seq_cst:
      _ReadWriteBarrier();
      AeFullSync();
      _ReadWriteBarrier();
      break;
    default:
      assert(false);
  }
}
#endif
}  // end namespace moodycamel
#else
// Use standard library of atomics
#include <atomic>

namespace moodycamel
{
AE_FORCEINLINE void compilerFence(memory_order order)
{
  switch (order)
  {
    case memory_order_relaxed:
      break;
    case memory_order_acquire:
      std::atomic_signal_fence(std::memory_order_acquire);
      break;
    case memory_order_release:
      std::atomic_signal_fence(std::memory_order_release);
      break;
    case memory_order_acq_rel:
      std::atomic_signal_fence(std::memory_order_acq_rel);
      break;
    case memory_order_seq_cst:
      std::atomic_signal_fence(std::memory_order_seq_cst);
      break;
    default:
      assert(false);
  }
}

AE_FORCEINLINE void fence(memory_order order)
{
  switch (order)
  {
    case memory_order_relaxed:
      break;
    case memory_order_acquire:
      std::atomic_thread_fence(std::memory_order_acquire);
      break;
    case memory_order_release:
      std::atomic_thread_fence(std::memory_order_release);
      break;
    case memory_order_acq_rel:
      std::atomic_thread_fence(std::memory_order_acq_rel);
      break;
    case memory_order_seq_cst:
      std::atomic_thread_fence(std::memory_order_seq_cst);
      break;
    default:
      assert(false);
  }
}

}  // end namespace moodycamel

#endif

#if !defined(AE_VCPP) || (_MSC_VER >= 1700 && !defined(__cplusplus_cli))
#define AE_USE_STD_ATOMIC_FOR_WEAK_ATOMIC
#endif

#ifdef AE_USE_STD_ATOMIC_FOR_WEAK_ATOMIC
#include <atomic>
#endif
#include <utility>

// WARNING: *NOT* A REPLACEMENT FOR std::atomic. READ CAREFULLY:
// Provides basic support for atomic variables -- no memory ordering guarantees are provided.
// The guarantee of atomicity is only made for types that already have atomic load and store guarantees
// at the hardware level -- on most platforms this generally means aligned pointers and integers (only).
namespace moodycamel
{
template <typename T>
class WeakAtomic
{
public:
  WeakAtomic()
  {
  }
#ifdef AE_VCPP
#pragma warning(disable : 4100)  // Get rid of (erroneous) 'unreferenced formal parameter' warning
#endif
  template <typename U>
  WeakAtomic(U&& x) : value_(std::forward<U>(x))
  {
  }
#ifdef __cplusplus_cli
  // Work around bug with universal reference/nullptr combination that only appears when /clr is on
  WeakAtomic(nullptr_t) : value_(nullptr)
  {
  }
#endif
  WeakAtomic(WeakAtomic const& other) : value_(other.value_)
  {
  }
  WeakAtomic(WeakAtomic&& other) : value_(std::move(other.value_))
  {
  }
#ifdef AE_VCPP
#pragma warning(default : 4100)
#endif

  AE_FORCEINLINE operator T() const
  {
    return load();
  }

#ifndef AE_USE_STD_ATOMIC_FOR_WEAK_ATOMIC
  template <typename U>
  AE_FORCEINLINE WeakAtomic const& operator=(U&& x)
  {
    value_ = std::forward<U>(x);
    return *this;
  }
  AE_FORCEINLINE WeakAtomic const& operator=(WeakAtomic const& other)
  {
    value_ = other.value_;
    return *this;
  }

  AE_FORCEINLINE T load() const
  {
    return value_;
  }

  AE_FORCEINLINE T fetchAddAcquire(T increment)
  {
#if defined(AE_ARCH_X64) || defined(AE_ARCH_X86)
    if (sizeof(T) == 4)
      return _InterlockedExchangeAdd((long volatile*)&value_, (long)increment);
#if defined(_M_AMD64)
    else if (sizeof(T) == 8)
      return _InterlockedExchangeAdd64((long long volatile*)&value_, (long long)increment);
#endif
#else
#error Unsupported platform
#endif
    assert(false && "T must be either a 32 or 64 bit type");
    return value_;
  }

  AE_FORCEINLINE T fetchAddRelease(T increment)
  {
#if defined(AE_ARCH_X64) || defined(AE_ARCH_X86)
    if (sizeof(T) == 4)
      return _InterlockedExchangeAdd((long volatile*)&value_, (long)increment);
#if defined(_M_AMD64)
    else if (sizeof(T) == 8)
      return _InterlockedExchangeAdd64((long long volatile*)&value_, (long long)increment);
#endif
#else
#error Unsupported platform
#endif
    assert(false && "T must be either a 32 or 64 bit type");
    return value_;
  }
#else
  template <typename U>
  AE_FORCEINLINE WeakAtomic const& operator=(U&& x)
  {
    value_.store(std::forward<U>(x), std::memory_order_relaxed);
    return *this;
  }

  AE_FORCEINLINE WeakAtomic const& operator=(WeakAtomic const& other)
  {
    value_.store(other.value_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    return *this;
  }

  AE_FORCEINLINE T load() const
  {
    return value_.load(std::memory_order_relaxed);
  }

  AE_FORCEINLINE T fetchAddAcquire(T increment)
  {
    return value_.fetch_add(increment, std::memory_order_acquire);
  }

  AE_FORCEINLINE T fetchAddRelease(T increment)
  {
    return value_.fetch_add(increment, std::memory_order_release);
  }
#endif

private:
#ifndef AE_USE_STD_ATOMIC_FOR_WEAK_ATOMIC
  // No std::atomic support, but still need to circumvent compiler optimizations.
  // `volatile` will make memory access slow, but is guaranteed to be reliable.
  volatile T value_;
#else
  std::atomic<T> value_;
#endif
};

}  // end namespace moodycamel

// Portable single-producer, single-consumer semaphore below:

#if defined(_WIN32)
// Avoid including windows.h in a header; we only need a handful of
// items, so we'll redeclare them here (this is relatively safe since
// the API generally has to remain stable between Windows versions).
// I know this is an ugly hack but it still beats polluting the global
// namespace with thousands of generic names or adding a .cpp for nothing.
extern "C" {
struct _SECURITY_ATTRIBUTES;
__declspec(dllimport) void* __stdcall CreateSemaphoreW(_SECURITY_ATTRIBUTES* lpSemaphoreAttributes, long lInitialCount,
                                                       long lMaximumCount, const wchar_t* lpName);
__declspec(dllimport) int __stdcall CloseHandle(void* hObject);
__declspec(dllimport) unsigned long __stdcall WaitForSingleObject(void* hHandle, unsigned long dwMilliseconds);
__declspec(dllimport) int __stdcall ReleaseSemaphore(void* hSemaphore, long lReleaseCount, long* lpPreviousCount);
}
#elif defined(__MACH__)
#include <mach/mach.h>
#elif defined(__unix__)
#include <semaphore.h>
#endif

namespace moodycamel
{
// Code in the spsc_sema namespace below is an adaptation of Jeff Preshing's
// portable + lightweight semaphore implementations, originally from
// https://github.com/preshing/cpp11-on-multicore/blob/master/common/sema.h
// LICENSE:
// Copyright (c) 2015 Jeff Preshing
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgement in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
namespace spsc_sema
{
#if defined(_WIN32)
class Semaphore
{
private:
  void* m_hSema;

  Semaphore(const Semaphore& other);
  Semaphore& operator=(const Semaphore& other);

public:
  Semaphore(int initialCount = 0)
  {
    assert(initialCount >= 0);
    const long maxLong = 0x7fffffff;
    m_hSema = CreateSemaphoreW(nullptr, initialCount, maxLong, nullptr);
  }

  ~Semaphore()
  {
    CloseHandle(m_hSema);
  }

  void wait()
  {
    const unsigned long infinite = 0xffffffff;
    WaitForSingleObject(m_hSema, infinite);
  }

  bool tryWait()
  {
    const unsigned long RC_WAIT_TIMEOUT = 0x00000102;
    return WaitForSingleObject(m_hSema, 0) != RC_WAIT_TIMEOUT;
  }

  bool timedWait(std::uint64_t usecs)
  {
    const unsigned long RC_WAIT_TIMEOUT = 0x00000102;
    return WaitForSingleObject(m_hSema, (unsigned long)(usecs / 1000)) != RC_WAIT_TIMEOUT;
  }

  void signal(int count = 1)
  {
    ReleaseSemaphore(m_hSema, count, nullptr);
  }
};
#elif defined(__MACH__)
//---------------------------------------------------------
// Semaphore (Apple iOS and OSX)
// Can't use POSIX semaphores due to http://lists.apple.com/archives/darwin-kernel/2009/Apr/msg00010.html
//---------------------------------------------------------
class Semaphore
{
private:
  semaphore_t sema_;

  Semaphore(const Semaphore& other);
  Semaphore& operator=(const Semaphore& other);

public:
  Semaphore(int initialCount = 0)
  {
    assert(initialCount >= 0);
    semaphore_create(mach_task_self(), &sema_, SYNC_POLICY_FIFO, initialCount);
  }

  ~Semaphore()
  {
    semaphore_destroy(mach_task_self(), sema_);
  }

  void wait()
  {
    semaphore_wait(sema_);
  }

  bool tryWait()
  {
    return timedWait(0);
  }

  bool timedWait(std::int64_t timeout_usecs)
  {
    mach_timespec_t ts;
    ts.tv_sec = timeout_usecs / 1000000;
    ts.tv_nsec = (timeout_usecs % 1000000) * 1000;

    // added in OSX 10.10:
    // https://developer.apple.com/library/prerelease/mac/documentation/General/Reference/APIDiffsMacOSX10_10SeedDiff/modules/Darwin.html
    kern_return_t rc = semaphore_timedwait(sema_, ts);

    return rc != KERN_OPERATION_TIMED_OUT;
  }

  void signal()
  {
    semaphore_signal(sema_);
  }

  void signal(int count)
  {
    while (count-- > 0)
    {
      semaphore_signal(sema_);
    }
  }
};
#elif defined(__unix__)
//---------------------------------------------------------
// Semaphore (POSIX, Linux)
//---------------------------------------------------------
class Semaphore
{
private:
  sem_t sema_;

  Semaphore(const Semaphore& other);
  Semaphore& operator=(const Semaphore& other);

public:
  Semaphore(int initialCount = 0)
  {
    assert(initialCount >= 0);
    sem_init(&sema_, 0, initialCount);
  }

  ~Semaphore()
  {
    sem_destroy(&sema_);
  }

  void wait()
  {
    // http://stackoverflow.com/questions/2013181/gdb-causes-sem-wait-to-fail-with-eintr-error
    int rc;
    do
    {
      rc = sem_wait(&sema_);
    } while (rc == -1 && errno == EINTR);
  }

  bool tryWait()
  {
    int rc;
    do
    {
      rc = sem_trywait(&sema_);
    } while (rc == -1 && errno == EINTR);
    return !(rc == -1 && errno == EAGAIN);
  }

  bool timedWait(std::uint64_t usecs)
  {
    struct timespec ts;
    const int usecs_in_1_sec = 1000000;
    const int nsecs_in_1_sec = 1000000000;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += usecs / usecs_in_1_sec;
    ts.tv_nsec += (usecs % usecs_in_1_sec) * 1000;
    // sem_timedwait bombs if you have more than 1e9 in tv_nsec
    // so we have to clean things up before passing it in
    if (ts.tv_nsec > nsecs_in_1_sec)
    {
      ts.tv_nsec -= nsecs_in_1_sec;
      ++ts.tv_sec;
    }

    int rc;
    do
    {
      rc = sem_timedwait(&sema_, &ts);
    } while (rc == -1 && errno == EINTR);
    return !(rc == -1 && errno == ETIMEDOUT);
  }

  void signal()
  {
    sem_post(&sema_);
  }

  void signal(int count)
  {
    while (count-- > 0)
    {
      sem_post(&sema_);
    }
  }
};
#else
#error Unsupported platform! (No semaphore wrapper available)
#endif

//---------------------------------------------------------
// LightweightSemaphore
//---------------------------------------------------------
class LightweightSemaphore
{
public:
  typedef std::make_signed<std::size_t>::type ssize_t;

private:
  WeakAtomic<ssize_t> count_;
  Semaphore sema_;

  bool waitWithPartialSpinning(std::int64_t timeout_usecs = -1)
  {
    ssize_t old_count;
    // Is there a better way to set the initial spin count?
    // If we lower it to 1000, testBenaphore becomes 15x slower on my Core i7-5930K Windows PC,
    // as threads start hitting the kernel semaphore.
    int spin = 10000;
    while (--spin >= 0)
    {
      if (count_.load() > 0)
      {
        count_.fetchAddAcquire(-1);
        return true;
      }
      compilerFence(memory_order_acquire);  // Prevent the compiler from collapsing the loop.
    }
    old_count = count_.fetchAddAcquire(-1);
    if (old_count > 0)
      return true;
    if (timeout_usecs < 0)
    {
      sema_.wait();
      return true;
    }
    if (sema_.timedWait(timeout_usecs))
      return true;
    // At this point, we've timed out waiting for the semaphore, but the
    // count is still decremented indicating we may still be waiting on
    // it. So we have to re-adjust the count, but only if the semaphore
    // wasn't signaled enough times for us too since then. If it was, we
    // need to release the semaphore too.
    while (true)
    {
      old_count = count_.fetchAddRelease(1);
      if (old_count < 0)
        return false;  // successfully restored things to the way they were
      // Oh, the producer thread just signaled the semaphore after all. Try again:
      old_count = count_.fetchAddAcquire(-1);
      if (old_count > 0 && sema_.tryWait())
        return true;
    }
  }

public:
  LightweightSemaphore(ssize_t initialCount = 0) : count_(initialCount)
  {
    assert(initialCount >= 0);
  }

  bool tryWait()
  {
    if (count_.load() > 0)
    {
      count_.fetchAddAcquire(-1);
      return true;
    }
    return false;
  }

  void wait()
  {
    if (!tryWait())
      waitWithPartialSpinning();
  }

  bool wait(std::int64_t timeout_usecs)
  {
    return tryWait() || waitWithPartialSpinning(timeout_usecs);
  }

  void signal(ssize_t count = 1)
  {
    assert(count >= 0);
    ssize_t old_count = count_.fetchAddRelease(count);
    assert(old_count >= -1);
    if (old_count < 0)
    {
      sema_.signal(1);
    }
  }

  ssize_t availableApprox() const
  {
    ssize_t count = count_.load();
    return count > 0 ? count : 0;
  }
};
}  // end namespace spsc_sema
}  // end namespace moodycamel

#if defined(AE_VCPP) && (_MSC_VER < 1700 || defined(__cplusplus_cli))
#pragma warning(pop)
#ifdef __cplusplus_cli
#pragma managed(pop)
#endif
#endif
