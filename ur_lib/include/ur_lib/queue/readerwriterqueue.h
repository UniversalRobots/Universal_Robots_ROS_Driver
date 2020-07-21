// ©2013-2016 Cameron Desrochers.
// Distributed under the simplified BSD license (see the license file that
// should have come with this header).

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdlib>  // For malloc/free/abort & size_t
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include "atomicops.h"
#if __cplusplus > 199711L || _MSC_VER >= 1700  // C++11 or VS2012
#include <chrono>
#endif

// A lock-free queue for a single-consumer, single-producer architecture.
// The queue is also wait-free in the common path (except if more memory
// needs to be allocated, in which case malloc is called).
// Allocates memory sparingly (O(lg(n) times, amortized), and only once if
// the original maximum size estimate is never exceeded.
// Tested on x86/x64 processors, but semantics should be correct for all
// architectures (given the right implementations in atomicops.h), provided
// that aligned integer and pointer accesses are naturally atomic.
// Note that there should only be one consumer thread and producer thread;
// Switching roles of the threads, or using multiple consecutive threads for
// one role, is not safe unless properly synchronized.
// Using the queue exclusively from one thread is fine, though a bit silly.

#ifndef MOODYCAMEL_CACHE_LINE_SIZE
#define MOODYCAMEL_CACHE_LINE_SIZE 64
#endif

#ifndef MOODYCAMEL_EXCEPTIONS_ENABLED
#if (defined(_MSC_VER) && defined(_CPPUNWIND)) || (defined(__GNUC__) && defined(__EXCEPTIONS)) ||                      \
    (!defined(_MSC_VER) && !defined(__GNUC__))
#define MOODYCAMEL_EXCEPTIONS_ENABLED
#endif
#endif

#ifdef AE_VCPP
#pragma warning(push)
#pragma warning(disable : 4324)  // structure was padded due to __declspec(align())
#pragma warning(disable : 4820)  // padding was added
#pragma warning(disable : 4127)  // conditional expression is constant
#endif

namespace moodycamel
{
template <typename T, size_t MAX_BLOCK_SIZE = 512>
class ReaderWriterQueue
{
  // Design: Based on a queue-of-queues. The low-level queues are just
  // circular buffers with front and tail indices indicating where the
  // next element to dequeue is and where the next element can be enqueued,
  // respectively. Each low-level queue is called a "block". Each block
  // wastes exactly one element's worth of space to keep the design simple
  // (if front == tail then the queue is empty, and can't be full).
  // The high-level queue is a circular linked list of blocks; again there
  // is a front and tail, but this time they are pointers to the blocks.
  // The front block is where the next element to be dequeued is, provided
  // the block is not empty. The back block is where elements are to be
  // enqueued, provided the block is not full.
  // The producer thread owns all the tail indices/pointers. The consumer
  // thread owns all the front indices/pointers. Both threads read each
  // other's variables, but only the owning thread updates them. E.g. After
  // the consumer reads the producer's tail, the tail may change before the
  // consumer is done dequeuing_ an object, but the consumer knows the tail
  // will never go backwards, only forwards.
  // If there is no room to enqueue an object, an additional block (of
  // equal size to the last block) is added. Blocks are never removed.

public:
  // Constructs a queue that can hold maxSize elements without further
  // allocations. If more than MAX_BLOCK_SIZE elements are requested,
  // then several blocks of MAX_BLOCK_SIZE each are reserved (including
  // at least one extra buffer block).
  explicit ReaderWriterQueue(size_t max_size = 15)
#ifndef NDEBUG
    : enqueuing_(false), dequeuing_(false)
#endif
  {
    assert(max_size > 0);
    assert(MAX_BLOCK_SIZE == ceilToPow2(MAX_BLOCK_SIZE) && "MAX_BLOCK_SIZE must be a power of 2");
    assert(MAX_BLOCK_SIZE >= 2 && "MAX_BLOCK_SIZE must be at least 2");

    Block* first_block = nullptr;

    largest_block_size_ = ceilToPow2(max_size + 1);  // We need a spare slot to fit maxSize elements in the block
    if (largest_block_size_ > MAX_BLOCK_SIZE * 2)
    {
      // We need a spare block in case the producer is writing to a different block the consumer is reading from, and
      // wants to enqueue the maximum number of elements. We also need a spare element in each block to avoid the
      // ambiguity
      // between front == tail meaning "empty" and "full".
      // So the effective number of slots that are guaranteed to be usable at any time is the block size - 1 times the
      // number of blocks - 1. Solving for maxSize and applying a ceiling to the division gives us (after simplifying):
      size_t initial_block_count = (max_size + MAX_BLOCK_SIZE * 2 - 3) / (MAX_BLOCK_SIZE - 1);
      largest_block_size_ = MAX_BLOCK_SIZE;
      Block* last_block = nullptr;
      for (size_t i = 0; i != initial_block_count; ++i)
      {
        auto block = makeBlock(largest_block_size_);
        if (block == nullptr)
        {
#ifdef MOODYCAMEL_EXCEPTIONS_ENABLED
          throw std::bad_alloc();
#else
          abort();
#endif
        }
        if (first_block == nullptr)
        {
          first_block = block;
        }
        else
        {
          last_block->next = block;
        }
        last_block = block;
        block->next = first_block;
      }
    }
    else
    {
      first_block = makeBlock(largest_block_size_);
      if (first_block == nullptr)
      {
#ifdef MOODYCAMEL_EXCEPTIONS_ENABLED
        throw std::bad_alloc();
#else
        abort();
#endif
      }
      first_block->next = first_block;
    }
    front_block_ = first_block;
    tail_block_ = first_block;

    // Make sure the reader/writer threads will have the initialized memory setup above:
    fence(memory_order_sync);
  }

  // Note: The queue should not be accessed concurrently while it's
  // being deleted. It's up to the user to synchronize this.
  ~ReaderWriterQueue()
  {
    // Make sure we get the latest version of all variables from other CPUs:
    fence(memory_order_sync);

    // Destroy any remaining objects in queue and free memory
    Block* front_block = front_block_;
    Block* block = front_block;
    do
    {
      Block* next_block = block->next;
      size_t block_front = block->front;
      size_t block_tail = block->tail;

      for (size_t i = block_front; i != block_tail; i = (i + 1) & block->sizeMask)
      {
        auto element = reinterpret_cast<T*>(block->data + i * sizeof(T));
        element->~T();
        (void)element;
      }

      auto raw_block = block->rawThis;
      block->~Block();
      std::free(raw_block);
      block = next_block;
    } while (block != front_block);
  }

  // Enqueues a copy of element if there is room in the queue.
  // Returns true if the element was enqueued, false otherwise.
  // Does not allocate memory.
  AE_FORCEINLINE bool tryEnqueue(T const& element)
  {
    return innerEnqueue<CannotAlloc>(element);
  }

  // Enqueues a moved copy of element if there is room in the queue.
  // Returns true if the element was enqueued, false otherwise.
  // Does not allocate memory.
  AE_FORCEINLINE bool tryEnqueue(T&& element)
  {
    return innerEnqueue<CannotAlloc>(std::forward<T>(element));
  }

  // Enqueues a copy of element on the queue.
  // Allocates an additional block of memory if needed.
  // Only fails (returns false) if memory allocation fails.
  AE_FORCEINLINE bool enqueue(T const& element)
  {
    return innerEnqueue<CanAlloc>(element);
  }

  // Enqueues a moved copy of element on the queue.
  // Allocates an additional block of memory if needed.
  // Only fails (returns false) if memory allocation fails.
  AE_FORCEINLINE bool enqueue(T&& element)
  {
    return innerEnqueue<CanAlloc>(std::forward<T>(element));
  }

  // Attempts to dequeue an element; if the queue is empty,
  // returns false instead. If the queue has at least one element,
  // moves front to result using operator=, then returns true.
  template <typename U>
  bool tryDequeue(U& result)
  {
#ifndef NDEBUG
    ReentrantGuard guard(this->dequeuing_);
#endif

    // High-level pseudocode:
    // Remember where the tail block is
    // If the front block has an element in it, dequeue it
    // Else
    //     If front block was the tail block when we entered the function, return false
    //     Else advance to next block and dequeue the item there

    // Note that we have to use the value of the tail block from before we check if the front
    // block is full or not, in case the front block is empty and then, before we check if the
    // tail block is at the front block or not, the producer fills up the front block *and
    // moves on*, which would make us skip a filled block. Seems unlikely, but was consistently
    // reproducible in practice.
    // In order to avoid overhead in the common case, though, we do a double-checked pattern
    // where we have the fast path if the front block is not empty, then read the tail block,
    // then re-read the front block and check if it's not empty again, then check if the tail
    // block has advanced.

    Block* front_block = front_block_.load();
    size_t block_tail = front_block->localTail;
    size_t block_front = front_block->front.load();

    if (block_front != block_tail || block_front != (front_block->localTail = front_block->tail.load()))
    {
      fence(memory_order_acquire);

    non_empty_front_block:
      // Front block not empty, dequeue from here
      auto element = reinterpret_cast<T*>(front_block->data + block_front * sizeof(T));
      result = std::move(*element);
      element->~T();

      block_front = (block_front + 1) & front_block->sizeMask;

      fence(memory_order_release);
      front_block->front = block_front;
    }
    else if (front_block != tail_block_.load())
    {
      fence(memory_order_acquire);

      front_block = front_block_.load();
      block_tail = front_block->localTail = front_block->tail.load();
      block_front = front_block->front.load();
      fence(memory_order_acquire);

      if (block_front != block_tail)
      {
        // Oh look, the front block isn't empty after all
        goto non_empty_front_block;
      }

      // Front block is empty but there's another block ahead, advance to it
      Block* next_block = front_block->next;
      // Don't need an acquire fence here since next can only ever be set on the tail_block,
      // and we're not the tail_block, and we did an acquire earlier after reading tail_block which
      // ensures next is up-to-date on this CPU in case we recently were at tail_block.

      size_t next_block_front = next_block->front.load();
      size_t next_block_tail = next_block->localTail = next_block->tail.load();
      fence(memory_order_acquire);

      // Since the tail_block is only ever advanced after being written to,
      // we know there's for sure an element to dequeue on it
      assert(next_block_front != next_block_tail);
      AE_UNUSED(next_block_tail);

      // We're done with this block, let the producer use it if it needs
      fence(memory_order_release);  // Expose possibly pending changes to front_block->front from last dequeue
      front_block_ = front_block = next_block;

      compilerFence(memory_order_release);  // Not strictly needed

      auto element = reinterpret_cast<T*>(front_block->data + next_block_front * sizeof(T));

      result = std::move(*element);
      element->~T();

      next_block_front = (next_block_front + 1) & front_block->sizeMask;

      fence(memory_order_release);
      front_block->front = next_block_front;
    }
    else
    {
      // No elements in current block and no other block to advance to
      return false;
    }

    return true;
  }

  // Returns a pointer to the front element in the queue (the one that
  // would be removed next by a call to `tryDequeue` or `pop`). If the
  // queue appears empty at the time the method is called, nullptr is
  // returned instead.
  // Must be called only from the consumer thread.
  T* peek()
  {
#ifndef NDEBUG
    ReentrantGuard guard(this->dequeuing_);
#endif
    // See tryDequeue() for reasoning

    Block* front_block = front_block_.load();
    size_t block_tail = front_block->localTail;
    size_t block_front = front_block->front.load();

    if (block_front != block_tail || block_front != (front_block->localTail = front_block->tail.load()))
    {
      fence(memory_order_acquire);
    non_empty_front_block:
      return reinterpret_cast<T*>(front_block->data + block_front * sizeof(T));
    }
    else if (front_block != tail_block_.load())
    {
      fence(memory_order_acquire);
      front_block = front_block_.load();
      block_tail = front_block->localTail = front_block->tail.load();
      block_front = front_block->front.load();
      fence(memory_order_acquire);

      if (block_front != block_tail)
      {
        goto non_empty_front_block;
      }

      Block* next_block = front_block->next;

      size_t next_block_front = next_block->front.load();
      fence(memory_order_acquire);

      assert(next_block_front != next_block->tail.load());
      return reinterpret_cast<T*>(next_block->data + next_block_front * sizeof(T));
    }

    return nullptr;
  }

  // Removes the front element from the queue, if any, without returning it.
  // Returns true on success, or false if the queue appeared empty at the time
  // `pop` was called.
  bool pop()
  {
#ifndef NDEBUG
    ReentrantGuard guard(this->dequeuing_);
#endif
    // See tryDequeue() for reasoning

    Block* front_block = front_block_.load();
    size_t block_tail = front_block->localTail;
    size_t block_front = front_block->front.load();

    if (block_front != block_tail || block_front != (front_block->localTail = front_block->tail.load()))
    {
      fence(memory_order_acquire);

    non_empty_front_block:
      auto element = reinterpret_cast<T*>(front_block->data + block_front * sizeof(T));
      element->~T();

      block_front = (block_front + 1) & front_block->sizeMask;

      fence(memory_order_release);
      front_block->front = block_front;
    }
    else if (front_block != tail_block_.load())
    {
      fence(memory_order_acquire);
      front_block = front_block_.load();
      block_tail = front_block->localTail = front_block->tail.load();
      block_front = front_block->front.load();
      fence(memory_order_acquire);

      if (block_front != block_tail)
      {
        goto non_empty_front_block;
      }

      // Front block is empty but there's another block ahead, advance to it
      Block* next_block = front_block->next;

      size_t next_block_front = next_block->front.load();
      size_t next_block_tail = next_block->localTail = next_block->tail.load();
      fence(memory_order_acquire);

      assert(next_block_front != next_block_tail);
      AE_UNUSED(next_block_tail);

      fence(memory_order_release);
      front_block_ = front_block = next_block;

      compilerFence(memory_order_release);

      auto element = reinterpret_cast<T*>(front_block->data + next_block_front * sizeof(T));
      element->~T();

      next_block_front = (next_block_front + 1) & front_block->sizeMask;

      fence(memory_order_release);
      front_block->front = next_block_front;
    }
    else
    {
      // No elements in current block and no other block to advance to
      return false;
    }

    return true;
  }

  // Returns the approximate number of items currently in the queue.
  // Safe to call from both the producer and consumer threads.
  inline size_t sizeApprox() const
  {
    size_t result = 0;
    Block* front_block = front_block_.load();
    Block* block = front_block;
    do
    {
      fence(memory_order_acquire);
      size_t block_front = block->front.load();
      size_t block_tail = block->tail.load();
      result += (block_tail - block_front) & block->sizeMask;
      block = block->next.load();
    } while (block != front_block);
    return result;
  }

private:
  enum AllocationMode
  {
    CanAlloc,
    CannotAlloc
  };

  template <AllocationMode canAlloc, typename U>
  bool innerEnqueue(U&& element)
  {
#ifndef NDEBUG
    ReentrantGuard guard(this->enqueuing_);
#endif

    // High-level pseudocode (assuming we're allowed to alloc a new block):
    // If room in tail block, add to tail
    // Else check next block
    //     If next block is not the head block, enqueue on next block
    //     Else create a new block and enqueue there
    //     Advance tail to the block we just enqueued to

    Block* tail_block = tail_block_.load();
    size_t block_front = tail_block->localFront;
    size_t block_tail = tail_block->tail.load();

    size_t next_block_tail = (block_tail + 1) & tail_block->sizeMask;
    if (next_block_tail != block_front || next_block_tail != (tail_block->localFront = tail_block->front.load()))
    {
      fence(memory_order_acquire);
      // This block has room for at least one more element
      char* location = tail_block->data + block_tail * sizeof(T);
      new (location) T(std::forward<U>(element));

      fence(memory_order_release);
      tail_block->tail = next_block_tail;
    }
    else
    {
      fence(memory_order_acquire);
      if (tail_block->next.load() != front_block_)
      {
        // Note that the reason we can't advance to the front_block and start adding new entries there
        // is because if we did, then dequeue would stay in that block, eventually reading the new values,
        // instead of advancing to the next full block (whose values were enqueued first and so should be
        // consumed first).

        fence(memory_order_acquire);  // Ensure we get latest writes if we got the latest front_block

        // tail_block is full, but there's a free block ahead, use it
        Block* tail_block_next = tail_block->next.load();
        size_t next_block_front = tail_block_next->localFront = tail_block_next->front.load();
        next_block_tail = tail_block_next->tail.load();
        fence(memory_order_acquire);

        // This block must be empty since it's not the head block and we
        // go through the blocks in a circle
        assert(next_block_front == next_block_tail);
        tail_block_next->localFront = next_block_front;

        char* location = tail_block_next->data + next_block_tail * sizeof(T);
        new (location) T(std::forward<U>(element));

        tail_block_next->tail = (next_block_tail + 1) & tail_block_next->sizeMask;

        fence(memory_order_release);
        tail_block_ = tail_block_next;
      }
      else if (canAlloc == CanAlloc)
      {
        // tail_block is full and there's no free block ahead; create a new block
        auto new_block_size = largest_block_size_ >= MAX_BLOCK_SIZE ? largest_block_size_ : largest_block_size_ * 2;
        auto new_block = makeBlock(new_block_size);
        if (new_block == nullptr)
        {
          // Could not allocate a block!
          return false;
        }
        largest_block_size_ = new_block_size;

        new (new_block->data) T(std::forward<U>(element));

        assert(new_block->front == 0);
        new_block->tail = new_block->localTail = 1;

        new_block->next = tail_block->next.load();
        tail_block->next = new_block;

        // Might be possible for the dequeue thread to see the new tail_block->next
        // *without* seeing the new tail_block value, but this is OK since it can't
        // advance to the next block until tail_block is set anyway (because the only
        // case where it could try to read the next is if it's already at the tail_block,
        // and it won't advance past tail_block in any circumstance).

        fence(memory_order_release);
        tail_block_ = new_block;
      }
      else if (canAlloc == CannotAlloc)
      {
        // Would have had to allocate a new block to enqueue, but not allowed
        return false;
      }
      else
      {
        assert(false && "Should be unreachable code");
        return false;
      }
    }

    return true;
  }

  // Disable copying
  ReaderWriterQueue(ReaderWriterQueue const&)
  {
  }

  // Disable assignment
  ReaderWriterQueue& operator=(ReaderWriterQueue const&)
  {
  }

  AE_FORCEINLINE static size_t ceilToPow2(size_t x)
  {
    // From http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    --x;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    for (size_t i = 1; i < sizeof(size_t); i <<= 1)
    {
      x |= x >> (i << 3);
    }
    ++x;
    return x;
  }

  template <typename U>
  static AE_FORCEINLINE char* alignFor(char* ptr)
  {
    const std::size_t alignment = std::alignment_of<U>::value;
    return ptr + (alignment - (reinterpret_cast<std::uintptr_t>(ptr) % alignment)) % alignment;
  }

private:
#ifndef NDEBUG
  struct ReentrantGuard
  {
    ReentrantGuard(bool& in_section) : in_section_(in_section)
    {
      assert(!in_section_ && "ReaderWriterQueue does not support enqueuing_ or dequeuing_ elements from other "
                             "elements' "
                             "ctors and dtors");
      in_section_ = true;
    }

    ~ReentrantGuard()
    {
      in_section_ = false;
    }

  private:
    ReentrantGuard& operator=(ReentrantGuard const&);

  private:
    bool& in_section_;
  };
#endif

  struct Block
  {
    // Avoid false-sharing by putting highly contended variables on their own cache lines
    WeakAtomic<size_t> front;  // (Atomic) Elements are read from here
    size_t localTail;          // An uncontended shadow copy of tail, owned by the consumer

    char cachelineFiller0_[MOODYCAMEL_CACHE_LINE_SIZE - sizeof(WeakAtomic<size_t>) - sizeof(size_t)];
    WeakAtomic<size_t> tail;  // (Atomic) Elements are enqueued here
    size_t localFront;

    char cachelineFiller1_[MOODYCAMEL_CACHE_LINE_SIZE - sizeof(WeakAtomic<size_t>) - sizeof(size_t)];  // next isn't
                                                                                                       // very
                                                                                                       // contended, but
                                                                                                       // we don't want
                                                                                                       // it on the same
                                                                                                       // cache line as
                                                                                                       // tail (which
                                                                                                       // is)
    WeakAtomic<Block*> next;                                                                           // (Atomic)

    char* data;  // Contents (on heap) are aligned to T's alignment

    const size_t sizeMask;

    // size must be a power of two (and greater than 0)
    Block(size_t const& _size, char* _rawThis, char* _data)
      : front(0)
      , localTail(0)
      , tail(0)
      , localFront(0)
      , next(nullptr)
      , data(_data)
      , sizeMask(_size - 1)
      , rawThis(_rawThis)
    {
    }

  private:
    // C4512 - Assignment operator could not be generated
    Block& operator=(Block const&);

  public:
    char* rawThis;
  };

  static Block* makeBlock(size_t capacity)
  {
    // Allocate enough memory for the block itself, as well as all the elements it will contain
    auto size = sizeof(Block) + std::alignment_of<Block>::value - 1;
    size += sizeof(T) * capacity + std::alignment_of<T>::value - 1;
    auto new_block_raw = static_cast<char*>(std::malloc(size));
    if (new_block_raw == nullptr)
    {
      return nullptr;
    }

    auto new_block_aligned = alignFor<Block>(new_block_raw);
    auto new_block_data = alignFor<T>(new_block_aligned + sizeof(Block));
    return new (new_block_aligned) Block(capacity, new_block_raw, new_block_data);
  }

private:
  WeakAtomic<Block*> front_block_;  // (Atomic) Elements are enqueued to this block

  char cachelineFiller_[MOODYCAMEL_CACHE_LINE_SIZE - sizeof(WeakAtomic<Block*>)];
  WeakAtomic<Block*> tail_block_;  // (Atomic) Elements are dequeued from this block

  size_t largest_block_size_;

#ifndef NDEBUG
  bool enqueuing_;
  bool dequeuing_;
#endif
};

// Like ReaderWriterQueue, but also providees blocking operations
template <typename T, size_t MAX_BLOCK_SIZE = 512>
class BlockingReaderWriterQueue
{
private:
  typedef ::moodycamel::ReaderWriterQueue<T, MAX_BLOCK_SIZE> ReaderWriterQueue;

public:
  explicit BlockingReaderWriterQueue(size_t maxSize = 15) : inner_(maxSize)
  {
  }

  // Enqueues a copy of element if there is room in the queue.
  // Returns true if the element was enqueued, false otherwise.
  // Does not allocate memory.
  AE_FORCEINLINE bool tryEnqueue(T const& element)
  {
    if (inner_.tryEnqueue(element))
    {
      sema_.signal();
      return true;
    }
    return false;
  }

  // Enqueues a moved copy of element if there is room in the queue.
  // Returns true if the element was enqueued, false otherwise.
  // Does not allocate memory.
  AE_FORCEINLINE bool tryEnqueue(T&& element)
  {
    if (inner_.tryEnqueue(std::forward<T>(element)))
    {
      sema_.signal();
      return true;
    }
    return false;
  }

  // Enqueues a copy of element on the queue.
  // Allocates an additional block of memory if needed.
  // Only fails (returns false) if memory allocation fails.
  AE_FORCEINLINE bool enqueue(T const& element)
  {
    if (inner_.enqueue(element))
    {
      sema_.signal();
      return true;
    }
    return false;
  }

  // Enqueues a moved copy of element on the queue.
  // Allocates an additional block of memory if needed.
  // Only fails (returns false) if memory allocation fails.
  AE_FORCEINLINE bool enqueue(T&& element)
  {
    if (inner_.enqueue(std::forward<T>(element)))
    {
      sema_.signal();
      return true;
    }
    return false;
  }

  // Attempts to dequeue an element; if the queue is empty,
  // returns false instead. If the queue has at least one element,
  // moves front to result using operator=, then returns true.
  template <typename U>
  bool tryDequeue(U& result)
  {
    if (sema_.tryWait())
    {
      bool success = inner_.tryDequeue(result);
      assert(success);
      AE_UNUSED(success);
      return true;
    }
    return false;
  }

  // Attempts to dequeue an element; if the queue is empty,
  // waits until an element is available, then dequeues it.
  template <typename U>
  void waitDequeue(U& result)
  {
    sema_.wait();
    bool success = inner_.tryDequeue(result);
    AE_UNUSED(result);
    assert(success);
    AE_UNUSED(success);
  }

  // Attempts to dequeue an element; if the queue is empty,
  // waits until an element is available up to the specified timeout,
  // then dequeues it and returns true, or returns false if the timeout
  // expires before an element can be dequeued.
  // Using a negative timeout indicates an indefinite timeout,
  // and is thus functionally equivalent to calling waitDequeue.
  template <typename U>
  bool waitDequeTimed(U& result, std::int64_t timeout_usecs)
  {
    if (!sema_.wait(timeout_usecs))
    {
      return false;
    }
    bool success = inner_.tryDequeue(result);
    AE_UNUSED(result);
    assert(success);
    AE_UNUSED(success);
    return true;
  }

#if __cplusplus > 199711L || _MSC_VER >= 1700
  // Attempts to dequeue an element; if the queue is empty,
  // waits until an element is available up to the specified timeout,
  // then dequeues it and returns true, or returns false if the timeout
  // expires before an element can be dequeued.
  // Using a negative timeout indicates an indefinite timeout,
  // and is thus functionally equivalent to calling waitDequeue.
  template <typename U, typename Rep, typename Period>
  inline bool waitDequeTimed(U& result, std::chrono::duration<Rep, Period> const& timeout)
  {
    return waitDequeTimed(result, std::chrono::duration_cast<std::chrono::microseconds>(timeout).count());
  }
#endif

  // Returns a pointer to the front element in the queue (the one that
  // would be removed next by a call to `tryDequeue` or `pop`). If the
  // queue appears empty at the time the method is called, nullptr is
  // returned instead.
  // Must be called only from the consumer thread.
  AE_FORCEINLINE T* peek()
  {
    return inner_.peek();
  }

  // Removes the front element from the queue, if any, without returning it.
  // Returns true on success, or false if the queue appeared empty at the time
  // `pop` was called.
  AE_FORCEINLINE bool pop()
  {
    if (sema_.tryWait())
    {
      bool result = inner_.pop();
      assert(result);
      AE_UNUSED(result);
      return true;
    }
    return false;
  }

  // Returns the approximate number of items currently in the queue.
  // Safe to call from both the producer and consumer threads.
  AE_FORCEINLINE size_t sizeApprox() const
  {
    return sema_.availableApprox();
  }

private:
  // Disable copying & assignment
  BlockingReaderWriterQueue(ReaderWriterQueue const&)
  {
  }
  BlockingReaderWriterQueue& operator=(ReaderWriterQueue const&)
  {
  }

private:
  ReaderWriterQueue inner_;
  spsc_sema::LightweightSemaphore sema_;
};

}  // end namespace moodycamel

#ifdef AE_VCPP
#pragma warning(pop)
#endif
