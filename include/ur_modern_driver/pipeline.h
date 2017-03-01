#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/queue/readerwriterqueue.h"

using namespace moodycamel;
using namespace std;

template <typename T>
class IConsumer
{
public:
  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }

  virtual bool consume(unique_ptr<T> product) = 0;
};

template <typename T>
class IProducer
{
public:
  virtual void setupProducer()
  {
  }
  virtual void teardownProducer()
  {
  }
  virtual void stopProducer()
  {
  }

  virtual bool tryGet(std::vector<unique_ptr<T>>& products) = 0;
};

template <typename T>
class Pipeline
{
private:
  IProducer<T>& producer_;
  IConsumer<T>& consumer_;
  BlockingReaderWriterQueue<unique_ptr<T>> queue_;
  atomic<bool> running_;
  thread pThread_, cThread_;

  void run_producer()
  {
    producer_.setupProducer();
    std::vector<unique_ptr<T>> products;
    while (running_)
    {
      if (!producer_.tryGet(products))
      {
        break;
      }

      for (auto& p : products)
      {
        if (!queue_.try_enqueue(std::move(p)))
        {
          LOG_ERROR("Pipeline producer owerflowed!");
        }
      }

      products.clear();
    }
    producer_.teardownProducer();
    LOG_DEBUG("Pipline producer ended");
    consumer_.stopConsumer();
  }

  void run_consumer()
  {
    consumer_.setupConsumer();
    unique_ptr<T> product;
    while (running_)
    {
      // 16000us timeout was chosen because we should
      // roughly recieve messages at 125hz which is every
      // 8ms == 8000us and double it for some error margin
      if (!queue_.wait_dequeue_timed(product, 16000))
      {
        continue;
      }
      if (!consumer_.consume(std::move(product)))
        break;
    }
    consumer_.teardownConsumer();
    LOG_DEBUG("Pipline consumer ended");
    producer_.stopProducer();
  }

public:
  Pipeline(IProducer<T>& producer, IConsumer<T>& consumer)
    : producer_(producer), consumer_(consumer), queue_{ 32 }, running_{ false }
  {
  }

  void run()
  {
    if (running_)
      return;

    running_ = true;
    pThread_ = thread(&Pipeline::run_producer, this);
    cThread_ = thread(&Pipeline::run_consumer, this);
  }

  void stop()
  {
    if (!running_)
      return;

    LOG_DEBUG("Stopping pipeline");

    consumer_.stopConsumer();
    producer_.stopProducer();

    running_ = false;

    pThread_.join();
    cThread_.join();
  }
};