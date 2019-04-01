/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <atomic>
#include <chrono>
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
  virtual void onTimeout()
  {
  }

  virtual bool consume(shared_ptr<T> product) = 0;
};

template <typename T>
class MultiConsumer : public IConsumer<T>
{
private:
  std::vector<IConsumer<T>*> consumers_;

public:
  MultiConsumer(std::vector<IConsumer<T>*> consumers) : consumers_(consumers)
  {
  }

  virtual void setupConsumer()
  {
    for (auto& con : consumers_)
    {
      con->setupConsumer();
    }
  }
  virtual void teardownConsumer()
  {
    for (auto& con : consumers_)
    {
      con->teardownConsumer();
    }
  }
  virtual void stopConsumer()
  {
    for (auto& con : consumers_)
    {
      con->stopConsumer();
    }
  }
  virtual void onTimeout()
  {
    for (auto& con : consumers_)
    {
      con->onTimeout();
    }
  }

  bool consume(shared_ptr<T> product)
  {
    bool res = true;
    for (auto& con : consumers_)
    {
      if (!con->consume(product))
        res = false;
    }
    return res;
  }
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

class INotifier
{
public:
  virtual void started(std::string name)
  {
  }
  virtual void stopped(std::string name)
  {
  }
};

template <typename T>
class Pipeline
{
private:
  typedef std::chrono::high_resolution_clock Clock;
  typedef Clock::time_point Time;
  IProducer<T>& producer_;
  IConsumer<T>& consumer_;
  std::string name_;
  INotifier& notifier_;
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
          LOG_ERROR("Pipeline producer overflowed! <%s>", name_.c_str());
        }
      }

      products.clear();
    }
    producer_.teardownProducer();
    LOG_DEBUG("Pipeline producer ended! <%s>", name_.c_str());
    consumer_.stopConsumer();
    running_ = false;
    notifier_.stopped(name_);
  }

  void run_consumer()
  {
    consumer_.setupConsumer();
    unique_ptr<T> product;
    while (running_)
    {
      // timeout was chosen because we should receive messages
      // at roughly 125hz (every 8ms) and have to update
      // the controllers (i.e. the consumer) with *at least* 125Hz
      // So we update the consumer more frequently via onTimeout
      if (!queue_.wait_dequeue_timed(product, std::chrono::milliseconds(8)))
      {
        consumer_.onTimeout();
        continue;
      }

      if (!consumer_.consume(std::move(product)))
        break;
    }
    consumer_.teardownConsumer();
    LOG_DEBUG("Pipeline consumer ended! <%s>", name_.c_str());
    producer_.stopProducer();
    running_ = false;
    notifier_.stopped(name_);
  }

public:
  Pipeline(IProducer<T>& producer, IConsumer<T>& consumer, std::string name, INotifier& notifier)
    : producer_(producer), consumer_(consumer), name_(name), notifier_(notifier), queue_{ 32 }, running_{ false }
  {
  }

  void run()
  {
    if (running_)
      return;

    running_ = true;
    pThread_ = thread(&Pipeline::run_producer, this);
    cThread_ = thread(&Pipeline::run_consumer, this);
    notifier_.started(name_);
  }

  void stop()
  {
    if (!running_)
      return;

    LOG_DEBUG("Stopping pipeline! <%s>", name_.c_str());

    consumer_.stopConsumer();
    producer_.stopProducer();

    running_ = false;

    pThread_.join();
    cThread_.join();
    notifier_.stopped(name_);
  }
};
