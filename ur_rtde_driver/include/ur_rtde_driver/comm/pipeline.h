/*
 * Copyright 2019, FZI Forschungszentrum Informatik (templating)
 *
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

#include "ur_rtde_driver/comm/package.h"
#include "ur_rtde_driver/log.h"
#include "ur_rtde_driver/queue/readerwriterqueue.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>

namespace ur_driver
{
namespace comm
{
// TODO: Remove these!!!
using namespace moodycamel;

template <typename T>
class IConsumer
{
public:
  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
    stopConsumer();
  }
  virtual void stopConsumer()
  {
  }
  virtual void onTimeout()
  {
  }

  virtual bool consume(std::shared_ptr<T> product) = 0;
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

  bool consume(std::shared_ptr<T> product)
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

template <typename HeaderT>
class IProducer
{
public:
  virtual void setupProducer()
  {
  }
  virtual void teardownProducer()
  {
    stopProducer();
  }
  virtual void stopProducer()
  {
  }

  virtual bool tryGet(std::vector<std::unique_ptr<URPackage<HeaderT>>>& products) = 0;
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

template <typename HeaderT>
class Pipeline
{
public:
  typedef std::chrono::high_resolution_clock Clock;
  typedef Clock::time_point Time;
  using _package_type = URPackage<HeaderT>;
  Pipeline(IProducer<HeaderT>& producer, IConsumer<_package_type>& consumer, std::string name, INotifier& notifier)
    : producer_(producer), consumer_(&consumer), name_(name), notifier_(notifier), queue_{ 32 }, running_{ false }
  {
  }
  Pipeline(IProducer<HeaderT>& producer, std::string name, INotifier& notifier)
    : producer_(producer), consumer_(nullptr), name_(name), notifier_(notifier), queue_{ 32 }, running_{ false }
  {
  }

  virtual ~Pipeline()
  {
    LOG_DEBUG("Destructing pipeline");
    stop();
  }

  void run()
  {
    if (running_)
      return;

    running_ = true;
    producer_.setupProducer();
    pThread_ = std::thread(&Pipeline::runProducer, this);
    if (consumer_ != nullptr)
      cThread_ = std::thread(&Pipeline::runConsumer, this);
    notifier_.started(name_);
  }

  void stop()
  {
    if (!running_)
      return;

    LOG_DEBUG("Stopping pipeline! <%s>", name_.c_str());

    running_ = false;

    if (pThread_.joinable())
    {
      pThread_.join();
    }
    if (cThread_.joinable())
    {
      cThread_.join();
    }
    notifier_.stopped(name_);
  }

  bool getLatestProduct(std::unique_ptr<URPackage<HeaderT>>& product, std::chrono::milliseconds timeout)
  {
    return queue_.waitDequeTimed(product, timeout);
  }

private:
  IProducer<HeaderT>& producer_;
  IConsumer<_package_type>* consumer_;
  std::string name_;
  INotifier& notifier_;
  BlockingReaderWriterQueue<std::unique_ptr<_package_type>> queue_;
  std::atomic<bool> running_;
  std::thread pThread_, cThread_;

  void runProducer()
  {
    LOG_DEBUG("Starting up producer");
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    bool has_realtime;
    realtime_file >> has_realtime;
    if (has_realtime)
    {
      const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
      if (max_thread_priority != -1)
      {
        // We'll operate on the currently running thread.
        pthread_t this_thread = pthread_self();

        // struct sched_param is used to store the scheduling priority
        struct sched_param params;

        // We'll set the priority to the maximum.
        params.sched_priority = max_thread_priority;

        int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
        if (ret != 0)
        {
          LOG_ERROR("Unsuccessful in setting producer thread realtime priority. Error code: %d", ret);
        }
        // Now verify the change in thread priority
        int policy = 0;
        ret = pthread_getschedparam(this_thread, &policy, &params);
        if (ret != 0)
        {
          std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;
        }

        // Check the correct policy was applied
        if (policy != SCHED_FIFO)
        {
          LOG_ERROR("Producer thread: Scheduling is NOT SCHED_FIFO!");
        }
        else
        {
          LOG_INFO("Producer thread: SCHED_FIFO OK");
        }

        // Print thread scheduling priority
        LOG_INFO("Thread priority is %d", params.sched_priority);
      }
      else
      {
        LOG_ERROR("Could not get maximum thread priority for producer thread");
      }
    }
    else
    {
      LOG_WARN("No realtime capabilities found. Consider using a realtime system for better performance");
    }
    std::vector<std::unique_ptr<_package_type>> products;
    while (running_)
    {
      if (!producer_.tryGet(products))
      {
        break;
      }

      for (auto& p : products)
      {
        if (!queue_.tryEnqueue(std::move(p)))
        {
          LOG_ERROR("Pipeline producer overflowed! <%s>", name_.c_str());
        }
      }

      products.clear();
    }
    producer_.teardownProducer();
    LOG_DEBUG("Pipeline producer ended! <%s>", name_.c_str());
    running_ = false;
    notifier_.stopped(name_);
  }

  void runConsumer()
  {
    consumer_->setupConsumer();
    std::unique_ptr<_package_type> product;
    while (running_)
    {
      // timeout was chosen because we should receive messages
      // at roughly 125hz (every 8ms) and have to update
      // the controllers (i.e. the consumer) with *at least* 125Hz
      // So we update the consumer more frequently via onTimeout
      if (!queue_.waitDequeTimed(product, std::chrono::milliseconds(8)))
      {
        consumer_->onTimeout();
        continue;
      }

      if (!consumer_->consume(std::move(product)))
        break;
    }
    consumer_->teardownConsumer();
    LOG_DEBUG("Pipeline consumer ended! <%s>", name_.c_str());
    running_ = false;
    notifier_.stopped(name_);
  }
};
}  // namespace comm
}  // namespace ur_driver
