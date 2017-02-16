#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/queue/readerwriterqueue.h"

using namespace moodycamel; 
using namespace std;


template <typename T>
class IConsumer {
public:
    virtual void setup_consumer() { }
    virtual void teardown_consumer() { }
    virtual void stop_consumer() { }

    virtual bool consume(unique_ptr<T> product) = 0;
};

template <typename T>
class IProducer {
public:
    virtual void setup_producer() { }
    virtual void teardown_producer() { }
    virtual void stop_producer() { }

    virtual bool try_get(std::vector<unique_ptr<T>> &products) = 0;
};


template <typename T>
class Pipeline {
private:
    IProducer<T> &_producer;
    IConsumer<T> &_consumer;
    BlockingReaderWriterQueue<unique_ptr<T>> _queue;
    atomic<bool> _running;
    thread _pThread, _cThread;

    void run_producer() {
        _producer.setup_producer();
        std::vector<unique_ptr<T>> products;
        while(_running) {
            if(!_producer.try_get(products)) {
                break;
            }
            
            for(auto &p : products) {
                if(!_queue.try_enqueue(std::move(p))) {
                    LOG_WARN("Pipeline owerflowed!");
                }
            }

            products.clear();
        }
        _producer.teardown_producer();
        //todo cleanup
    }

    void run_consumer() {
        _consumer.setup_consumer();
        unique_ptr<T> product;
        while(_running) {
            _queue.wait_dequeue(product);
            if(!_consumer.consume(std::move(product))) 
                break;
        }
        _consumer.teardown_consumer();
        //todo cleanup
    }
public:
    Pipeline(IProducer<T> &producer, IConsumer<T> &consumer) 
    : _producer(producer), 
    _consumer(consumer), 
    _queue{32}, 
    _running{false} 
    { }

    void run() {
        if(_running) 
            return;

        _running = true;
        _pThread = thread(&Pipeline::run_producer, this);
        _cThread = thread(&Pipeline::run_consumer, this);
    }

    void stop() {
        if(!_running)
            return;
        
        _consumer.stop_consumer();
        _producer.stop_producer();

        _running = false;

        _pThread.join();
        _cThread.join();
    }
};