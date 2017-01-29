#pragma once

#include <thread>
#include <atomic>
#include "ur_modern_driver/queue/readerwriterqueue.h"

using namespace moodycamel; 
using namespace std;


template <typename T>
class IProducer {
public:
    virtual void setup_producer() = 0;
    virtual void teardown_producer() = 0;
    virtual void stop_producer() = 0;
    virtual unique_ptr<T> try_get() = 0;
};

template <typename T>
class IConsumer {
public:
    virtual void setup_consumer() = 0;
    virtual void teardown_consumer() = 0;
    virtual void stop_consumer() = 0;
    virtual bool push(unique_ptr<T> product) = 0;
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
        while(_running) {
            unique_ptr<T> product(_producer.try_get());
            
            if(product == nullptr)
                break;

            if(!_queue.try_enqueue(std::move(product))) {
                //log dropped product
            }
        }
        _producer.teardown_producer();
        //todo cleanup
    }

    void run_consumer() {
        _consumer.setup_consumer();
        while(_running) {
            unique_ptr<T> product;
            _queue.wait_dequeue(product);
            if(!_consumer.push(std::move(product))) 
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

        _pThread.join();
        _cThread.join();
    }
};