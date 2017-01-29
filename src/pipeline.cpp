#include "ur_modern_driver/queue/readerwriterqueue.h"

using namespace moodycamel; 
using namespace std;

class UR2ROSRelay {
private:
    URReciever reciever;
    URParser parser;

    BlockingReaderWriterQueue<unique_ptr<URMessage>> queue;

    ROSConverter converter;
    ROSPublisher publisher;


public:
    UR2ROSRelay(string &host, int port) : reciever{host, port} { }

    void run() {

    }
};
