#include <libobjdetect/core.hpp>

using namespace libobjdetect;

int main (int argc, char** argv) {
    shared_ptr<PointCloudVisualizer> visualizer(new PointCloudVisualizer);
    shared_ptr<PointCloudProducer> producer(new ExamplePointCloudProducer);
    producer->registerConsumer(visualizer);

    while (visualizer->spinOnce()) {
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
