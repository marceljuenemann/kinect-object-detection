#include <libobjdetect/core.hpp>
#include <libobjdetect/io.hpp>

using namespace libobjdetect;

int main (int argc, char** argv) {
    shared_ptr<PointCloudViewer> viewer(new PointCloudViewer);
    shared_ptr<PointCloudProducer> producer(new KinectPointCloudProducer);
    producer->registerConsumer(viewer);

    while (!viewer->wasStopped()) {}
    producer->stop();

    return 0;
}
