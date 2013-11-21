#include <libobjdetect/core.hpp>
#include <libobjdetect/io.hpp>
#include <libobjdetect/viewer.hpp>

#include <boost/make_shared.hpp>
#include <pcl/console/parse.h>

using namespace libobjdetect;
using namespace boost;
using namespace pcl;
using namespace pcl::visualization;

class ExamplePointCloudProducer : public PointCloudProducer {
private:
    PointCloud<Point>::Ptr examplePointCloud;

public:
    ExamplePointCloudProducer();
    void stop() {}
    void registerConsumer(shared_ptr<PointCloudConsumer> consumer) {
        consumer->consumePointCloud(examplePointCloud);
    }
};

int main (int argc, char** argv) {
    shared_ptr<PointCloudViewer> viewer(new PointCloudViewer);
    shared_ptr<PointCloudProducer> producer;

    if (pcl::console::find_argument(argc, argv, "--example") >= 0) {
        producer = make_shared<ExamplePointCloudProducer>();
    } else {
        producer = make_shared<KinectPointCloudProducer>();
    }

    producer->registerConsumer(viewer);
    while (!viewer->wasStopped()) {}
    producer->stop();

    return 0;
}

ExamplePointCloudProducer::ExamplePointCloudProducer()
    : examplePointCloud(new PointCloud<Point>)
{
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05) {
        for (float angle(0.0); angle <= 360.0; angle += 5.0) {
            Point point;
            point.x = 0.5 * cosf(deg2rad(angle));
            point.y = sinf(deg2rad(angle));
            point.z = z;

            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);

            examplePointCloud->points.push_back(point);
        }

        if (z < 0.0) {
            r -= 12;
            g += 12;
        } else {
            g -= 12;
            b += 12;
        }
    }

    examplePointCloud->width = (int) examplePointCloud->points.size ();
    examplePointCloud->height = 1;
}
