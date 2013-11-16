#include "core.hpp"

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

namespace libobjdetect {

    ////////////////////////////////////////////////////////

    ExamplePointCloudProducer::ExamplePointCloudProducer() {
        examplePointCloud = make_shared<PointCloud<Point> >();

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

    void ExamplePointCloudProducer::registerConsumer(shared_ptr<PointCloudConsumer> consumer) {
        consumer->consumePointCloud(examplePointCloud);
    }

    ////////////////////////////////////////////////////////

    PointCloudVisualizer::PointCloudVisualizer() {
        // Basic Visualizer setup
        viewer = make_shared<PCLVisualizer>("TODO: Window Name");
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        // Fill with empty cloud
        PointCloud<Point>::Ptr emptyCloud (new PointCloud<Point>);
        PointCloudColorHandlerRGBField<Point> rgb(emptyCloud);
        viewer->addPointCloud (emptyCloud, rgb);
        viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3);

        // Call onInit
        onInit();
    }

    void PointCloudVisualizer::consumePointCloud(PointCloud<Point>::ConstPtr cloud) {
        onUpdate(cloud);
    }

    bool PointCloudVisualizer::spinOnce() {
        if (viewer->wasStopped ()) {
            return false;
        }

        viewer->spinOnce (100);
        return true;
    }

    void PointCloudVisualizer::onInit() {}

    void PointCloudVisualizer::onUpdate(PointCloud<Point>::ConstPtr cloud) {
        viewer->updatePointCloud(cloud);
    }

}
