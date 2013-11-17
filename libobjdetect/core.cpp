#include "core.hpp"

#include <pcl/visualization/boost.h>
#include <iostream>

namespace libobjdetect {

    ////////////////////////////////////////////////////////

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

    void ExamplePointCloudProducer::registerConsumer(shared_ptr<PointCloudConsumer> consumer) {
        consumer->consumePointCloud(examplePointCloud);
    }

    ////////////////////////////////////////////////////////

    PointCloudViewer::PointCloudViewer()
        : viewer(new CloudViewer("Point Cloud Viewer")),
          hasCloud(false) {}

    ////////////////////////////////////////////////////////

    void PointCloudViewer::consumePointCloud(PointCloud<Point>::ConstPtr cloud) {
        onPointCloudReceived(cloud);
    }

    void PointCloudViewer::_cb_init(PCLVisualizer& visualizer) {
        onInit(visualizer);
    }

    void PointCloudViewer::_cb_update(PCLVisualizer& visualizer) {
        onUpdate(visualizer);
    }

    ////////////////////////////////////////////////////////

    void PointCloudViewer::showPointCloud (PointCloud<Point>::ConstPtr cloud, const std::string &cloudname) {
        viewer->showCloud(cloud, cloudname);

        if (!hasCloud) {
            hasCloud = true;
            viewer->runOnVisualizationThreadOnce(bind(&PointCloudViewer::_cb_init, this, _1));
        }

        viewer->runOnVisualizationThreadOnce(bind(&PointCloudViewer::_cb_update, this, _1));
    }

    bool PointCloudViewer::wasStopped() {
        return viewer->wasStopped();
    }

    ////////////////////////////////////////////////////////

    void PointCloudViewer::onPointCloudReceived(PointCloud<Point>::ConstPtr cloud) {
        showPointCloud(cloud);
    }

    void PointCloudViewer::onInit(PCLVisualizer& visualizer) {
        visualizer.setBackgroundColor(0.3, 0.3, 1.0);
        visualizer.setCameraClipDistances(1.0, 10.0);

        // position viewport 3m behind kinect, but look around the point 2m in front of it
        visualizer.setCameraPosition(0., 0., -3., 0., 0., 2., 0., -1., 0.);
    }

    void PointCloudViewer::onUpdate(PCLVisualizer& visualizer) {
    }
}
