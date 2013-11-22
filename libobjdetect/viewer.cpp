#include "viewer.hpp"
#include <pcl/visualization/boost.h>

using namespace boost;
using namespace pcl;
using namespace pcl::visualization;

namespace libobjdetect {
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
        // position viewport 3m behind kinect, but look around the point 2m in front of it
        visualizer.setCameraPosition(0., 0., -3., 0., 0., 2., 0., -1., 0.);
        visualizer.setCameraClipDistances(1.0, 10.0);
        visualizer.setBackgroundColor(0.3, 0.3, 0.8);
    }

    void PointCloudViewer::onUpdate(PCLVisualizer& visualizer) {
    }
    
    ////////////////////////////////////////////////////////

    void ObjectDetectionViewer::onPointCloudReceived(PointCloud<Point>::ConstPtr cloud) {
        posix_time::ptime startTime = posix_time::microsec_clock::local_time();

        Scene::Ptr scene = Scene::fromPointCloud(cloud, config);
        Table::Collection detectedTables = tableDetector.detectTables(scene);

        {
            mutex::scoped_lock(resultMutex);
            this->detectedTables = detectedTables;
        }

        posix_time::time_duration diff = posix_time::microsec_clock::local_time() - startTime;
        processingTime = diff.total_milliseconds();

        showPointCloud(scene->getFullPointCloud());
    }

    void ObjectDetectionViewer::onUpdate(PCLVisualizer& visualizer) {
        Table::Collection tables;
        {
            mutex::scoped_lock(resultMutex);
            tables = this->detectedTables;
        }
            
        int tableId = 0;
        visualizer.removeAllShapes();
        for (std::vector<Table::Ptr>::Iterator table = tables->begin(); table != tables->end(); ++table) {
            visualizer.addPolygon<Point>(foundTableHulls[i], 0, 255, 0, std::string("table") + (tableId++));
        }
       
        // TODO: print processing time in lower left corner (together with FPS)
    }

    ////////////////////////////////////////////////////////
}
