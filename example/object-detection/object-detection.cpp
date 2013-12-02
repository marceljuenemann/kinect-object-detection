#include <libobjdetect/core.hpp>
#include <libobjdetect/io.hpp>
#include <libobjdetect/viewer.hpp>
#include <libobjdetect/detection.hpp>

#include <boost/make_shared.hpp>
#include <pcl/console/parse.h>

// todo: remove unnecessary headers
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

using namespace libobjdetect;
using namespace boost;

int main (int argc, char** argv) {
    ConfigProvider::Ptr config(new IniFileConfigProvider("config.ini"));
    shared_ptr<PointCloudViewer> viewer(new ObjectDetectionViewer(config));
    shared_ptr<PointCloudProducer> producer(new KinectPointCloudProducer);

    producer->registerConsumer(viewer);
    while (!viewer->wasStopped()) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    producer->stop();

    return 0;
}
