#include "detection.hpp"

#include <boost/make_shared.hpp>

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

using namespace boost;
using namespace pcl;

namespace libobjdetect {

    PointCloud<Point>::ConstPtr Scene::getFullPointCloud() {
        return cloud;
    }

    PointCloud<Point>::ConstPtr Scene::getDownsampledPointCloud() {
        return downsampledCloud;
    }

    PointCloud<pcl::Normal>::ConstPtr Scene::getNormals() {
        return normals;
    }

    Scene::Ptr Scene::fromPointCloud(PointCloud<Point>::ConstPtr &originalCloud, ConfigProvider::Ptr config) {
        // Build the result object
        Scene::Ptr scene(new Scene());
        scene->cloud = originalCloud;
        scene->downsampledCloud = make_shared< PointCloud<Point> >();
        scene->normals = make_shared< PointCloud<Normal> >();

        // Crop the point cloud
        PassThrough<Point> cropFilter;
        PointCloud<Point>::Ptr cropCloud(new PointCloud<Point>());
        std::string keyPrefix("preprocessing.");
        for (char direction = 'z'; direction >= 'x'; --direction) {
            if (!config->getBoolean(keyPrefix + direction + "_filter")) {
                continue;
            }

            double min = config->getDouble(keyPrefix + direction + "_min");
            double max = config->getDouble(keyPrefix + direction + "_max");
            cropFilter.setInputCloud(scene->cloud);
            cropFilter.setFilterFieldName(std::string("") + direction);
            cropFilter.setFilterLimits(min, max);
            cropFilter.filter(*cropCloud);
            scene->cloud = cropCloud;
        }

        // Downsample data
        double downsamplingResolution = config->getDouble("preprocessing.downsamplingResolution");
        VoxelGrid<Point> downsampler;
        downsampler.setInputCloud(scene->cloud);
        downsampler.setLeafSize(downsamplingResolution, downsamplingResolution, downsamplingResolution);
        downsampler.filter(*(scene->downsampledCloud));

        // calculate surface normals
        NormalEstimation<Point, Normal> normalEstimator;
        search::KdTree<Point>::Ptr treeNormals(new search::KdTree<Point>());
        normalEstimator.setInputCloud(scene->downsampledCloud);
        normalEstimator.setSearchMethod(treeNormals);
        normalEstimator.setKSearch(10);
        normalEstimator.compute(*(scene->normals));

        return scene;
    }

    ////////////////////////////////////////////////////////

}
