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

    Scene::Ptr Scene::fromPointCloud(PointCloud<Point>::ConstPtr &cloud, ConfigProvider::Ptr config) {
        // trim to reachable area
        double minX = config->getDouble("preprocessing.minX");
        double maxX = config->getDouble("preprocessing.maxX");
        double minY = config->getDouble("preprocessing.minY");
        double maxY = config->getDouble("preprocessing.maxY");
        double minZ = config->getDouble("preprocessing.minZ");
        double maxZ = config->getDouble("preprocessing.maxZ");

        // TODO: do this more effienct and include flags for activate/deactivate filtering
        PointCloud<Point>::Ptr cloudIn (new PointCloud<Point>(*cloud));
        PointCloud<Point>::Ptr cloudOut(new PointCloud<Point>());
        PassThrough<Point> filterReachable;
        filterReachable.setInputCloud(cloudIn);
        filterReachable.setFilterFieldName("x");
        filterReachable.setFilterLimits(minX, maxX);
        filterReachable.filter(*cloudOut);

        cloudIn = cloudOut;
        filterReachable.setInputCloud(cloudIn);
        filterReachable.setFilterFieldName("y");
        filterReachable.setFilterLimits(minY, maxY);
        filterReachable.filter(*cloudOut);

        cloudIn = cloudOut;
        filterReachable.setInputCloud(cloudIn);
        filterReachable.setFilterFieldName("z");
        filterReachable.setFilterLimits(minZ, maxZ);
        filterReachable.filter(*cloudOut);

        Scene::Ptr scene(new Scene());
        scene->cloud = cloudOut;
        scene->downsampledCloud = make_shared< PointCloud<Point> >(*(scene->cloud));
        scene->normals = make_shared< PointCloud<Normal> >();


        // Downsample data
        //PointCloud<Point>::Ptr cloudDownsampled(new PointCloud<Point>(*cloudOut));

        /* TODO: https://github.com/PointCloudLibrary/pcl/issues/371
        double downsamplingResolution = config->getDouble("preprocessing.downsamplingResolution");
        PointCloud<Point>::Ptr cloudDownsampled(new PointCloud<Point>());
        VoxelGrid<Point> downsampler;
        downsampler.setInputCloud(cloud);
        downsampler.setLeafSize(downsamplingResolution, downsamplingResolution, downsamplingResolution);
        downsampler.filter(*cloudDownsampled);
        */

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
