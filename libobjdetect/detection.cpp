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

    Table::Ptr Table::fromConvexHull(pcl::PointCloud<Point>::ConstPtr hull) {
        Table::Ptr table(new Table());
        table->convexHull = hull;
        getMinMax3D(*hull, table->minDimensions, table->maxDimensions);

        return table;
    }
        
    ////////////////////////////////////////////////////////

    Table::Collection TableDetector::detectTables(Scene::Ptr scene) {
        PointCloud<Point>::ConstPtr cloud = scene->getDownsampledPointCloud();
        PointCloud<Normal>::ConstPtr normals = scene->getNormals();
        
        // filter points by their normals
        PointCloud<Point>::Ptr candidatePoints(new PointCloud<Point>);
        double maxAngle = config->getDouble("tableDetection.maxAngle");
        maxAngle = cos(M_PI * maxAngle / 180.0);
        for (int i = 0; i < normals->points.size(); ++i){
            if (normals->points[i].normal_y >= maxAngle || normals->points[i].normal_y <= -maxAngle){
                candidatePoints->points.push_back(cloud->points[i]);
            }
        }

        // not enough candidates?
        Table::Collection foundTables(new std::vector<Table::Ptr>());
        int minPoints = config->getInt("tableDetection.minPoints");
        if (candidatePoints->points.size() < minPoints){
            return foundTables;
        }
        
        // cluster the candidates to tables
        std::vector<PointIndices> tableClusters;
        EuclideanClusterExtraction<Point> clusterExtractor;
        search::KdTree<Point>::Ptr kdtree(new search::KdTree<Point>);
        kdtree->setInputCloud(candidatePoints);
        clusterExtractor.setInputCloud(candidatePoints);
        clusterExtractor.setSearchMethod(kdtree);
        clusterExtractor.setMinClusterSize(minPoints);
        clusterExtractor.setClusterTolerance(config->getDouble("tableDetection.tolerance"));
        clusterExtractor.extract(tableClusters);

        // prepare RANSAC (used for finding a plane model)
        SACSegmentation<Point> segmentation;
        segmentation.setModelType(SACMODEL_PLANE);
        segmentation.setMethodType(SAC_RANSAC);
        segmentation.setProbability(0.99);
        segmentation.setMaxIterations(config->getInt("tableDetection.maxIterations"));
        segmentation.setDistanceThreshold(config->getDouble("tableDetection.threshold"));
        segmentation.setOptimizeCoefficients(false);

        // for each table cluster...
        double minWidth = config->getDouble("tableDetection.minWidth");
        double minDepth = config->getDouble("tableDetection.minDepth");
        for (std::vector<PointIndices>::iterator cluster = tableClusters.begin(); cluster != tableClusters.end(); ++cluster) {
            // find a plane model
            ModelCoefficients::Ptr planeCoefficients(new ModelCoefficients());
            PointIndices::Ptr tableIndices(new PointIndices());
            segmentation.setInputCloud(candidatePoints);
            segmentation.setIndices(make_shared<PointIndices>(*cluster));
            segmentation.segment(*tableIndices, *planeCoefficients);
            if (tableIndices->indices.size() < minPoints) continue;

            // project the table points on the plane model
            PointCloud<Point>::Ptr projectedTable(new PointCloud<Point>());
            ProjectInliers<Point> projector;
            projector.setInputCloud(candidatePoints);
            projector.setIndices(tableIndices);
            projector.setModelCoefficients(planeCoefficients);
            projector.setModelType(SACMODEL_PLANE);
            projector.filter(*projectedTable);

            // calculate a (2-dimensional) convex hull
            PointCloud<Point>::Ptr tableHull(new PointCloud<Point>());
            ConvexHull<Point> convexHullCalculator;
            convexHullCalculator.setInputCloud(projectedTable);
            convexHullCalculator.reconstruct(*tableHull);

            // check for minimal width and depth
            Table::Ptr table = Table::fromConvexHull(tableHull);
            if (table->getWidth() < minWidth) continue;
            if (table->getDepth() < minDepth) continue;
            foundTables->push_back(table);
        }
        
        return foundTables;
    }

}
