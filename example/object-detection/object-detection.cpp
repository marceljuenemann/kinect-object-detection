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
using namespace pcl;
using namespace pcl::visualization;

class ObjectDetectionViewer : public PointCloudViewer {
private:
    typedef PointCloudViewer super;

protected:
    ConfigProvider::Ptr config;
    std::vector<PointCloud<Point>::Ptr> tmpFoundTableHulls;

    virtual void onPointCloudReceived(PointCloud<Point>::ConstPtr cloud) {

        posix_time::ptime startTime = posix_time::microsec_clock::local_time();


        Scene::Ptr scene = Scene::fromPointCloud(cloud, config);
        PointCloud<Normal>::ConstPtr normals = scene->getNormals();
        PointCloud<Point>::ConstPtr cloudDownsampled = scene->getDownsampledPointCloud();


        // filter points by their normals
        double tableDetectionMaxAngle = config->getDouble("tableDetection.maxAngle");
        PointCloud<Point>::Ptr cloudTableCandidates (new PointCloud<Point>);
        float maxAngle = cos (M_PI * tableDetectionMaxAngle / 180.0);
        for (int i=0; i<normals->points.size(); ++i){
            if (normals->points[i].normal_y >= maxAngle || normals->points[i].normal_y <= -maxAngle){
                cloudTableCandidates->points.push_back(cloudDownsampled->points[i]);
            }
        }

        int minPoints = config->getInt("tableDetection.minPoints");
        if (cloudTableCandidates->points.size() > minPoints){
            // calculate table clusters
            std::vector<PointIndices> indicesVecTableClusters;
            EuclideanClusterExtraction<Point> clusterTables;
            search::KdTree<Point>::Ptr treeTables(new search::KdTree<Point>);
            treeTables->setInputCloud(cloudTableCandidates);
            clusterTables.setInputCloud(cloudTableCandidates);
            clusterTables.setSearchMethod(treeTables);
            clusterTables.setMinClusterSize(minPoints);
            clusterTables.setClusterTolerance(config->getDouble("tableDetection.tolerance"));
            clusterTables.extract(indicesVecTableClusters);

            // prepare segmentation (=find a plane model)
            SACSegmentation<Point> segmentation;
            segmentation.setModelType(SACMODEL_PLANE);
            segmentation.setMethodType(SAC_RANSAC);
            segmentation.setProbability(0.99);
            segmentation.setMaxIterations(config->getInt("tableDetection.maxIterations"));
            segmentation.setDistanceThreshold(config->getDouble("tableDetection.threshold"));
            segmentation.setOptimizeCoefficients(false);


            std::vector<PointCloud<Point>::Ptr> foundTableHulls;

            double minWidth = config->getDouble("tableDetection.minWidth");

            // for each table cluster...
            for (int t=0; t<indicesVecTableClusters.size(); ++t){
                // execute segmentation
                ModelCoefficients::Ptr planeTable(new ModelCoefficients());
                PointIndices::Ptr indicesTable(new PointIndices());
                segmentation.setInputCloud(cloudTableCandidates);
                segmentation.setIndices(make_shared<PointIndices>(indicesVecTableClusters[t]));
                segmentation.segment(*indicesTable, *planeTable);
                if (indicesTable->indices.size() < minPoints) continue;

                // project the table points on the plane model
                PointCloud<Point>::Ptr cloudTable(new PointCloud<Point>());
                ProjectInliers<Point> projectionTable;
                projectionTable.setInputCloud(cloudTableCandidates);
                projectionTable.setIndices(indicesTable);
                projectionTable.setModelCoefficients(planeTable);
                projectionTable.setModelType(SACMODEL_PLANE);
                projectionTable.filter(*cloudTable);

                // calculate a (2-dimensional) convex hull
                PointCloud<Point>::Ptr hullTable(new PointCloud<Point>());
                ConvexHull<Point> convexHullCalculator;
                convexHullCalculator.setInputCloud(cloudTable);
                convexHullCalculator.reconstruct(*hullTable);

                // check for minimal width
                Point minDimensions;
                Point maxDimensions;
                getMinMax3D(*hullTable, minDimensions, maxDimensions);
                if (maxDimensions.x - minDimensions.x < minWidth) continue;
                if (maxDimensions.z - minDimensions.z < minWidth) continue;


                foundTableHulls.push_back(hullTable);
            }
            tmpFoundTableHulls = foundTableHulls;
        }

        // TODO: print this info in the corner of the window
        posix_time::time_duration diff = posix_time::microsec_clock::local_time() - startTime;
        std::cout << "Cloud processing took " << diff.total_milliseconds() << "ms\n";
        std::cout.flush();

        showPointCloud(scene->getFullPointCloud());
    }

    virtual void onInit(PCLVisualizer& visualizer) {
        super::onInit(visualizer);
    }

    virtual void onUpdate(PCLVisualizer& visualizer) {
        visualizer.removeAllShapes();

        std::vector<PointCloud<Point>::Ptr> foundTableHulls = tmpFoundTableHulls;
        for (int i = 0; i < foundTableHulls.size(); ++i) {
            std::string tableId("table");
            tableId += i;
            //pcl::PointCloud<Point>::ConstPtr cloud(new pcl::PointCloud<Point>(*(foundTableHulls[i])));
            visualizer.addPolygon<Point>(foundTableHulls[i], 0, 255, 0, tableId);
       }
    }


public:
    ObjectDetectionViewer(ConfigProvider::Ptr config) : config(config) {}
};


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
