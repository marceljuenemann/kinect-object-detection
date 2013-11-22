#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "core.hpp"

namespace libobjdetect {

    /**
     * @brief The Scene class represents a scene grabbed from the Kinect. It holds the PointCloud as well as additional objects like the normals.
     * Furthermore, the factory method is doing preprocessing, e.g. downsampling and croping.
     */
    class Scene {
    public:
        typedef boost::shared_ptr<Scene> Ptr;

        pcl::PointCloud<Point>::ConstPtr getFullPointCloud() { return cloud; }
        pcl::PointCloud<Point>::ConstPtr getDownsampledPointCloud() { return downsampledCloud; }
        pcl::PointCloud<pcl::Normal>::ConstPtr getNormals() { return normals; }

        static Scene::Ptr fromPointCloud(pcl::PointCloud<Point>::ConstPtr &cloud, ConfigProvider::Ptr config);

    private:
        pcl::PointCloud<Point>::ConstPtr cloud;
        pcl::PointCloud<Point>::Ptr downsampledCloud;
        pcl::PointCloud<pcl::Normal>::Ptr normals;

        Scene(){}
    };

    class Table {
    public:
        typedef boost::shared_ptr<Table> Ptr;
        typedef boost::shared_ptr<std::vector<Table::Ptr> > Collection;
        
        pcl::PointCloud<Point>::ConstPtr getConvexHull() { return convexHull; }
        Point getMinDimensions() { return minDimensions; }
        Point getMaxDimensions() { return maxDimensions; }
        double getWidth() { return maxDimensions.x - minDimensions.x; }
        double getDepth() { return maxDimensions.z - minDimensions.z; }

        static Table::Ptr fromConvexHull(pcl::PointCloud<Point>::ConstPtr hull);

    private:
        pcl::PointCloud<Point>::ConstPtr convexHull;
        Point minDimensions;
        Point maxDimensions;
        
        Table(){}
    };

    class TableDetector {
    public:
        TableDetector(ConfigProvider::Ptr config) : config(config) {}
        Table::Collection detectTables(Scene::Ptr scene);
        
    private:
        ConfigProvider::Ptr config;
    };

}

#endif // DETECTION_HPP
