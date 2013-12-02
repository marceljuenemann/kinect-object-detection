#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "core.hpp"

#include <pcl/ModelCoefficients.h>

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
        pcl::ModelCoefficientsConstPtr getModelCoefficients() { return modelCoefficients; }
        Point getMinDimensions() { return minDimensions; }
        Point getMaxDimensions() { return maxDimensions; }
        double getWidth() { return maxDimensions.x - minDimensions.x; }
        double getDepth() { return maxDimensions.z - minDimensions.z; }

        static Table::Ptr fromConvexHull(pcl::PointCloud<Point>::ConstPtr hull, pcl::ModelCoefficients::ConstPtr modelCoefficients);

    private:
        pcl::PointCloud<Point>::ConstPtr convexHull;
        pcl::ModelCoefficients::ConstPtr modelCoefficients;
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

    class Object {
    public:
        typedef boost::shared_ptr<Object> Ptr;
        typedef boost::shared_ptr<std::vector<Object::Ptr> > Collection;

        pcl::PointCloud<Point>::ConstPtr getPointCloud() { return pointCloud; }
        pcl::PointCloud<Point>::ConstPtr getBaseConvexHull() { return baseConvexHull; }
        Point getMinDimensions() { return minDimensions; }
        Point getMaxDimensions() { return maxDimensions; }
        double getWidth() { return maxDimensions.x - minDimensions.x; }
        double getHeight() { return maxDimensions.y - minDimensions.y; }
        int getPointCount() { return getPointCloud()->points.size(); }

        static Object::Ptr create(pcl::PointCloud<Point>::ConstPtr pointCloud, pcl::PointCloud<Point>::ConstPtr baseConvexHull);

    private:
        pcl::PointCloud<Point>::ConstPtr pointCloud;
        pcl::PointCloud<Point>::ConstPtr baseConvexHull;
        Point minDimensions;
        Point maxDimensions;

        Object(){}
    };

    class ObjectDetector {
    public:
        ObjectDetector(ConfigProvider::Ptr config) : config(config) {}
        Object::Collection detectObjects(Scene::Ptr scene, Table::Collection tables);

    private:
        ConfigProvider::Ptr config;
    };

}

#endif // DETECTION_HPP
