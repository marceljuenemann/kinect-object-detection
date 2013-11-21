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

        pcl::PointCloud<Point>::ConstPtr getFullPointCloud();
        pcl::PointCloud<Point>::ConstPtr getDownsampledPointCloud();
        pcl::PointCloud<pcl::Normal>::ConstPtr getNormals();

        static Scene::Ptr fromPointCloud(pcl::PointCloud<Point>::ConstPtr &cloud, ConfigProvider::Ptr config);

    private:
        pcl::PointCloud<Point>::Ptr cloud;
        pcl::PointCloud<Point>::Ptr downsampledCloud;
        pcl::PointCloud<pcl::Normal>::Ptr normals;

        Scene();
    };

}

#endif // DETECTION_HPP
