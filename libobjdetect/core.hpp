#ifndef CORE_HPP
#define CORE_HPP

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace libobjdetect {

    using namespace boost;
    using namespace pcl;
    using namespace pcl::visualization;

    /**
     * @brief type definitions
     */
    typedef PointXYZRGBA Point;

    /**
     * @brief Receives PointCloud objects from a producer and processes them
     */
    class PointCloudConsumer {
    public:
        virtual ~PointCloudConsumer() {}
        virtual void consumePointCloud(PointCloud<Point>::ConstPtr cloud) = 0;
    };

    /**
     * @brief A PointCloudProducer produces PointClouds and sends them to a consumer
     */
    class PointCloudProducer {
    public:
        virtual ~PointCloudProducer() {}
        virtual void registerConsumer(shared_ptr<PointCloudConsumer> consumer) = 0;
    };

    /**
     * @brief The ExamplePointCloudProducer class send each consumer an example point cloud once
     */
    class ExamplePointCloudProducer : public PointCloudProducer {
    public:
        ExamplePointCloudProducer();
        void registerConsumer(shared_ptr<PointCloudConsumer> consumer);

    private:
        PointCloud<Point>::Ptr examplePointCloud;
    };

    /**
     * @brief The PointCloudVisualizer class displays a PointCloud in a simple window using a PCLVisualizer.
     * Override the onInit and onUpdate methods to extend the functionality
     */
    class PointCloudVisualizer : public PointCloudConsumer {
    public:
        PointCloudVisualizer();
        void consumePointCloud(PointCloud<Point>::ConstPtr cloud);

        bool spinOnce();

    protected:
        shared_ptr<PCLVisualizer> viewer;

        virtual void onInit();

        virtual void onUpdate(PointCloud<Point>::ConstPtr cloud);

    };

}

#endif // CORE_HPP
