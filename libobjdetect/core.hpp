#ifndef OBJDET_CORE_HPP
#define OBJDET_CORE_HPP

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

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
        virtual void stop() = 0;
    };

    /**
     * @brief The ExamplePointCloudProducer class send each consumer an example point cloud once
     */
    class ExamplePointCloudProducer : public PointCloudProducer {
    public:
        ExamplePointCloudProducer();
        void registerConsumer(shared_ptr<PointCloudConsumer> consumer);
        void stop() {}

    private:
        PointCloud<Point>::Ptr examplePointCloud;
    };

    /**
     * @brief The PointCloudVisualizer class displays a PointCloud in a simple window using a PCLVisualizer.
     * Override the protect event listeners to extend the functionality or add processing.
     */
    class PointCloudViewer : public PointCloudConsumer {
    protected:

        /**
         * @brief Called when the producer sends a new point cloud. Do your cloud processing in here and call showPointCloud
         * @param visualizer
         */
        virtual void onPointCloudReceived(PointCloud<Point>::ConstPtr cloud);

        /**
         * @brief show the given cloud. Should only be called from within onPointCloudReceived
         * @param cloud
         * @param cloudname
         */
        void showPointCloud (PointCloud<Point>::ConstPtr cloud, const std::string &cloudname = "cloud");

        /**
         * @brief Triggered when showPointCloud was called for the first time. Do intial setting on the visualizer here
         * @param visualizer
         */
        virtual void onInit(PCLVisualizer& visualizer);

        /**
         * @brief Called after each call to showPointCloud and intended for manipulating the visualizer
         * @param visualizer
         */
        virtual void onUpdate(PCLVisualizer& visualizer);

    public:

        /**
         * @brief returns whether the visualizer was stopped. Yields the thread, so it can be used in a loop without sleeping
         * @return
         */
        bool wasStopped();

        PointCloudViewer();
        void consumePointCloud(PointCloud<Point>::ConstPtr cloud);
        void _cb_init(PCLVisualizer& visualizer);
        void _cb_update(PCLVisualizer& visualizer);

    private:
        shared_ptr<CloudViewer> viewer;
        bool hasCloud;
    };

}

#endif // OBJDET_CORE_HPP
