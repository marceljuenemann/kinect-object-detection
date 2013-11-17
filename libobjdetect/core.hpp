#ifndef OBJDET_CORE_HPP
#define OBJDET_CORE_HPP

#include <pcl/common/common_headers.h>

namespace libobjdetect {

    /**
     * @brief type definitions
     */
    typedef pcl::PointXYZRGBA Point;

    /**
     * @brief Receives PointCloud objects from a producer and processes them
     */
    class PointCloudConsumer {
    public:
        virtual ~PointCloudConsumer() {}
        virtual void consumePointCloud(pcl::PointCloud<Point>::ConstPtr cloud) = 0;
    };

    /**
     * @brief A PointCloudProducer produces PointClouds and sends them to a consumer
     */
    class PointCloudProducer {
    public:
        virtual ~PointCloudProducer() {}
        virtual void registerConsumer(boost::shared_ptr<PointCloudConsumer> consumer) = 0;
        virtual void stop() = 0;
    };

}

#endif // OBJDET_CORE_HPP
