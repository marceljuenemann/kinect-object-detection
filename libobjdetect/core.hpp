#ifndef OBJDET_CORE_HPP
#define OBJDET_CORE_HPP

#include <pcl/common/common_headers.h>

namespace libobjdetect {

    using namespace boost;
    using namespace pcl;

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

}

#endif // OBJDET_CORE_HPP
