#ifndef OBJDET_IO_HPP
#define OBJDET_IO_HPP

#include "core.hpp"
#include <pcl/io/openni_grabber.h>

#include <pcl/io/grabber.h>

namespace libobjdetect {

    class KinectPointCloudProducer : public PointCloudProducer {
    public:
        KinectPointCloudProducer();
        void grabberCallback(const pcl::PointCloud<Point>::ConstPtr &cloud);
        void registerConsumer(boost::shared_ptr<PointCloudConsumer> consumer);
        void stop();

    private:
        boost::shared_ptr<pcl::Grabber> grabber;
        std::vector<boost::shared_ptr<PointCloudConsumer> > consumers;
    };

}

#endif // OBJDET_IO_HPP
