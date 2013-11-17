#ifndef OBJDET_IO_HPP
#define OBJDET_IO_HPP

#include "core.hpp"
#include <pcl/io/openni_grabber.h>

namespace libobjdetect {

    using namespace boost;
    using namespace pcl;

    class KinectPointCloudProducer : public PointCloudProducer {
    public:
        KinectPointCloudProducer();
        void grabberCallback(const PointCloud<Point>::ConstPtr &cloud);
        void registerConsumer(shared_ptr<PointCloudConsumer> consumer);
        void stop();

    private:
        shared_ptr<Grabber> grabber;
        std::vector<shared_ptr<PointCloudConsumer> > consumers;
    };

}

#endif // OBJDET_IO_HPP
