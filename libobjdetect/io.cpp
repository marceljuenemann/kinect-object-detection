#include "io.hpp"

#include <boost/make_shared.hpp>
#include <pcl/io/openni_grabber.h>

using namespace boost;
using namespace pcl;

namespace libobjdetect {

    ////////////////////////////////////////////////////////

    KinectPointCloudProducer::KinectPointCloudProducer() {
        grabber = make_shared<OpenNIGrabber>();
        function<void (const PointCloud<Point>::ConstPtr&)> callback =
            bind(&KinectPointCloudProducer::grabberCallback, this, _1);
        grabber->registerCallback(callback);
        grabber->start();
    }

    void KinectPointCloudProducer::stop() {
        grabber->stop();
        consumers.clear();
    }

    void KinectPointCloudProducer::registerConsumer(shared_ptr<PointCloudConsumer> consumer) {
        consumers.push_back(consumer);
    }

    void KinectPointCloudProducer::grabberCallback(const PointCloud<Point>::ConstPtr &cloud) {
        for (std::vector<shared_ptr<PointCloudConsumer> >::iterator it = consumers.begin(); it != consumers.end(); ++it) {
            (*it)->consumePointCloud(cloud);
        }
    }

    ////////////////////////////////////////////////////////

}
