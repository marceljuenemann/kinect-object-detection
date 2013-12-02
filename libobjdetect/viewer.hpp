#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "core.hpp"
#include "detection.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace libobjdetect {

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
        virtual void onPointCloudReceived(pcl::PointCloud<Point>::ConstPtr cloud);

        /**
         * @brief show the given cloud. Should only be called from within onPointCloudReceived
         * @param cloud
         * @param cloudname
         */
        void showPointCloud (pcl::PointCloud<Point>::ConstPtr cloud, const std::string &cloudname = "cloud");

        /**
         * @brief Triggered when showPointCloud was called for the first time. Do intial setting on the visualizer here
         * @param visualizer
         */
        virtual void onInit(pcl::visualization::PCLVisualizer& visualizer);

        /**
         * @brief Called after each call to showPointCloud and intended for manipulating the visualizer
         * @param visualizer
         */
        virtual void onUpdate(pcl::visualization::PCLVisualizer& visualizer);

    public:

        /**
         * @brief returns whether the visualizer was stopped. Yields the thread, so it can be used in a loop without sleeping
         * @return
         */
        bool wasStopped();

        PointCloudViewer();
        void consumePointCloud(pcl::PointCloud<Point>::ConstPtr cloud);
        void _cb_init(pcl::visualization::PCLVisualizer& visualizer);
        void _cb_update(pcl::visualization::PCLVisualizer& visualizer);

    private:
        boost::shared_ptr<pcl::visualization::CloudViewer> viewer;
        bool hasCloud;
    };


    class ObjectDetectionViewer : public PointCloudViewer {
    public:
        ObjectDetectionViewer(ConfigProvider::Ptr config) 
            : config(config), tableDetector(config), objectDetector(config)
            {}

    protected:
        ConfigProvider::Ptr config;

        void onPointCloudReceived(pcl::PointCloud<Point>::ConstPtr cloud);
        void onUpdate(pcl::visualization::PCLVisualizer& visualizer);
        
    private:
        typedef PointCloudViewer super;

        TableDetector tableDetector;
        ObjectDetector objectDetector;

        boost::mutex resultMutex;
        Table::Collection detectedTables;
        Object::Collection detectedObjects;
        int processingTime;
    };
}

#endif // VIEWER_HPP
