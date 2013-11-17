#ifndef OBJDET_CORE_HPP
#define OBJDET_CORE_HPP

#include <pcl/common/common_headers.h>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/thread/mutex.hpp>

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

    /**
     * @brief Provides a simple key value store for abstracting configurations
     */
    class ConfigProvider {
    public:
        typedef boost::shared_ptr<ConfigProvider> Ptr;

        virtual ~ConfigProvider() {}
        virtual int getInt(const std::string &path) = 0;
        virtual double getDouble(const std::string &path) = 0;
        virtual std::string getString(const std::string &path) = 0;
    };

    /**
     * @brief Load confiurations from an ini file and automatically reloads it regularly
     */
    class IniFileConfigProvider : public ConfigProvider {
    public:
        IniFileConfigProvider(const std::string &filename);
        int getInt(const std::string &path);
        double getDouble(const std::string &path);
        std::string getString(const std::string &path);

    private:
        std::string filename;
        boost::mutex mtx;
        boost::property_tree::ptree properties;
        boost::posix_time::ptime lastUpdate;

        template<class Type>
        Type getConfig(const std::string &path);
        void loadProperties();
    };
}

#endif // OBJDET_CORE_HPP
