#include "core.hpp"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/thread.hpp>

#define CONFIG_UPDATE_INTERVAL 500

using namespace boost;

namespace libobjdetect {

    IniFileConfigProvider::IniFileConfigProvider(const std::string &filename)
        : filename(filename)
    {
        loadProperties();
    }

    void IniFileConfigProvider::loadProperties(){
        read_ini(filename, properties);
        lastUpdate = posix_time::second_clock::local_time();
    }

    template<class Type>
    Type IniFileConfigProvider::getConfig(const std::string &path) {
        // TODO: Don't lock on read
        mutex::scoped_lock lock(mtx);

        posix_time::time_duration diff = posix_time::second_clock::local_time() - lastUpdate;
        if (diff.total_milliseconds() > CONFIG_UPDATE_INTERVAL) {
            loadProperties();
        }

        return properties.get<Type>(path);
    }

    ////////////////////////////////////////////////////////

    int IniFileConfigProvider::getInt(const std::string &path) {
        return getConfig<int>(path);
    }

    double IniFileConfigProvider::getDouble(const std::string &path) {
        return getConfig<double>(path);
    }

    std::string IniFileConfigProvider::getString(const std::string &path) {
        return getConfig<std::string>(path);
    }

    ////////////////////////////////////////////////////////

}
