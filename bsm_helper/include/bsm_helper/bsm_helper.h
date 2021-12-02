#include <string>
#include <vector>
#include <exception>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <cav_msgs/BSMCoreData.h>
#include <boost/optional.hpp>

namespace BSMHelper {

    class BSMHelper
    {
        public:

            /**
             * @brief Converts bsm id from uint8_t vector to hex string
             * @param id The incoming bsm id
             * @return The converted id in hex string
            */
            static std::string bsmIDtoString(std::vector<uint8_t, std::allocator<uint8_t>> id);


    };




}