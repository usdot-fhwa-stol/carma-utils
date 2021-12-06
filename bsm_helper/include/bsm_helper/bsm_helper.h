/* Copyright (C) 2021 LEIDOS.

 Licensed under the Apache License, Version 2.0 (the "License"); you may not
 use this file except in compliance with the License. You may obtain a copy of
 the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 License for the specific language governing permissions and limitations under
 the License.
*/

#pragma once
#include <string.h>
#include <ros/ros.h>
#include <vector>
#include <exception>
#include <mutex>
#include <memory>
#include <boost/optional.hpp>

namespace BSMHelper {

    class BSMHelper
    {
        public:

            /**
             * @brief Converts bsm id from uint8_t vector to hex string
             * @param id The incoming four-element uint8_t vector bsm id 
             * @return The converted id in hex string
            */
            static std::string bsmIDtoString(std::vector<uint8_t, std::allocator<uint8_t>> id);


    };




}