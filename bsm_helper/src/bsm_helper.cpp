/* Copyright (C) 2021 LEIDOS.

 Licensed under the Apache License, Version 2.0 (the "License"); you may not
 use this file except in compliance with the License. You may obtain a copy of
 the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 License for the specific language governing permissions and limitations under
 the License.
*/
#include <sstream>
#include <ios>
#include "bsm_helper/bsm_helper.h"

namespace BSMHelper{


    std::string BSMHelper::bsmIDtoString(std::vector<uint8_t, std::allocator<uint8_t>> id)
    {
        if(id.size() != 4)
        {
            throw std::invalid_argument("Invalid id value");
        }

        unsigned long id_num = 0;
        for(size_t i = 0; i < id.size(); i++)
        {     
            id_num = (id_num << 8) | id.at(i);
        }


        std::stringstream converted;
        converted << std::hex << id_num;
        std::string id_string(converted.str());

        return id_string;
        
    }

    
    
 






}