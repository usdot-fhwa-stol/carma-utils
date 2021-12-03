#include "bsm_helper/bsm_helper.h"

namespace BSMHelper{


    std::string BSMHelper::bsmIDtoString(std::vector<uint8_t, std::allocator<uint8_t>> id)
    {

        std::string id_test;

        for(size_t i =0; i < id.size();i++)
        {
            id_test.append(std::to_string(id.at(i)));
        }

        std::stringstream int_id(id_test);

        unsigned long id_num;

        int_id >> id_num;


        std::stringstream converted;
        converted << std::hex << id_num;
        std::string id_string(converted.str());

        return id_string;
        
    }

    
    
 






}