#include "bsm_helper/bsm_helper.h"

#include <gtest/gtest.h>

namespace BSMHelper{
    TEST(BSMHelper_Test, bsmIDToString_Test)
    {
        std::vector<u_int8_t> input;
        input.push_back(32);
        input.push_back(54);
        input.push_back(22);
        input.push_back(19);
        std::string output = BSMHelper::bsmIDtoString(input);        
        ASSERT_TRUE(output.size() <= 8);
        ASSERT_EQ(output, "1f08e0b" );

    }

}

