/* Copyright (C) 2021 LEIDOS.

 Licensed under the Apache License, Version 2.0 (the "License"); you may not
 use this file except in compliance with the License. You may obtain a copy of
 the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 License for the specific language governing permissions and limitations under
 the License.
*/

#include "bsm_helper/bsm_helper.h"
#include <gtest/gtest.h>

namespace BSMHelper{
    TEST(BSMHelper_Test, bsmIDToString_Test)
    {
        std::vector<u_int8_t> input;
        input.push_back(240);
        input.push_back(58);
        input.push_back(214);
        input.push_back(16);

        input.push_back(45); //Throw test
        EXPECT_THROW(BSMHelper::bsmIDtoString(input), std::invalid_argument);
        input.pop_back();
        std::string output = BSMHelper::bsmIDtoString(input);        
        ASSERT_TRUE(output.size() <= 8);
        ASSERT_EQ(output, "f03ad610" );

    }

}

