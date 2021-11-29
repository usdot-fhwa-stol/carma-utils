#include "BSMHelper.h"

namespace ros{


    std::string BSMHelper::bsmIDtoString(std::vector<uint8_t, std::allocator<uint8_t>> id)
    {
        std::string id_string;

        for(size_t i = 0; i < id.size(); i++)
        {
            id_string.append(decTohex(id[i]));
        }

        return id_string;
        
    }

    std::string BSMHelper::decTohex(int n)
    {
        // String to store hexadecimal number
        std::string hexaDeciNum;
 
        // counter for hexadecimal number array
        int i = 0;
        while (n != 0) 
        {
            // temporary variable to store remainder
            int temp = 0;
 
            // storing remainder in temp variable.
            temp = n % 16;
 
            // check if temp < 10
            if (temp < 10) 
            {
                hexaDeciNum[i] = temp + 48;
                i++;
            }
            else 
            {
                hexaDeciNum[i] = temp + 55;
                i++;
            }
 
            n = n / 16;
        }

        return hexaDeciNum;


    }
 











}