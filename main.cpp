#include <iostream>
#include "raspiserial.h"
#include <string>

int main()
{
    RaspiSerial rs(115200);
    std::string ts1("TEST 1 \n");
    std::string ts2("TEST 2 \n");
    std::string ts3("TEST 3 \n");

    uint8_t test_arr[3];
    test_arr[1] = 0x55;
    rs.add_job( (char*)ts3.data(), ts3.length() );
    rs.add_job( (char*)ts2.data(), ts2.length() );
    rs.add_job( (char*)ts1.data(), ts1.length() );
    rs.add_job( (char*)&test_arr[0], 5);

    std::cout << "Test Serial Component!" << std::endl;

    while(1)
    {

    }
    return 0;
}
