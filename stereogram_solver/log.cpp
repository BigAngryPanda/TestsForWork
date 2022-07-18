#include "log.hpp"
#include <iostream>

namespace trace
{
    void stdout(std::string const& msg)
    {
#ifdef DEBUG
        std::cout << msg << std::endl;
#endif
    }
}