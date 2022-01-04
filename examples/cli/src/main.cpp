#include <Cockroach.hpp>
#include <iostream>


int main()
{
    using namespace Cockroach;


    std::string value = greeting();
    std::cout << value << std::endl;

    return 0;
}