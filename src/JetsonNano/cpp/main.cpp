// test.cpp
// g++ -o pug --std=c++11 main.cpp -lm

#include <iostream>
#include "rtmatrix.hpp"

using namespace linalg::ostream_overloads;

int main(int argc, const char * argv[]) {

  Position    position;
  Orientation orientation;

  {
    RTMatrix m{ orientation, position };
    std::wcout << m.rotate() << std::endl;
  }

  {
    orientation.roll = 1.0;
    RTMatrix m{ orientation, position };
    std::wcout << m.X() << std::endl;
  }

  return 0;
}
