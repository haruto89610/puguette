// test.cpp
// g++ -o pug --std=c++11 main.cpp -lm

#include <iostream>
#include "rtmatrix.h"

using namespace linalg::ostream_overloads;

int main(int argc, const char * argv[]) {

  Orientation o;
  Position    p;
  Coordinate  c;
  RTMatrix rt(o, p, c);
  while (true) {
    o.roll  = 1.0;
    o.pitch = 1.0;
    o.yaw   = 1.0;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    c.x = 1.0;
    c.y = 1.0;
    c.z = 1.0;
    rt.update(o, p, c);
    std::wcout << rt.transform() << std::endl;
  }
  return 0;
}
