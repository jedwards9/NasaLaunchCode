#include <WString.h>
#include <math.h>

class Vector3{
  public:
    float x;
    float y;
    float z;

  Vector3(float x_ = 0, float y_ = 0, float z_ = 0){
    x = x_;
    y = y_;
    z = z_;
  }

  float mag(){
    return sqrt(x*x+y*y+z*z);
  }

  Vector3 normalized(){
    float magnitude = mag();
    return *(new Vector3(x/magnitude, y/magnitude, z/magnitude));
  }

  void normalize(){
    Vector3 norm = normalized();
    x = norm.x;
    y = norm.y;
    z = norm.z;
  }

  String print(){
    return "X: " + String(x) + " Y: " + String(y) + " Z: " + String(z);
  }
};