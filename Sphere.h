#ifndef SPHERE
#define SPHERE
#include "Point3.h"
#include "Object.h"
#include "Ray.h"

class Sphere:public Object
{
public:
    Sphere();
	~Sphere();
	Sphere(Point3 point, float radius);
	bool hit(const Ray& ray);

private:
    float r;
	Point3 p;

};
#endif
