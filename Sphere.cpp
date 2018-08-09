#include "Sphere.h"
#include <math.h>
#include <stdio.h>

Sphere::Sphere()
  : p(0,0,0)
{
    r = 0;
}

Sphere::~Sphere()
{
}

Sphere::Sphere(Point3 point, float radius)
  : p(point)
{
    r = radius;
}

bool Sphere::hit(const Ray& ray)
{
  float t;
  Vector3 temp = ray.o - p;
  float a = ray.d.dot(ray.d);  
  float b = 2.0 * (temp.dot(ray.d));
  float c = temp.dot(temp) - r * r;
  float disc = b * b - 4.0 * a * c; 
  //printf("Sphere check hit: ray = %2.2f, %2.2f %2.2f\n", ray.o.e[0], ray.o.e[1], ray.o.e[3]);
  if(disc < 0){
    return 0;
  }
  else {
	float e = sqrt(disc);
	float denom = 2.0 * a;
	
	t = (-b - e)/denom; // smaller root
	if (t>0)
	  return 1;
	
	t = (-b + e)/denom; // larger root
	if (t>0)
	  return 1;
  }
  
  return 0;
}
