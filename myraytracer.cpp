#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

template<typename T>
class Vec3
{
public:
    T x, y, z;
    Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
    Vec3(T xx) : x(xx), y(xx), z(xx) {}
    Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
    Vec3& normalize()
    {
        T nor2 = length2();
        if (nor2 > 0) {
            T invNor = 1 / sqrt(nor2);
            x *= invNor, y *= invNor, z *= invNor;
        }
        return *this;
    }
    Vec3<T> operator * (const T &f) const { return Vec3<T>(x * f, y * f, z * f); }
    Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x * v.x, y * v.y, z * v.z); }
    T dot(const Vec3<T> &v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
    Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
    Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
    Vec3<T>& operator *= (const Vec3<T> &v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
    Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }
    T length2() const { return x * x + y * y + z * z; }
    T length() const { return sqrt(length2()); }
    friend std::ostream & operator << (std::ostream &os, const Vec3<T> &v)
    {
        os << "[" << v.x << " " << v.y << " " << v.z << "]";
        return os;
    }
};

typedef Vec3<float> Vec3f;


class Object
{
public:
    Vec3f surfaceColor, emissionColor;
    float transparency, reflection;

    Object(const Vec3f &sc, const float &refl, const float &transp, const Vec3f &ec) :
        surfaceColor(sc), emissionColor(ec), transparency(transp), reflection(refl) {}
    
    virtual ~Object() {}
    
    // Virtual functions to be overridden by Sphere and Cube
    virtual bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t0, float &t1) const = 0;
    virtual Vec3f getNormal(const Vec3f &phit) const = 0;
    virtual Vec3f getCenter() const = 0;
};


class Sphere : public Object
{
public:
    Vec3f center;                           /// position of the sphere
    float radius, radius2;                  /// sphere radius and radius^2
    Sphere(
        const Vec3f &c,
        const float &r,
        const Vec3f &sc,
        const float &refl = 0,
        const float &transp = 0,
        const Vec3f &ec = 0) :
        center(c), radius(r), Object(sc, refl, transp, ec), radius2(r * r)
    {}
    bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t0, float &t1) const override
    {
        Vec3f l = center - rayorig;
        float tca = l.dot(raydir); // distance along the ray closest to the sphere center
        if (tca < 0) return false;
        float d2 = l.dot(l) - tca * tca;
        if (d2 > radius2) return false;
        float thc = sqrt(radius2 - d2); // from closest point to center to surface of sphere
        t0 = tca - thc;
        t1 = tca + thc;
        
        return true;
    }

    Vec3f getNormal(const Vec3f &phit) const override
    {
        return (phit - center).normalize();
    }

    Vec3f getCenter() const override { return center; }
};

class Cube : public Object
{
    public:
        Vec3f bounds[2];

        Cube(const Vec3f &bmin, const Vec3f &bmax, const Vec3f &sc, const float &refl = 0, const float &transp = 0, const Vec3f &ec = 0) :
            Object(sc, refl, transp, ec)
        {
            bounds[0] = bmin;
            bounds[1] = bmax;
        }

        // Initialize Axis-Aligned Bounding Box algorithm
        bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t0, float &t1) const override
        {
            Vec3f invdir(1/raydir.x,1/raydir.y,1/raydir.z);
            int sign[3] = {raydir.x < 0, raydir.y < 0, raydir.z < 0};

            float tmin  = (bounds[sign[0]].x - rayorig.x) * invdir.x;
            float tmax  = (bounds[1 - sign[0]].x - rayorig.x) * invdir.x;
            float tymin = (bounds[sign[1]].y - rayorig.y) * invdir.y;
            float tymax = (bounds[1 - sign[1]].y - rayorig.y) * invdir.y;

            if(tmin > tmax || tymin > tymax) return false;
            if(tymin > tmin) tmin = tymin;
            if(tymax < tmax) tmax = tymax;

            float tzmin = (bounds[sign[2]].z - rayorig.z) * invdir.z;
            float tzmax = (bounds[1 - sign[2]].z - rayorig.z) * invdir.z;

            if ((tmin > tzmax) || (tzmin > tmax)) return false;
            if (tzmin > tmin) tmin = tzmin;
            if (tzmax < tmax) tmax = tzmax;

            t0 = tmin;
            t1 = tmax;
            if (t0 < 0 && t1 < 0) return false;
            return true;
        }

        // Determine normal based on which face the hit point is physically touching
        Vec3f getNormal(const Vec3f &phit) const override
        {
            Vec3f normal(0);
            float bias = 1e-4; // Margin of error for floating point math
            
            if (std::abs(phit.x - bounds[0].x) < bias) normal.x = -1;
            else if (std::abs(phit.x - bounds[1].x) < bias) normal.x = 1;
            else if (std::abs(phit.y - bounds[0].y) < bias) normal.y = -1;
            else if (std::abs(phit.y - bounds[1].y) < bias) normal.y = 1;
            else if (std::abs(phit.z - bounds[0].z) < bias) normal.z = -1;
            else if (std::abs(phit.z - bounds[1].z) < bias) normal.z = 1;
            
            return normal;
        }

        Vec3f getCenter() const override { return (bounds[0] + bounds[1]) * 0.5f; }
};

//[comment]
// This variable controls the maximum recursion depth
//[/comment]
#define MAX_RAY_DEPTH 5

float mix(const float &a, const float &b, const float &mix)
{
    return b * mix + a * (1 - mix);
}

Vec3f trace(
    const Vec3f& rayorigin,
    const Vec3f &raydir,
    const std::vector<Object*> &objs,
    const int &depth)
{
    //if(raydir.length() != 1) std::cerr << "Error" << raydir << std::endl;
    const Object* hitobj = NULL;
    float tnear = INFINITY;
    // find intersection of this ray with the sphere in the scene
    for(int i = 0; i < objs.size(); i++){
        float t0 = INFINITY, t1 = INFINITY;
        if(objs[i]->intersect(rayorigin, raydir, t0, t1)){
            if(t0 < 0) t0 = t1;
            if(t0 < tnear){
                tnear = t0;
                hitobj = objs[i];
            }
        }
    }
    // if no intersection return background color or black
    if (!hitobj) return Vec3f(2);
    Vec3f surfacecolor = 0; // to be calculated - color intersecting with ray
    Vec3f phit = rayorigin + raydir * tnear; // point of intersection
    Vec3f nhit = hitobj->getNormal(phit); // normal of phit from center of object
    
    float bias = 1e-4;
    bool inside = false;
    if(raydir.dot(nhit) > 0) inside = true, nhit = -nhit; // normal and view direction are not opposite of each other so we reverse normal
    if ((hitobj->transparency > 0 || hitobj->reflection > 0) && depth < MAX_RAY_DEPTH) {
        float facingratio = -raydir.dot(nhit);
        // change the mix value to tweak the effect
        float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);
        // compute reflection direction (not need to normalize because all vectors
        // are already normalized)
        Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);
        refldir.normalize();
        Vec3f reflection = trace(phit + nhit * bias, refldir, objs, depth + 1);
        Vec3f refraction = 0;
        // if the sphere is also transparent compute refraction ray (transmission)
        if (hitobj->transparency) {
            float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
            float cosi = -nhit.dot(raydir);
            float k = 1 - eta * eta * (1 - cosi * cosi);
            Vec3f refrdir = raydir * eta + nhit * (eta *  cosi - sqrt(k));
            refrdir.normalize();
            refraction = trace(phit - nhit * bias, refrdir, objs, depth + 1);
        }

        surfacecolor = (
            reflection * fresneleffect +
            refraction * (1 - fresneleffect) * hitobj->transparency) * hitobj->surfaceColor;
    }
    else {
        // 4. Handle Matte surfaces and Shadows
        for (unsigned i = 0; i < objs.size(); ++i) {
            if (objs[i]->emissionColor.x > 0) {
                // This is a light source
                Vec3f transmission = 1;
                
                // Use getCenter() to find the light's position
                Vec3f lightDirection = objs[i]->getCenter() - phit;
                lightDirection.normalize();
                
                // Shadow ray
                for (unsigned j = 0; j < objs.size(); ++j) {
                    if (i != j) {
                        float t0, t1;
                        if (objs[j]->intersect(phit + nhit * bias, lightDirection, t0, t1)) {
                            transmission = 0; // Point is in shadow
                            break;
                        }
                    }
                }
                
                surfacecolor += hitobj->surfaceColor * transmission *
                std::max(float(0), nhit.dot(lightDirection)) * objs[i]->emissionColor;
            }
        }
    }

    return surfacecolor + hitobj->emissionColor;
}

void render(const std::vector<Object*>& objs){
    unsigned width = 640, height = 480;
    Vec3f *image = new Vec3f[width * height], *pixel = image;
    float invWidth = 1 / float(width), invHeight = 1 / float(height);
    float fov = 30, aspectratio = width / float(height);
    float angle = tan(M_PI * 0.5 * fov / 180.);
    // Trace rays
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x, ++pixel) {
            float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
            float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;
            Vec3f raydir(xx, yy, -1);
            raydir.normalize();
            *pixel = trace(Vec3f(0), raydir, objs, 0);
        }
    }
    // Save result to a PPM image (keep these flags if you compile under Windows)
    std::ofstream ofs("./myuntitled.ppm", std::ios::out | std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (unsigned i = 0; i < width * height; ++i) {
        ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
               (unsigned char)(std::min(float(1), image[i].y) * 255) <<
               (unsigned char)(std::min(float(1), image[i].z) * 255);
    }
    ofs.close();
    delete [] image;
}

int main(int argc, char** argv){
    srand48(13);

    std::vector<Object*> objects;

    // A large floor sphere
    objects.push_back(new Sphere(Vec3f( 0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
    
    // Add original spheres
    objects.push_back(new Sphere(Vec3f( 0.0,      0, -20),     4, Vec3f(1.00, 0.32, 0.36), 1, 0.5));
    objects.push_back(new Sphere(Vec3f( 5.0,     -1, -15),     2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
    
    // Add a CUBE (Min Bounds, Max Bounds, Color, Reflectivity, Transparency)
    objects.push_back(new Cube(Vec3f(-7.0, -2.0, -22.0), Vec3f(-3.0, 2.0, -18.0), Vec3f(0.20, 0.80, 0.20), 0.5, 0.0));

    // light
    objects.push_back(new Sphere(Vec3f( 0.0,     20, -30),     3, Vec3f(0.00, 0.00, 0.00), 0, 0.0, Vec3f(3)));

    render(objects);

    for (size_t i =0; i < objects.size(); i++) delete objects[i];

    return 0;
}