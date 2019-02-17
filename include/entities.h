#pragma once

#include <glm/glm.hpp>

#include "bbox.h"
#include "material.h"
#include "ray.h"

using glm::vec3;
using glm::dvec3;

/// A base class for all entities in the scene.
struct Entity {

    constexpr Entity() : material(Material(glm::dvec3(1, 0, 0), Material::Diffuse)) {}
    constexpr Entity(const Material& material) : material(material) {}

    /// Check if a ray intersects the object. The arguments intersect and normal will contain the
    /// point of intersection and its normals.
    virtual bool intersect(const Ray& ray, glm::dvec3& intersect, glm::dvec3& normal) const = 0;

    /// Returns an axis-aligned bounding box of the entity.
    virtual BoundingBox boundingBox() const = 0;

    glm::dvec3 pos = {0, 0, 0};
    Material material;
};

// TODO Implement implicit triangle
struct Triangle: public Entity{
    Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, const Material& material)
            : Entity(material), v0(v0), v1(v1), v2(v2)
    {
        glm::vec3 e1 = v1 - v0;
        glm::vec3 e2 = v2 - v0;
        _normal = glm::normalize(glm::cross(e2, e1));
    }

    glm::vec3 normal(const glm::vec3& position) {
        return _normal;
    }

    bool intersect(const Ray& ray, dvec3& intersect, glm::dvec3& normal) {
        float EPSILON = 0.000001;
        dvec3 P, Q, T;
        float det, inv_det, u, v;
        double t;

        // Edges sharing v0
        dvec3 e1 = v1 - v0;
        dvec3 e2 = v2 - v0;
        // Compute determinant
        P = glm::cross(ray.dir, e2);
        det = glm::dot(e1, P);

        if (det > -EPSILON && det < EPSILON) {
            // The ray lies within the plane of the triangle
            return 0;
        }
        inv_det = 1.f / det;

        // Calculate the distance from v0 to the start point
        T = ray.origin - v0;

        // Calculate u parameter and test bound
        u = glm::dot(T, P) * inv_det;
        if (u < 0.f || u > 1.f) {
            // The intersection lies outside of the triangle
            return 0;
        }

        // V parameter
        Q = glm::cross(T, e1);
        v = glm::dot(ray.dir, Q) * inv_det;

        if (v < 0.f || u + v  > 1.f) {
            // The intersection lies outside of the triangle
            return 0;
        }

        t = glm::dot(e2, Q) * inv_det;

        if (t > EPSILON) {
            // Ray intersection
            intersect = ray.origin + t * ray.dir;
            return 1;
        }

        // No intersection
        return 0;
    }

    glm::dvec3 v0;
    glm::dvec3 v1;
    glm::dvec3 v2;
    glm::dvec3 _normal;
};

// TODO Implement implicit sphere
struct Sphere: public Entity{

    Sphere() : Entity(), center(vec3(0,0,0)), radius(1) {}
    Sphere(glm::vec3 center, float radius, Material& material)
            : Entity(material), center(center), radius(radius) {}

    glm::vec3 normal(const glm::vec3& position) {
        return glm::normalize((position - center) / radius);
    }

    bool intersect(const Ray &ray, glm::dvec3 &intersect, glm::dvec3 &normal) const override {

        float dx = ray.dir.x;
        float dy = ray.dir.y;
        float dz = ray.dir.z;

        float a = dx*dx + dy*dy + dz*dz;

        float b = 2 * dx*(ray.origin.x - center.x)
                  + 2 * dy*(ray.origin.y - center.y)
                  + 2 * dz*(ray.origin.z - center.z);

        float c = center.x*center.x
                  + center.y*center.y
                  + center.z*center.z
                  + ray.origin.x*ray.origin.x
                  + ray.origin.y*ray.origin.y
                  + ray.origin.z*ray.origin.z
                  - 2 * (center.x*ray.origin.x + center.y*ray.origin.y + center.z*ray.origin.z)
                  - radius*radius;

        float disc = b*b - 4 * a*c;

        // No intersection
        if (disc < 0) {
            return false;
        }
        // Ray tangent or intersects in two points
        float t = (-b - sqrt(disc)) / (2 * a);
        if (t < 0) {
            return false;
        }

        intersect.x = ray.origin.x + t * dx;
        intersect.y = ray.origin.y + t * dy;
        intersect.z = ray.origin.z + t * dz;

        return true;

    }

    virtual BoundingBox boundingBox() const override {
        BoundingBox aabb = BoundingBox(dvec3(0,0,0),dvec3(1,1,1));
        return aabb;
    };

    glm::vec3 center;
    float radius;
};
// TODO Implement explicit sphere (triangles)
// TODO Implement explicit quad (triangles)
struct Quad:Entity{};
// TODO Implement explicit cube (triangles)
struct Cube:Entity{};

