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

    Sphere() : Entity(), center(dvec3(0,0,0)), radius(1) {}
    Sphere(glm::dvec3 center, float radius, Material& material)
            : Entity(material), center(center), radius(radius) {}

    glm::dvec3 normal(const glm::dvec3& position) const {
        return glm::normalize((position - center) / radius);
    }

    bool intersect(const Ray &ray, glm::dvec3 &intersect, glm::dvec3 &normal) const override {

        double t0, t1;
        dvec3 dir = ray.dir;
        dvec3 origin = ray.origin;

        dvec3 L = origin - this->center;
        double a = glm::dot(dir, dir);
        double b = 2 * glm::dot(dir, L);
        double c = glm::dot(L, L) - sqrt(this->radius);

        double disc = b*b - 4 * a*c;

        if (disc < 0) return false;
        else if (disc == 0) t0 = t1 = - 0.5 * b / a;
        else {
            double q = (b > 0) ?
                      -0.5 * (b + sqrt(disc)) :
                      -0.5 * (b - sqrt(disc));
            t0 = q / a;
            t1 = c / q;
        }
        if (t0 > t1) std::swap(t0, t1);

        if (t0 < 0) {
            t0 = t1; // if t0 is negative, let's use t1 instead
            if (t0 < 0) return false; // both t0 and t1 are negative
        }

        intersect = ray.origin + t0 * ray.dir;
        normal = this->normal(intersect);

        return true;

    }

    virtual BoundingBox boundingBox() const override {
        BoundingBox aabb = BoundingBox(dvec3(0,0,0),dvec3(1,1,1));
        return aabb;
    };

    glm::dvec3 center;
    double radius;
};
// TODO Implement explicit sphere (triangles)
// TODO Implement explicit quad (triangles)
struct Quad:Entity{};
// TODO Implement explicit cube (triangles)
struct Cube:Entity{};

