#pragma once

#include <glm/glm.hpp>

/// Represents the material properties of an entity. For now it only contains color, but it should
/// probably be extended to allow more options.
struct Material {
    constexpr explicit Material(glm::dvec3 color, int type)
    : color(std::move(color)), type(type) {}

    typedef enum Type {
        Diffuse,
        DiffuseSpecular,
        Specular,
        Glass
    }Type;

    int type;
    glm::dvec3 color;

};