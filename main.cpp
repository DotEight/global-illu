#include <QApplication>

#include <iostream>

#include "camera.h"
#include "gui.h"
#include "material.h"
#include "entities.h"

using glm::vec3;
using glm::dvec3;

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    /* Materials */
    float AIR_REFRACTIVE_INDEX = 1.0;
    float GLASS_REFRACTIVE_INDEX = 1.52;
    float DIFFUSE_SPECULAR_REFLECTION = 0.18;

/* Colors */
    const vec3 SHADOW_COLOR = 0.0f * vec3(1, 1, 1);
    const vec3 VOID_COLOR(0.75, 0.75, 0.75);

    Camera camera({0, 0, 10}, {0,0,0});
    glm::dvec3 light{10, 10, 10};

    RayTracer raytracer(camera, light);

    // Set up scene
    Octree scene({-20, -20, -20}, {20, 20, 20});
    // TODO Add objects to the scene
    Material diffuseBlue(dvec3(0.10, 0.10,1), Material::Diffuse);
    Material diffuseRed(dvec3(1, 0.10,0.10), Material::Specular);
    Material diffuseGreen(dvec3(0.10, 1,1), Material::Diffuse);

    Entity* s1 = new Sphere(dvec3(-2,0,0), 4, diffuseBlue);
    Entity* s2 = new Sphere(dvec3(1,0,0), 2, diffuseRed);
    Entity* s3 = new Sphere(dvec3(4,0,0), 1, diffuseGreen);
    scene.push_back(s1);
    scene.push_back(s2);
    scene.push_back(s3);

    raytracer.setScene(&scene);

    Gui window(500, 500, raytracer);
    window.show();
    return app.exec();
}
