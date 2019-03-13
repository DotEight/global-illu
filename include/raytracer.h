#pragma once

#include <algorithm>
#include <memory>
//#include <future>

#include <glm/glm.hpp>

#include "camera.h"
#include "entities.h"
#include "image.h"
#include "octree.h"

using glm::dvec3;

class RayTracer {
  public:
    RayTracer() = delete;
    RayTracer(const Camera& camera, glm::dvec3 light)
        : _camera(camera), _light(light), _image(std::make_shared<Image>(0,0)){};

    void setScene(const Octree* scene) { _scene = scene; }

    void run(int w, int h) {
        // TODO Implement this
        _image = std::make_shared<Image>(w, h);

        dvec3 u = glm::normalize(glm::cross(_camera.forward, _camera.up));
        dvec3 v = glm::normalize(glm::cross(u, _camera.forward));

        // The structure of the for loop should remain for incremental rendering.
        for (int y = 0; y < h && _running; ++y) {
            for (int x = 0; x < w && _running; ++x) {
                // TODO Implement this

                //get normalized device coordinates
                double ndcX = (2 * ((x + 0.5) / w) - 1)*(w/h);
                double ndcY = (1 - 2 * ((y + 0.5) / h));

                dvec3 worldPos = ndcX*u + ndcY*v +_camera.pos - (_camera.forward);
                dvec3 dir = glm::normalize(worldPos - _camera.pos);

                Ray ray = Ray(_camera.pos, dir);
                std::vector<Entity*> objects = _scene->intersect(ray);
                double mind = INFINITY;
                Entity* hitObject;
                dvec3 intersect;
                dvec3 normal;

                dvec3 color = {0,0,0};
                for (Entity* object : objects) {
                    if(object->intersect(ray, intersect, normal)) {
                        // There is an intersection
                        double t = glm::length(_camera.pos-intersect);
                        // Get closest intersected object
                        if(t < mind) {
                            mind = t;
                            hitObject = object;
                        }

                        // Light direction (point light)
                        dvec3 L = glm::normalize(_light-intersect);
                        // Diffuse component
                        dvec3 diffuse = {0,0,0};
                        // Specular component
                        dvec3 specular = {0,0,0};

                        // Calculate light according to the material
                        switch (hitObject->material.type) {
                            case Material::Diffuse:
                                // Lambertian surface
                                diffuse = hitObject->material.albedo * 4.0 * std::max(0.0, glm::dot(normal,L)); //4 is light intensity
                                break;
                            case Material::Specular:
                                // If the material is specular add specular component
                                diffuse = hitObject->material.albedo * 4.0 * std::max(0.0, glm::dot(normal,L));
                                // Calculate perfect reflection according
                                dvec3 reflect = L - 2 * glm::dot(L, normal) * normal;
                                double specularComp = 4.0 * std::pow(std::max(0.0, glm::dot(reflect, ray.dir)), 10);
                                specular={specularComp,specularComp,specularComp};
                                break;
                        }

                        color =glm::clamp(hitObject->material.color*(diffuse*hitObject->material.Kd + specular*hitObject->material.Ks),0.0,1.0);

                        bool shadow = false;
                        Ray secondaryRay = Ray(intersect+(normal*0.001), glm::normalize(_light-intersect));
                        for (Entity* object : objects) {
                            if(object->intersect(secondaryRay, intersect, normal))
                                color = {0,0,0};
                        }
                    }
                }
                //dvec3 color = trace(x,y,objects,ray);
                _image->setPixel(x, y, color);
            }
        }
    }

    bool running() const { return _running; }
    void stop() { _running = false; }
    void start() { _running = true; }

    std::shared_ptr<Image> getImage() const { return _image; }

  private:

    dvec3 trace(int x, int y, std::vector<Entity*> objects, Ray& ray) {

        double mind = INFINITY;
        Entity* hitObject;
        dvec3 intersect;
        dvec3 normal;

        for (Entity* object : objects) {
            if(object->intersect(ray, intersect, normal)) {
                double t = glm::length(_camera.pos-intersect);
                if(t < mind) {
                    mind = t;
                    hitObject = object;
                }

                bool shadow = false;
                Ray secondaryRay = Ray(intersect+(normal*0.001), glm::normalize(_light-intersect));
                for (Entity* object : objects) {
                    if(object->intersect(secondaryRay, intersect, normal))
                        shadow = true;
                }

                dvec3 L = glm::normalize(_light-dvec3(0,0,0));
                dvec3 diffuse = {0,0,0};
                dvec3 specular = {0,0,0};

                switch (hitObject->material.type) {

                    case Material::Diffuse:
                        diffuse = hitObject->material.albedo * 4.0 * std::max(0.0, glm::dot(normal,L));
                        break;
                    case Material::Specular:

                        diffuse = hitObject->material.albedo * 4.0 * std::max(0.0, glm::dot(normal,L));
                        dvec3 reflect = L - 2 * glm::dot(L, normal) * normal;
                        double specularComp = 4.0 * std::pow(std::max(0.0, glm::dot(reflect, ray.dir)), 10);
                        specular={specularComp,specularComp,specularComp};
                        break;
                }

                dvec3 pixColor =(double)!shadow*glm::clamp(hitObject->material.color*(diffuse*hitObject->material.Kd + specular*hitObject->material.Ks),0.0,1.0);
                return pixColor;

            } else {
                return {1,1,1};
            }
        }
    }
    bool _running = false;
    const Octree* _scene;
    Camera _camera;
    glm::dvec3 _light;
    std::shared_ptr<Image> _image;
};
