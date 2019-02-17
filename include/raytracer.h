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

        // The structure of the for loop should remain for incremental rendering.
        for (int y = 0; y < h && _running; ++y) {
            for (int x = 0; x < w && _running; ++x) {
                // TODO Implement this
                float px = (2 * ((x + 0.5) / w) - 1) * tan(45 / 2 * M_PI / 180);
                float py = (1 - 2 * ((y + 0.5) / h)) * tan(45 / 2 * M_PI / 180);
                dvec3 worldPixel =  {-1, py, px};

                dvec3 dir = glm::normalize(worldPixel - _camera.pos);

                Ray ray = Ray(_camera.pos, dir);

                std::vector<Entity*> objects = _scene->intersect(ray);
                for (Entity* object : objects) {
                    dvec3 intersect;
                    dvec3 normal;
                    if(object->intersect(ray, intersect, normal)) {
                        _image->setPixel(x, y, object->material.color);
                    }
                }



            }
        }
    }

    bool running() const { return _running; }
    void stop() { _running = false; }
    void start() { _running = true; }

    std::shared_ptr<Image> getImage() const { return _image; }

  private:
    bool _running = false;
    const Octree* _scene;
    Camera _camera;
    glm::dvec3 _light;
    std::shared_ptr<Image> _image;
};
