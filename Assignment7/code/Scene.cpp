//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    
    if (intersection.happened) {
        if (intersection.m->hasEmission()) {
            return intersection.m->getEmission();
        }

        Vector3f light_direct = Vector3f(0, 0, 0);
        Intersection light_pos;
        float pdf_light = 0;
        sampleLight(light_pos, pdf_light);  

        auto p = intersection.coords;
        auto x = light_pos.coords;
        auto wo = ray.direction;
        auto ws = (x - p).normalized();

        Ray light_ray(p, ws);
        Intersection light_intersection = intersect(light_ray);
        if (light_intersection.distance - (x - p).norm() > -EPSILON) {
            Vector3f f_r = intersection.m->eval(wo, ws, intersection.normal);
            float dist2 = dotProduct(x - p, x - p);
            light_direct = light_pos.emit * f_r * dotProduct(ws, intersection.normal) * dotProduct(-ws, light_pos.normal) / dist2 / pdf_light;
        }

        Vector3f light_indirect = Vector3f(0, 0, 0);

        if (get_random_float() > RussianRoulette) {
            return light_direct;
        }

        Vector3f wi = (intersection.m->sample(wo, intersection.normal)).normalized();

        Ray new_ray(p, wi);
        Intersection new_intersection = intersect(new_ray);
        if (new_intersection.happened && !new_intersection.m->hasEmission()) {
            auto pdf = intersection.m->pdf(wo, wi, intersection.normal);
            light_indirect = castRay(new_ray, depth + 1) * new_intersection.m->eval(wo, wi, intersection.normal) * dotProduct(wi, intersection.normal) / pdf / RussianRoulette;
        }
        return light_direct + light_indirect;
    } else {
        return Vector3f(0, 0, 0);
    }

}
