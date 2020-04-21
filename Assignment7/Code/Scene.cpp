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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    auto p = this->intersect(ray);

    if(p.happened == false) return Vector3f(0);

    if(p.emit.norm() > 0) return Vector3f(1);

    auto wo = - ray.direction;  
    auto N = p.normal;

    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);

    auto x = inter_light.coords;  
    auto ws_unorm = x - p.coords; 
    auto ws = normalize(ws_unorm);  
    auto NN = normalize(inter_light.normal); 

    Vector3f L_dir(0);
    float dist_dir = (intersect(Ray(p.coords, ws)).coords - inter_light.coords).norm();
    if ( dist_dir < 1e-4 )
    {
        L_dir = inter_light.emit * p.m->eval(ws, wo, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dotProduct(ws_unorm, ws_unorm) / pdf_light;
    }

    Vector3f L_indir(0);
    float num = get_random_float();
    if (num < RussianRoulette)
    {
        auto wi = p.m->sample(wo, N); 
        L_indir = castRay(Ray(p.coords, wi), depth) * p.m->eval(wi, wo, N) * dotProduct(wi, N) / p.m->pdf(wi, wo, N) / RussianRoulette;
    }
    L_indir = L_indir * 1.0/RussianRoulette;

    return L_dir + L_indir;

}
