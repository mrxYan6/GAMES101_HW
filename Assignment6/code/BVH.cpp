#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i) {
        bounds = Union(bounds, objects[i]->getBounds());
    }

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {

        if (splitMethod == SplitMethod::NAIVE) {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i) {
                centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
            }

            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        } else {
            int best_axis = -1;
            int best_offset = -1;
            double best_cost = std::numeric_limits<double>::infinity();
            double area_inv = 1.0 / bounds.SurfaceArea();

            for (int dim = 0; dim < 3; ++dim) {
                std::sort(objects.begin(), objects.end(), [&](auto o1, auto o2){
                    return o1->getBounds().Centroid()[dim] < o2->getBounds().Centroid()[dim];
                });
                int n = objects.size();
                int B = std::min(int(objects.size()), 8);
                int step = (n + B - 1) / B;
                std::vector<Bounds3> bucket(B);
                std::vector<int> count(B);
                for (int i = 0; i < n; ++i) {
                    auto index = i / step;
                    count[index]++;
                    bucket[index] = Union(bucket[index], objects[i]->getBounds());
                }
                std::vector<Bounds3> bucket_pref(bucket), bucket_suf(bucket);
                std::vector<int> count_pref(count), count_suf(count);

                for (int i = 1; i < B; ++i) {
                    bucket_pref[i] = Union(bucket_pref[i - 1], bucket_pref[i]);
                    count_pref[i] = count_pref[i - 1] + count_pref[i];
                }
                for (int i = B - 2; i >= 0; --i) {
                    bucket_suf[i] = Union(bucket_suf[i + 1], bucket_suf[i]);
                    count_suf[i] = count_suf[i + 1] + count_suf[i];
                }
                for (int offset = 1; offset < B; ++offset) {
                    //offset不含左边，含右边
                    double cost = 1.0f + (count_pref[offset - 1] * bucket_pref[offset - 1].SurfaceArea() + count_suf[offset] * bucket_suf[offset].SurfaceArea()) * area_inv;
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_axis = dim;
                        best_offset = count_pref[offset - 1];
                    }
                }
            }
            
            std::sort(objects.begin(), objects.end(), [&](auto o1, auto o2){
                return o1->getBounds().Centroid()[best_axis] < o2->getBounds().Centroid()[best_axis];
            });

            auto beginning = objects.begin();
            auto middling = objects.begin() + best_offset;
            auto ending = objects.end();
            // std::cerr << "best_axis: " << best_axis << " best_offset: " << best_offset << std::endl;
            node->left = recursiveBuild(std::vector<Object*>(beginning, middling));
            node->right = recursiveBuild(std::vector<Object*>(middling, ending));
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }

    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;

    std::array<int, 3> dirIsNeg;
    for (int i = 0; i < 3; ++i) {
        dirIsNeg[i] = ray.direction[i] < 0;
    }
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return inter;
    }

    if (node->left == nullptr && node->right == nullptr) {
        return node->object->getIntersection(ray);
    }

    auto hitLeft = getIntersection(node->left, ray);
    auto hitRight = getIntersection(node->right, ray);
    return hitLeft.distance < hitRight.distance ? hitLeft : hitRight;
}