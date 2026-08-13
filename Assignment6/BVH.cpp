#include <algorithm>
#include <cassert>
#include "BVH.hpp"

//构造函数，接收所有图元，每个叶子节点最大图元数，划分策略
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)),//限制最大图元数不超过255 
    splitMethod(splitMethod),//保存划分策略
      primitives(std::move(p))//移动所有成员指针到成员变量
{
    time_t start, stop;
    time(&start);//开始计时
    if (primitives.empty())//如果没有图元直接返回
        return;

    root = recursiveBuild(primitives);//递归构建bvn树

    time(&stop);//结束计时
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
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
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
        //将排序后的列表从中间一分为二
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);//左半部分图元
        auto rightshapes = std::vector<Object*>(middling, ending);//右半部分图元

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));//检查，左右数量之和=总数
        //递归构建左右子树
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);
        //当前节点包围盒=左右子树包围盒并集
        node->bounds = Union(node->left->bounds, node->right->bounds);
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
       // 如果节点为空，返回空
    if (node == nullptr)
        return Intersection();

    // 计算射线的逆方向和方向符号（用于 AABB 相交测试）
    Vector3f invDir(1.0f / ray.direction.x,
        1.0f / ray.direction.y,
        1.0f / ray.direction.z);
    std::array<int, 3> dirIsNeg = {
        ray.direction.x > 0 ? 0 : 1,
        ray.direction.y > 0 ? 0 : 1,
        ray.direction.z > 0 ? 0 : 1
    };

    if (!node->bounds.IntersectP(ray, invDir, dirIsNeg))//不相交
        return Intersection();

    // 如果是叶子节点，直接测试图元
    if (node->left == nullptr && node->right == nullptr) {
        return node->object->getIntersection(ray);
    }

    // 内部节点：递归左右子树
    Intersection hitLeft = getIntersection(node->left, ray);
    Intersection hitRight = getIntersection(node->right, ray);

    // 返回距离较近的交点（t 值较小）
    return hitLeft.distance < hitRight.distance ? hitLeft : hitRight;

}