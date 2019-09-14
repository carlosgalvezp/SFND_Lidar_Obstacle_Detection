#include "kdtree.h"

#include <cmath>
#include <stack>

namespace
{
    bool isInBox(const std::vector<float>& target, const float distanceTol,
                 const std::vector<float>& test_point)
    {
        bool result = true;

        for (std::size_t i = 0U; i < target.size(); ++i)
        {
            if (std::fabs(target[i] - test_point[i]) > distanceTol)
            {
                result = false;
                break;
            }
        }

        return result;
    }

    float distance(const std::vector<float>& a, const std::vector<float>& b)
    {
        float sum = 0.0F;

        for (std::size_t i = 0U; i < a.size(); ++i)
        {
            sum += (a[i]-b[i]) * (a[i]-b[i]);
        }

        return std::sqrt(sum);
    }
}  // namespace

Node::Node(const std::vector<float>& setPoint, const int setId, const int setDepth) :
    point(setPoint), id(setId), depth(setDepth)
{}

void KdTree::insert(const std::vector<float>& point, const int id)
{
    // Start at the root
    std::reference_wrapper<std::shared_ptr<Node>> current_node = root_;
    const std::size_t point_size = point.size();
    int depth = 0;

    while (current_node.get().get() != nullptr)
    {
        const std::size_t dim = depth % point_size;

        // Determine next node
        if (point[dim] < current_node.get()->point[dim])
        {
            current_node = current_node.get()->left;
        }
        else
        {
            current_node = current_node.get()->right;
        }
        ++depth;
    }
    current_node.get().reset(new Node(point, id, depth));
}

std::vector<int> KdTree::search(const std::vector<float>& target, float distanceTol) const
{
    std::vector<int> ids;

    const std::size_t point_size = target.size();

    std::stack<std::shared_ptr<Node>> nodes_to_search;
    nodes_to_search.push(root_);

    while(!nodes_to_search.empty())
    {
        const std::shared_ptr<Node> current_node = nodes_to_search.top();
        nodes_to_search.pop();

        // Check if point in box
        if (isInBox(target, distanceTol, current_node->point) &&
            (distance(target, current_node->point) < distanceTol))
        {
            ids.push_back(current_node->id);
        }

        const std::size_t dim = current_node->depth % point_size;

        // Continue searching
        if (current_node->left && (target[dim] - distanceTol) < current_node->point[dim])
        {
            nodes_to_search.push(current_node->left);
        }
        if (current_node->right && (target[dim] + distanceTol) > current_node->point[dim])
        {
            nodes_to_search.push(current_node->right);
        }
    }

    return ids;
}

std::shared_ptr<Node> KdTree::getRoot() const
{
    return root_;
}
