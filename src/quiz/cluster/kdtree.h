/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <memory>
#include <functional>
#include <stack>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    int depth;
    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

    Node(std::vector<float> arr, int setId, int setDepth)
    :	point(arr), id(setId), depth(setDepth), left(nullptr), right(nullptr)
    {}
};

struct KdTree
{
    std::shared_ptr<Node> root;

    KdTree()
    : root(nullptr)
    {}

    void insert(std::vector<float> point, int id)
    {
        // Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        // Start at the root
        std::reference_wrapper<std::shared_ptr<Node>> current_node = root;
        int depth = 0;

        while (current_node.get().get() != nullptr)
        {
            // Determine next node
            if (depth % 2 == 0)  // branch on x
            {
                if (point[0] < current_node.get()->point[0])
                {
                    current_node = current_node.get()->left;
                }
                else
                {
                    current_node = current_node.get()->right;
                }
            }
            else                 // branch on y
            {
                if (point[1] < current_node.get()->point[1])
                {
                    current_node = current_node.get()->left;
                }
                else
                {
                    current_node = current_node.get()->right;
                }
            }
            ++depth;
        }
        current_node.get().reset(new Node(point, id, depth));
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;

        std::stack<std::shared_ptr<Node>> nodes_to_search;
        nodes_to_search.push(root);

        while(!nodes_to_search.empty())
        {
            const std::shared_ptr<Node> current_node = nodes_to_search.top();
            nodes_to_search.pop();

            // Check if point in box
            if (isInBox(target, distanceTol, current_node->point) &&
                distance(target, current_node->point) < distanceTol)
            {
                ids.push_back(current_node->id);
            }

            // Continue searching
            if (current_node->depth % 2 == 0)  // check x
            {
                if (current_node->left && (target[0] - distanceTol) < current_node->point[0])
                {
                    nodes_to_search.push(current_node->left);
                }
                if (current_node->right && (target[0] + distanceTol) > current_node->point[0])
                {
                    nodes_to_search.push(current_node->right);
                }
            }
            else                               // check y
            {
                if (current_node->left && (target[1] - distanceTol) < current_node->point[1])
                {
                    nodes_to_search.push(current_node->left);
                }
                if (current_node->right && (target[1] + distanceTol) > current_node->point[1])
                {
                    nodes_to_search.push(current_node->right);
                }
            }
        }

        return ids;
    }

 private:
    bool isInBox(const std::vector<float>& target, const float distanceTol,
                 const std::vector<float>& test_point)
    {
        return (std::fabs(target[0] - test_point[0]) < distanceTol) &&
               (std::fabs(target[1] - test_point[1]) < distanceTol);
    }

    float distance(const std::vector<float>& a, const std::vector<float>& b)
    {
        return std::sqrt((a[0]-b[0]) * (a[0]-b[0]) +
                         (a[1]-b[1]) * (a[1]-b[1]));
    }
};
