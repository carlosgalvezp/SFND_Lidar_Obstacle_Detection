/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <memory>
#include <functional>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

    Node(std::vector<float> arr, int setId)
    :	point(arr), id(setId), left(nullptr), right(nullptr)
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
        current_node.get().reset(new Node(point, id));
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        return ids;
    }


};
