#ifndef NODE_HPP
#define NODE_HPP

#include <cmath>
#include <algorithm>

class Node
{
public:
    Node* left = nullptr; 
    Node* right = nullptr;
    Node* parent = nullptr;
    int height = -1;
public:
    Node(){}
    Node* grandParent() const
    {
        if (parent == nullptr)
            return nullptr;
        return parent->parent;
    }
    virtual double get_key() const { return 0; }
    void setLeft(Node* node) 
    { 
        if (node != nullptr)
            node->parent = this;
        left = node;
    }
    void setRight(Node* node) 
    { 
        if (node != nullptr)
            node->parent = this;
        right = node;
    }

    int calculateHeight()
    {
        int leftHeight = left == nullptr ? 0 : left->getHeight();
        int rightHeight = right == nullptr ? 0 : right->getHeight();
        return 1 + std::max(leftHeight, rightHeight);
    }
    int getHeight()
    {
        if (height == -1)
        {
            height = calculateHeight();
        }
        return height;
    }
    int getBalance()
    {
        int leftHeight = left == nullptr ? 0 : left->getHeight();
        int rightHeight = right == nullptr ? 0 : right->getHeight();
        return leftHeight - rightHeight;
    }
    void updateHeight()
    {
        height = calculateHeight();
    }
    void updateAllHeight()
    {
        updateHeight();
        if (parent != nullptr)
            parent->updateAllHeight();
    }

    bool isLeftChild() const
    {
        return parent != nullptr && parent->left == this;
    }
    bool isRightChild() const
    {
        return parent != nullptr && parent->right == this;
    }
    bool isLeaf() const
    {
        return left == nullptr && right == nullptr;
    }
    bool hasOneChild() const
    {
        return (left == nullptr && right != nullptr) || (left != nullptr && right == nullptr);
    }
    bool hasTwoChildren() const
    {
        return left != nullptr && right != nullptr;
    }

    Node* getMinInSubtree()
    {
        Node* node = this;
        while (node->left != nullptr)
            node = node->left;
        return node;
    }
    Node* getMaxInSubtree()
    {
        Node* node = this;
        while (node->right != nullptr)
            node = node->right;
        return node;
    }

    Node* getSuccessor()
    {
        if (right != nullptr)
            return right->getMinInSubtree();
        Node* node = this;
        while (node->isRightChild())
            node = node->parent;
        if(node->parent == nullptr || node->parent->right == nullptr)
            return nullptr;
        return node->parent->right->getMinInSubtree();
    }
    Node* getPredecessor()
    {
        if (left != nullptr)
            return left->getMaxInSubtree();
        Node* node = this;
        while (node->isLeftChild())
            node = node->parent;
        if(node->parent == nullptr || node->parent->left == nullptr)
            return nullptr;
        return node->parent->left->getMaxInSubtree();
    }
};

#endif // NODE_HPP