#ifndef AVLTREE_HPP
#define AVLTREE_HPP

#include "AVLTreeNode.hpp"
#include <stack>

template <typename T>
class AVLTree {
public:
    AVLTree();
    ~AVLTree();

    void insert(const T& value);
    void remove(const T& value);
    AVLTreeNode<T>* contains(const T& value) const;

private:
    AVLTreeNode<T>* root;

    AVLTreeNode<T>* insert(AVLTreeNode<T>* subroot, AVLTreeNode<T>* node);
    AVLTreeNode<T>* remove(AVLTreeNode<T>* subroot, const T& value);
    AVLTreeNode<T>* rotateLeft(AVLTreeNode<T>* subroot);
    AVLTreeNode<T>* rotateRight(AVLTreeNode<T>* subroot);
    void clear(AVLTreeNode<T>* subroot);
public:
    //所有节点的迭代器
    class TreeIterator 
    {
    public:
        TreeIterator(AVLTreeNode<T>* node)
        {
            while (node != nullptr)
            {
                stack.push(node);
                node = node->getLeft();
            }
        }
        T operator*()
        {
            return stack.top()->getData();
        }
        TreeIterator& operator++()
        {
            if(stack.empty())
                return *this;
            AVLTreeNode<T>* node = stack.top();
            stack.pop();
            if (node->getRight() != nullptr)
            {
                node = node->getRight();
                while (node != nullptr)
                {
                    stack.push(node);
                    node = node->getLeft();
                }
            }
            return *this;
        }
        bool operator!=(const TreeIterator& other) const
        {
            if (stack.empty() && other.stack.empty())
                return false;
            if (stack.empty() || other.stack.empty())
                return true;
            return stack.top() != other.stack.top();
        }
    private:
        std::stack<AVLTreeNode<T>*> stack;
    };

    TreeIterator tbegin() const
    {
        return TreeIterator(root);
    }
    TreeIterator tend() const
    {
        return TreeIterator(nullptr);
    }

    //叶子节点的迭代器
    class LeafIterator 
    {
    public:
        LeafIterator(AVLTreeNode<T>* node)
        {
            if(node == nullptr)
                return;
            stack.push(node);
            while(!stack.empty())
            {
                AVLTreeNode<T>* node = stack.top();
                stack.pop();
                if(node->isLeaf())
                {
                    stack.push(node);
                    break;
                }
                if(node->getRight() != nullptr)
                    stack.push(node->getRight());
                if(node->getLeft() != nullptr)
                    stack.push(node->getLeft());
            }
        }
        T operator*()
        {
            return stack.top()->getData();
        }
        LeafIterator& operator++()
        {
            if(stack.empty())
                return *this;

            AVLTreeNode<T>* node = stack.top();
            stack.pop();
            while(!stack.empty())
            {
                AVLTreeNode<T>* temp = stack.top();
                stack.pop();
                if(temp->isLeaf())
                {
                    stack.push(temp);
                    break;
                }
                if(temp->getRight() != nullptr && temp->getRight() != node)
                    stack.push(temp->getRight());
                if(temp->getLeft() != nullptr && temp->getLeft() != node)
                    stack.push(temp->getLeft());
            }            

            return *this;
        }
        bool operator!=(const LeafIterator& other) const
        {
            if (stack.empty() && other.stack.empty())
                return false;
            if (stack.empty() || other.stack.empty())
                return true;
            return stack.top() != other.stack.top();
        }
    private:
        std::stack<AVLTreeNode<T>*> stack;
    };

    LeafIterator lbegin() const
    {
        return LeafIterator(root);
    }
    LeafIterator lend() const
    {
        return LeafIterator(nullptr);
    }
};

#include "AVLTree.tpp"

#endif // AVLTREE_HPP
