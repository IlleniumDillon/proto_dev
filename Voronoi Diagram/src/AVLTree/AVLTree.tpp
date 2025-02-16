#include "AVLTree.hpp"

template <typename T>
inline AVLTree<T>::AVLTree()
{
    root = nullptr;
}

template <typename T>
inline AVLTree<T>::~AVLTree()
{
    clear(root);
}

template <typename T>
inline void AVLTree<T>::insert(const T& value)
{
    AVLTreeNode<T>* node = new AVLTreeNode<T>(value);
    root = insert(root, node);
}

template <typename T>
inline void AVLTree<T>::remove(const T& value)
{
    root = remove(root, value);
}

template <typename T>
inline AVLTreeNode<T>* AVLTree<T>::contains(const T& value) const
{
    AVLTreeNode<T>* node = root;
    while (node != nullptr)
    {
        if (value == node->getData())
            return node;
        if (value < node->getData())
            node = node->getLeft();
        else
            node = node->getRight();
    }
    return nullptr;
}

template <typename T>
inline AVLTreeNode<T>* AVLTree<T>::insert(AVLTreeNode<T>* subroot, AVLTreeNode<T>* node)
{
    if (subroot == nullptr)
        return node;
    if (node->getData() < subroot->getData())
        subroot->setLeft(insert(subroot->getLeft(), node));
    else
        subroot->setRight(insert(subroot->getRight(), node));

    subroot->updateHeight();
    int balance = subroot->getBalance();

    if (balance > 1)
    {
        if (node->getData() > subroot->getLeft()->getData())
            subroot->setLeft(rotateLeft(subroot->getLeft()));
        return rotateRight(subroot);
    }
    if (balance < -1)
    {
        if (node->getData() < subroot->getRight()->getData())
            subroot->setRight(rotateRight(subroot->getRight()));
        return rotateLeft(subroot);
    }

    return subroot;
}

template <typename T>
inline AVLTreeNode<T>* AVLTree<T>::remove(AVLTreeNode<T>* subroot, const T& value)
{
    if (subroot == nullptr)
        return nullptr;
    if (value < subroot->getData())
        subroot->setLeft(remove(subroot->getLeft(), value));
    else if (value > subroot->getData())
        subroot->setRight(remove(subroot->getRight(), value));
    else
    {
        if (subroot->isLeaf())
        {
            delete subroot;
            return nullptr;
        }
        if (subroot->hasOneChild())
        {
            AVLTreeNode<T>* child = subroot->getLeft() == nullptr ? subroot->getRight() : subroot->getLeft();
            delete subroot;
            return child;
        }
        AVLTreeNode<T>* successor = subroot->getSuccessor();
        subroot->setData(successor->getData());
        subroot->setRight(remove(subroot->getRight(), successor->getData()));
    }

    subroot->updateHeight();
    int balance = subroot->getBalance();

    if (balance > 1)
    {
        if (subroot->getLeft()->getBalance() < 0)
            subroot->setLeft(rotateLeft(subroot->getLeft()));
        return rotateRight(subroot);
    }
    if (balance < -1)
    {
        if (subroot->getRight()->getBalance() > 0)
            subroot->setRight(rotateRight(subroot->getRight()));
        return rotateLeft(subroot);
    }

    return subroot;
}

template <typename T>
inline AVLTreeNode<T>* AVLTree<T>::rotateLeft(AVLTreeNode<T>* subroot)
{
    AVLTreeNode<T>* grandParent = subroot->getParent();
    AVLTreeNode<T>* y = subroot->getRight();
    AVLTreeNode<T>* T2 = y->getLeft();

    y->setParent(grandParent);

    if(grandParent != nullptr)
    {
        if(subroot->isLeftChild())
            grandParent->setLeft(y);
        else
            grandParent->setRight(y);
    }

    y->setLeft(subroot);
    subroot->setRight(T2);

    subroot->updateHeight();
    y->updateHeight();

    return y;
}

template <typename T>
inline AVLTreeNode<T>* AVLTree<T>::rotateRight(AVLTreeNode<T>* subroot)
{
    AVLTreeNode<T>* grandParent = subroot->getParent();
    AVLTreeNode<T>* y = subroot->getLeft();
    AVLTreeNode<T>* T2 = y->getRight();

    y->setParent(grandParent);

    if(grandParent != nullptr)
    {
        if(subroot->isLeftChild())
            grandParent->setLeft(y);
        else
            grandParent->setRight(y);
    }

    y->setRight(subroot);
    subroot->setLeft(T2);

    subroot->updateHeight();
    y->updateHeight();

    return y;
}

template <typename T>
inline void AVLTree<T>::clear(AVLTreeNode<T>* subroot)
{
    if (subroot == nullptr)
        return;
    clear(subroot->getLeft());
    clear(subroot->getRight());
    delete subroot;
}