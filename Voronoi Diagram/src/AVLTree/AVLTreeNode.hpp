#ifndef AVLTREENODE_HPP
#define AVLTREENODE_HPP

template <typename T>
class AVLTreeNode {
private:
    T data;
    AVLTreeNode* left;
    AVLTreeNode* right;
    AVLTreeNode* parent;
    int height;
public:
    AVLTreeNode(T value) 
      : data(value), 
        left(nullptr), 
        right(nullptr), 
        parent(nullptr),
        height(-1) 
    {}
    T getData() const { return data; }
    // T& getDataRef() { return data; }
    AVLTreeNode* getLeft() const { return left; }
    AVLTreeNode* getRight() const { return right; }
    AVLTreeNode* getParent() const { return parent; }
    AVLTreeNode* getGrandParent() const
    {
        if (parent == nullptr)
            return nullptr;
        return parent->getParent();
    }
    
    void setLeft(AVLTreeNode* node) 
    { 
        if (node != nullptr)
            node->parent = this;
        left = node;
    }
    void setRight(AVLTreeNode* node) 
    { 
        if (node != nullptr)
            node->parent = this;
        right = node;
    }
    void setData(const T& value) { data = value; }
    void setParent(AVLTreeNode* node) { parent = node; }

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

    AVLTreeNode* getMinInSubtree()
    {
        AVLTreeNode* node = this;
        while (node->left != nullptr)
            node = node->left;
        return node;
    }
    AVLTreeNode* getMaxInSubtree()
    {
        AVLTreeNode* node = this;
        while (node->right != nullptr)
            node = node->right;
        return node;
    }

    AVLTreeNode* getSuccessor()
    {
        if (right != nullptr)
            return right->getMinInSubtree();
        AVLTreeNode* node = this;
        while (node->isRightChild())
            node = node->parent;
        if(node->parent == nullptr || node->parent->right == nullptr)
            return nullptr;
        return node->parent->right->getMinInSubtree();
    }
    AVLTreeNode* getPredecessor()
    {
        if (left != nullptr)
            return left->getMaxInSubtree();
        AVLTreeNode* node = this;
        while (node->isLeftChild())
            node = node->parent;
        if(node->parent == nullptr || node->parent->left == nullptr)
            return nullptr;
        return node->parent->left->getMaxInSubtree();
    }
};

#endif // AVLTREENODE_HPP