#include <iostream>
#include "AVLTree.hpp"

void testInsert() 
{
    AVLTree<int> tree;
    tree.insert(10);
    tree.insert(20);
    tree.insert(30);
    tree.insert(40);
    tree.insert(50);
    tree.insert(25);

    auto it = tree.tbegin();
    while (it != tree.tend())
    {
        std::cout << *it << " ";
        ++it;
    }
    std::cout << std::endl;
    auto lit = tree.lbegin();
    while (lit != tree.lend())
    {
        std::cout << *lit << " ";
        ++lit;
    }
    std::cout << std::endl;
}

void testDelete() 
{
    AVLTree<int> tree;
    tree.insert(10);
    tree.insert(20);
    tree.insert(30);
    tree.insert(40);
    tree.insert(50);
    tree.insert(25);

    tree.remove(20);

    auto it = tree.tbegin();
    while (it != tree.tend())
    {
        std::cout << *it << " ";
        ++it;
    }
    std::cout << std::endl;
    auto lit = tree.lbegin();
    while (lit != tree.lend())
    {
        std::cout << *lit << " ";
        ++lit;
    }
    std::cout << std::endl;
}

void testFind() 
{
    AVLTree<int> tree;
    tree.insert(10);
    tree.insert(20);
    tree.insert(30);

}

int main() {
    testInsert();
    testDelete();
    testFind();
    return 0;
}