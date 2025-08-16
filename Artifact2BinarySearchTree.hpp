// header file contains class declaration for Binary Search Tree class
#include <iomanip>
#include <iostream>
using namespace std;

#ifndef BinarySearchTree_class
#define BinarySearchTree_class

struct prerequisiteListNode {
    string prerequisiteNumber;
    prerequisiteListNode* next;

    prerequisiteListNode() {
        prerequisiteNumber = "";
        next = nullptr;
    }
    prerequisiteListNode(string prereqNumber) :
        prerequisiteListNode() {
            prerequisiteNumber = prereqNumber;
        }
};

struct Course {
    string courseNumber;
    string name;
    prerequisiteListNode* prerequisiteListRoot;

    Course() {
        courseNumber = "";
        name = "";
        prerequisiteListRoot = nullptr;
    }

};

struct Node {
    Course course;
    Node* left;
    Node* right;

    // default constructor
    Node() {
        left = nullptr;
        right = nullptr;
    }

    // initialize with a bid
    Node(Course aCourse) :
        Node() {
        course = aCourse;
    }
};

//Inline function printCoursePrerequisiteList of void return with Course named aCourse as parameter
inline void printCoursePrerequisiteList(Course aCourse) {

    //Initialize prerequisiteListNode pointer named prerequisiteNode to aCourse's prerequisiteListRoot
    prerequisiteListNode* prerequisiteNode = aCourse.prerequisiteListRoot;

    //Output prerequisiteNode's prerequisiteNumber
    cout << " prerequisites: " << prerequisiteNode->prerequisiteNumber;

    //Set prerequisiteNode to prerequisiteNode's next pointer
    prerequisiteNode = prerequisiteNode->next;

    //While prerequisiteNode is not null
    while (prerequisiteNode != nullptr) {

        //Output ", " prerequisiteNode's prerequisiteNumber
        cout << ", " << prerequisiteNode->prerequisiteNumber;

        //Set prerequisiteNode to prerequisiteNode's next pointer
        prerequisiteNode = prerequisiteNode->next;
    }

    cout << endl;
}

//Inline function named recursiveNodePrint of void return with Node pointer named aNode as parameter
inline void recursiveNodePrint(Node* aNode) {

    //if aNode is not null
    if (aNode != nullptr) {

        //Call recursiveNodePrint with aNode's left pointer as parameter
        recursiveNodePrint(aNode->left);

        //Output aNode's course's courseNumber ", " aNode's course's name ", "
        cout << aNode->course.courseNumber << ", " << aNode->course.name << ", ";

        //Call printCoursePrerequisiteList with aNode's course as parameter
        printCoursePrerequisiteList(aNode->course);

        //Call recursiveNodePrint with aNode's right pointer as parameter
        recursiveNodePrint(aNode->right);
    }
}

//============================================================================
// Binary Search Tree class definition
//============================================================================

/**
 * Define a class containing data members and methods to
 * implement a binary search tree
 */
class BinarySearchTree {

private:
    Node* root;

    void addNode(Node* node, Course course);
    Node* removeNode(Node* node, string courseNumber);

public:
    BinarySearchTree();
    virtual ~BinarySearchTree();
    void inOrder(Node* aNode);
    void inOrder();
    void Insert(Course course);
    void Remove(string courseNumber);
    Course Search(string courseNumber);

    void printAllCourseBST() {
        Node* currentNode = this->root;
        recursiveNodePrint(currentNode); }

};

/**
 * Default constructor
 */
inline BinarySearchTree::BinarySearchTree() {
    //root is equal to nullptr

    this->root = nullptr;
}

/**
 * Destructor and Recursive Delete function
 */
inline void RecursiveNodeDeletion(Node* node) {
    //If node is not null
    if (node != nullptr) {

        //Call RecursiveNodeDeletion function for current node's left pointer
        RecursiveNodeDeletion(node->left);
        //Call RecursiveNodeDeletion function for current node's right pointer
        RecursiveNodeDeletion(node->right);
        //delete current node
        delete node;
    }
}
inline BinarySearchTree::~BinarySearchTree() {
    // recurse from root deleting every node
    RecursiveNodeDeletion(this->root);
}

/**
 * Insert a bid
 */
inline void BinarySearchTree::Insert(Course course) {
    //INITIALIZE Node pointer named currentNode to nul
    Node* currentNode = nullptr;

    //If root is not null
    if (root == nullptr) {

        //Set root's bid to bid, root's left pointer to null and root's right pointer to null
        this->root->course = course;
        this->root->left = nullptr;
        this->root->right = nullptr;
    }
    else {

        // Set currentNode to root
        currentNode = this->root;

        //While currentNode is not null
        while (currentNode != nullptr) {

            //If bid's bidId is less than currentNode's bid's bidId then
            if (atoi(course.courseNumber.c_str()) < atoi(currentNode->course.courseNumber.c_str())) {

                //If currentNode's left pointer is null
                if (currentNode->left == nullptr) {

                    //Set currentNode's left pointer's bid to bid, then set currentNode to null
                    currentNode->left->course = course;
                    currentNode = nullptr;
                }
                else {

                    //Set currentNode to currentNode's left pointer
                    currentNode = currentNode->left;
                }
            }
            else {

                //If currentNode's right pointer is null
                if (currentNode->right == nullptr) {

                    //Set currentNode's right pointer's bid to bid, then set currentNode to null
                    currentNode->right->course = course;
                    currentNode = nullptr;
                }
                else {

                    //Set currentNode to currentNode's right pointer
                    currentNode = currentNode->right;
                }
            }
        }
    }
}

/**
 * Remove a bid
 */
inline void BinarySearchTree::Remove(string courseNumber) {
    //Initialize Node pointers named previousNode and successorNode to null,
    //DEFINE Bid nodeData, and INITIALIZE Node pointer named currentNode to root
    Node* previousNode = nullptr;
    Node* successorNode = nullptr;
    Course nodeData;
    Node* currentNode = this->root;

    /*While currentNode is not null
     * -cycle through binary search tree to find Bid object with equivalent bidId, then
     *  reassign appropriate node data to node to be removed,
     *  then delete the node used to over-write node to be removed.
     */
    while (currentNode != nullptr) {

        //If currentNode's bid's bidId is equivalent to bidId
        if (currentNode->course.courseNumber == courseNumber) {

            //If currentNode's left pointer is null and currentNode's right pointer is null
            if (currentNode->left == nullptr && currentNode->right == nullptr) {
                //If previousNode is null
                if (previousNode == nullptr) {
                    //Set root to null
                    this->root = nullptr;
                }
                //Otherwise if previousNode's left pointer is currentNode
                else if (previousNode->left == currentNode) {

                    //Set previousNode's left pointer to null
                    previousNode->left = nullptr;
                }
                //Otherwise
                else {

                    //Set previousNode's right pointer to null
                    previousNode->right = nullptr;
                }
            }
            //Otherwise if currentNode's right pointer is null
            else if (currentNode->right == nullptr) {
                //If previousNode is null
                if (previousNode == nullptr) {

                    //root is currentNode's left pointer
                    this->root = currentNode->left;
                }
                //Otherwise if previousNode's left pointer is equivalent to currentNode
                else if (previousNode->left == currentNode) {

                    //Set previousNode's left pointer to currentNode's left pointer
                    previousNode->left = currentNode->left;
                }
                //Otherwise
                else {

                    //Set previousNode's right pointer to currentNode's right pointer
                    previousNode->right = currentNode->right;
                }
            }
            //Otherwise if currentNode's left pointer is null
            else if (currentNode->left == nullptr) {
                //If previousNode is null
                if (previousNode == nullptr) {

                    //Set root to currentNode's right pointer
                    this->root = currentNode->right;
                }
                //Otherwise if previousNode's left pointer is equivalent to currentNode
                else if (previousNode->left == currentNode) {

                    //Set previousNode's left pointer to currentNode's right pointer
                    previousNode->left = currentNode->right;
                }
                //Otherwise
                else {

                    //Set previousNode's right pointer to currentNode's right pointer
                    previousNode->right = currentNode->right;
                }
            }
            //Otherwise
            else {

                //Set successorNode to currentNode's right pointer
                successorNode = currentNode->right;

                //While successorNode's left pointer is not null
                while (successorNode->left != nullptr) {
                    //Set successorNode to successorNode's left pointer
                    successorNode = successorNode->left;
                }

                //Set nodeData to successorNode's bid, call Remove function with successorNode's bid's bidId as parameter,
                //Set currentNode's bid to nodeData
                nodeData = successorNode->course;
                Remove(successorNode->course.courseNumber);
                currentNode->course = nodeData;
            }
        }
        //Otherwise if currentNode's bid's bidId is less than bidId
        else if (atoi(currentNode->course.courseNumber.c_str()) < atoi(courseNumber.c_str())) {

            //Set previousNode to currentNode, set currentNode to currentNode's right pointer
            previousNode = currentNode;
            currentNode = currentNode->right;
        }
        //Otherwise
        else {

            //Set previousNode to currentNode, set currentNode to currentNode's left pointer
            previousNode = currentNode;
            currentNode = currentNode->left;
        }
    }

}

/**
 * Search for a bid
 */
inline Course BinarySearchTree::Search(string courseNumber) {
    // set current node equal to root
    // keep looping downwards until bottom reached or matching bidId found
    // if match found, return current bid
    // if bid is smaller than current node then traverse left
    // else larger so traverse right

    Course course;
    Node* currentNode = this->root;
    while (currentNode != nullptr) {
        if (currentNode->course.courseNumber == courseNumber) {
            return currentNode->course;
        }
        else if (atoi(currentNode->course.courseNumber.c_str()) > atoi(courseNumber.c_str())) {
            currentNode = currentNode->left;
        }
        else {
            currentNode = currentNode->right;
        }
    }

    return course;
}

inline void BinarySearchTree::inOrder(Node* node) {
    //Pre order root
    //if node is not equal to null ptr
    //InOrder not left
    //output bidID, title, amount, fund
    //InOder right
    prerequisiteListNode* nextPrerequisite;
    if (node != nullptr) {
        inOrder(node->left);
        cout << node->course.courseNumber << " | " << node->course.name << " | " << node->course.prerequisiteListRoot;

        nextPrerequisite = node->course.prerequisiteListRoot->next;
        while (nextPrerequisite != nullptr) {
            cout << " | " << nextPrerequisite->prerequisiteNumber;
            nextPrerequisite = nextPrerequisite->next;
        }
        cout << endl;

        inOrder(node->right);
    }
}


#endif