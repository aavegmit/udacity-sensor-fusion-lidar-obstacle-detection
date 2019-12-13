/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_helper(Node** node, int depth, std::vector<float> point, int id) {
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		else {
			int cd = depth % point.size();
			if (point[cd] < (*node)->point[cd]) {
				insert_helper(&(*node)->left, depth+1, point, id);

			}
			else {
				insert_helper(&(*node)->right, depth+1, point, id);

			}
		}
		
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, 0, point, id);

	}

	void search_helper(Node *node, std::vector<float> target, int depth, float distanceTol, std::vector<int> *ids) {

		if (node == NULL) {
			return;
		}
		int cd = depth % 3;
		
		if ( node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) && node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) ) {
			float dist = sqrt( (target[0] - node->point[0]) * (target[0] - node->point[0])  + (target[1] - node->point[1])*(target[1] - node->point[1]) );
			if (dist <= distanceTol) {
			    (*ids).push_back(node->id);
			}
		}
		if (target[cd] - distanceTol < node->point[cd]) {
			search_helper(node->left, target, depth+1, distanceTol, ids);
		}
		if (target[cd] + distanceTol > node->point[cd]) {
			search_helper(node->right, target, depth+1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root, target, 0, distanceTol, &ids);
		return ids;
	}
	

};




