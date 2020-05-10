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

	void insertHelper(Node* &node, uint depth, std::vector<float> point, int id){
		if(node == NULL){
			node = new Node(point, id);
		}
		else{
			//calculate dimension index(x==0,y==1)
			uint ui = depth % 2;
			if(point[ui] < node->point[ui]){
				insertHelper(node->left, depth+1, point, id);
			}
			else{
				insertHelper(node->right, depth+1, point, id);
			}
		}	
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, std::vector<int>& ids, float distanceTol){
		
		if(node !=NULL){
			float node_x = node->point[0];
			float node_y = node->point[1];
			float target_x = target[0];
			float target_y = target[1];

			bool x_dim = (node_x >= target_x - distanceTol) && (node_x <= target_x + distanceTol);
			bool y_dim = (node_y >= target_y - distanceTol) && (node_y <= target_y + distanceTol);

			if(x_dim && y_dim){
				float x_difference = node_x - target_x;
				float y_difference = node_y - target_y;
				float distance = sqrt(x_difference*x_difference + y_difference*y_difference);
				if (distance < distanceTol){
					ids.push_back(node->id);
				}
			}

			// recursive iterate
			uint si = depth % 2;
			if(target[si] - distanceTol < node->point[si]){
				searchHelper(target, node->left, depth+1, ids, distanceTol);
			}
			if(target[si] + distanceTol > node->point[si]){
				searchHelper(target, node->right, depth+1, ids, distanceTol);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, ids,distanceTol);
		return ids;
	}
	

};




