/*
 * Mandatory2.cpp
 *
 *  Created on: Mar 09, 2015
 *      Author: Ignacio Torroba Balmori
 */

#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <fstream>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <queue>          // std::priority_queue

using namespace std;
using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;


class GraphNode {

private: 

	Q _configuration;
	int _ID;
	vector<int> _connections;
	double _tempD;
	double _score;

public:

	//Default constructor 
	GraphNode(){
		_configuration=Q(6, 0,0,0,0,0,0); 
		_ID=-1;
		_tempD=0;
		_score=-1;
	}

	//Constructor
	GraphNode(Q q_config, int identifier){
		_configuration=q_config; 
		_ID=identifier;
		_tempD=0;
		_score=-1;
	}

	//Methods
	Q getConfig() const {return(_configuration);}
	int getID() const {return (_ID);}
	double getTempD() const {return(_tempD);}
	vector<int> getConnections() const {return(_connections);}
	double getScore() const {return(_score);}
	void setScore(double score) {_score=score;}


	double calculateMetrics(Q possibleNeighbour, Device::Ptr device) {
		Q q1, q2;
		q1=this->_configuration;
		q2=possibleNeighbour;
		double dist=0;
		for(size_t i=0; i<device->getDOF(); i++){
			dist+=pow(q1[i]-q2[i],2);
		}
		_tempD=dist;
		return (dist);	
	}

	void newConnection(int newBrunchID){
		_connections.push_back(newBrunchID);
	}

};

class Metrics{
public:
	bool operator()(GraphNode N1, GraphNode N2)
	{
		if (N2.getTempD()<N1.getTempD()) return true;
		return false;
	}
};

//This function only avoids generating cycles of three nodes. For bigger cycles nodes must be used and the problem becomes exponential
bool checkConnections(vector<int> newNodeCon, vector<int> neighbourCon, map<int, GraphNode> nodes){
	for(unsigned int i=0; i<newNodeCon.size(); i++){
		for(unsigned int j=0; j<neighbourCon.size(); j++){
			if(newNodeCon.at(i)==neighbourCon.at(j)){
				return false;
			}
		}
	}
	return true;
}

//************Check and test, specially the condition in the last if!!************
bool collisionChecking4(Q q1, Q q2, Device::Ptr device, const State &state, const CollisionDetector &detector) {
	State testState;
	CollisionDetector::QueryResult data;

	double epsilon=0.1;
	Q dq = q2 - q1;
	double n = dq.norm2()/epsilon;
	double levels = ceil(log2(n));
	Q dq_extended = dq*(pow(2,levels)/n);
	double steps;
	Q step;
	Q qi;
	Q q_add;
	for (int i=1; i<=levels; i++) {
		steps = pow(2, i-1);
		step = dq_extended/steps;
		for (int j=1; j<=steps; j++) {
			q_add = (j-1/2)*step;			
			qi = q1 + q_add;
			testState=state;
			device->setQ(qi, testState);
			if ((q_add.norm2() < dq.norm2()) && detector.inCollision(testState, &data)) {
				return true;
			}
		}
	} 
	return false;
}


Q randomConfiguration(Device::Ptr device, const State &state, const CollisionDetector &detector){
	State testState;
	CollisionDetector::QueryResult data;
	bool collision=true;
	Q Qrand;

	int a = 0;
	while(collision){
		Qrand=Math::ranQ(device->getBounds());
		testState=state;
		device->setQ(Qrand, testState);
		collision=detector.inCollision(testState,&data);
		if (a>20000) {
			cout << "Fail to find a collision free configuration" << endl;	//*******What shall we do here??
			break;
		}
		a++;
	}
	return Qrand;
}

/**
 * Creates a false graph
 * @param graph
 */
void createFalseGraph(map<int, GraphNode> & graph){
	Q q1(6, 0,0,0,0,0,0);
	Q q2(6, 0,0,0,0,0,1);
	Q q3(6, 0,0,0,0,0,2);
	Q q4(6, 12,0,0,0,0,3);
	Q q5(6, 23,0,0,0,0,4);
	Q q6(6, 0,0,0,0,0,5);
	Q q7(6, 0,0,0,0,0,6);
	Q q8(6, 0,0,0,0,0,7);

	graph[0] = GraphNode(q1, 0);
	graph[1] = GraphNode(q2, 1);
	graph[2] = GraphNode(q3, 2);
	graph[3] = GraphNode(q4, 3);
	graph[4] = GraphNode(q5, 4);
	graph[5] = GraphNode(q6, 5);
	graph[6] = GraphNode(q7, 6);
	graph[7] = GraphNode(q8, 7);

	graph[0].newConnection(1);
	graph[0].newConnection(2);
	graph[2].newConnection(4);
	graph[2].newConnection(3);
	graph[4].newConnection(5);
	graph[4].newConnection(6);
	graph[5].newConnection(7);
	graph[6].newConnection(7);
	graph[7].newConnection(8);

	for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
		cout<<"Connections of node: "<<it->second.getID()<<", "<<endl;
		for(int i=0; i<PRMgraph.find(it->second.getID())->second.getConnections().size(); i++){
			cout<<PRMgraph.find(it->second.getID())->second.getConnections()[i]<<endl;
		}
	}

}

/**
 * Given a Graph and the Q of one of its nodes, return the ID
 * @param graph The graph to search in
 * @param q the Q to search
 * @return The ID of the Q inside the graph
 */
int findIDfromQ(map<int, GraphNode> & graph, Q & q){
	int ID = 0;
	for (unsigned int nodeIterator = 0; nodeIterator < graph.size(); nodeIterator++){
		if (q == graph[nodeIterator].getConfig()) ID = graph[nodeIterator].getID();
	}
	return ID;
}

/**
 * This method calculates the score of a node based on the A start algorithm.
 * The map's cost has two values f(x) = g(x) + h(x).
 * g(x) is the already traveled distance.
 * h(x) is the heuristic score based on the distance left from a current state to the
 * goal.
 * @param node The node to calculate its score
 * @param previousNode The father of the node
 * @param goalNode The node to finish in
 * @return The score of the node
 */
float calculateAStarScore(GraphNode & node, double currentPathScore, Q goalQ, Device::Ptr device){
	//Calculates g(x)
	double g_x = currentPathScore;
	//Calculates h(x)
	double h_x = node.calculateMetrics(goalQ, device);
	//Stores the value in the node
	node.setScore(g_x + h_x);
	//Tadaaaaaa
	cout << "Score for node " << node.getID() <<  " is: " << g_x + h_x << endl;
	return g_x + h_x;
}

/**
 * Given a Graph calculates the shortest way from the Start node to the Goal node.
 * @param PRMgraph The graph to search the path from
 * @param startNode The node from which you start
 * @param goalNode The node you want to finish in
 * @return A vector with all the nodes followed
 */
vector<GraphNode> calculatePath( map <int, GraphNode> & PRMgraph, Q startQ, Q goalQ, Device::Ptr device){
	vector<GraphNode> solutionPath; //Here the solutions nodes will be stored

	map<int, GraphNode> openList;
	map<int, GraphNode> closedList;

	GraphNode * currentNode;

	//Lets find the ID of the start and goal states inside the graph
	int ID_start = findIDfromQ(PRMgraph, startQ);
	int ID_goal = findIDfromQ(PRMgraph, goalQ);

	//Starts with the startNode
	currentNode = &PRMgraph[ID_start];
	openList[ID_start] = *currentNode;
	//And calculate its score
	calculateAStarScore(*currentNode, 0, goalQ, device);

	int limit=0;
	int tempScore=99999;
	int tempID=0;

	//While nodes on the open list
	while (!openList.empty() && limit<6){
		tempScore = 999999;
		//Choose the node with the smallest score
		cout << endl << "The openList size is: " << openList.size() << endl;
		for (auto node : openList){
			if (node.second.getScore()<=tempScore && node.second.getScore()>0){
				cout << "Score for node " << node.second.getID() << " is: " << node.second.getScore() << endl;
				tempScore = node.second.getScore();
				tempID = node.second.getID();
			}
		}
		currentNode = &openList[tempID];
		cout << "So now the current node ID is: " << currentNode->getID() << endl;
		//Check if we are in the goal
		if (currentNode->getID()==ID_goal) return solutionPath;
		//We add the current map to the closed list
		closedList[currentNode->getID()] = *currentNode;
		//And remove it from the openList
		openList.erase(currentNode->getID());
		//For all the connections, calculate the score of each one
		for (auto nodeConnected : currentNode->getConnections()){
			//Check that it is not in the closed list
			//if (closedList.count(nodeConnected)==0){
				//Put it in the openList
				//cout << "Connection ID: " << nodeConnected << endl;
				openList[nodeConnected] = PRMgraph[nodeConnected];
				//If so, calculate its score!
				calculateAStarScore(openList.find(nodeConnected)->second, currentNode->getScore(), goalQ, device);
			//}
		}

		//Put it in the solution vector
		solutionPath.push_back(*currentNode);

		limit++;
	}

	if (currentNode->getID()!=ID_goal){
		solutionPath.empty();
		cout << "Solution not found" << endl;
	}

	return solutionPath;
}









int main(int argc, char** argv) {
	//Lets put randomness to this, baby!
	Math::seed(time(NULL));

	//Initializing workcell
	cout << " --- Program started --- " << endl << endl;
	const string wcFile = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/KukaKr16/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	const State state = wc->getDefaultState();

	map <int, GraphNode> PRMgraph;






	Q start(6, 0,0,0,0,0,0);
	Q goal(6, 0,0,0,0,0,7);

	createFalseGraph(PRMgraph);
	vector<GraphNode> solution = calculatePath(PRMgraph, start, goal, device);

	cout << "The solution found is:" << endl;
	for (auto i : solution){
		cout << i.getID() << ", ";
	}
	cout << endl;

/*	//Collision detector and strategy
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	//Graph: created as a map container. The key is the node's ID
	//GraphNode* newNode;
	map <int, GraphNode> PRMgraph;
	PRMgraph.erase(PRMgraph.begin(), PRMgraph.end());
	int dale=0;
	double maxDist=4.;
	int ID=0; 

	//Set Nc
	priority_queue<GraphNode, vector<GraphNode>, Metrics> candidateNeighbours;

	//PRM ALGORITHM 
	while(dale!=100){	//Limited to the creation of three edges (for testing)

		//Generation of new collision-free q
		GraphNode newNode(randomConfiguration(device, state, detector), ID);
		cout<<"New configuration: "<<newNode.getConfig()<<" ID: "<<newNode.getID()<<endl;

		//Go through the graph looking for neighbours closer than maxDist
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
			//If distance<=maxDist, we store the node in Nc (priority queue sorted by distance)
			if(newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), device)<=maxDist){
				candidateNeighbours.push(PRMgraph.find(it->second.getID())->second);	
				cout<<"Found neighbour: "<<it->second.getID()<<" Distance: "<<newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), device)<<endl;
			}
		}

		//Go through the set of neighbors
		while(!candidateNeighbours.empty()){
			//Check if there is a graph connection already ------> avoid cycles
			if(checkConnections(newNode.getConnections(), candidateNeighbours.top().getConnections(), PRMgraph)){
				//Check for collisions in the edges
				if(!collisionChecking4(newNode.getConfig(), candidateNeighbours.top().getConfig(), device, state, detector)){
					//Create the connection in the graph (update list of connections in both nodes)
					newNode.newConnection(candidateNeighbours.top().getID());									//New connection in the new node
					PRMgraph.find(candidateNeighbours.top().getID())->second.newConnection(newNode.getID());	//New connection in the node already in the graph
					dale++;
					cout<<"Edge created between "<<newNode.getID()<<" and "<<candidateNeighbours.top().getID()<<endl;
				}
				else{
					cout<<"Collision detected in the edge"<<endl;
				}
			}	
			else{
				cout<<"Nodes already graph connected"<<endl;
			}		
			//Remove neighbour from set (priority queue)
			candidateNeighbours.pop();
		}
		//Add new node to the PRM
		PRMgraph[newNode.getID()]=newNode;
		ID++;
	}*/

	cout<<"Size of the graph: "<<PRMgraph.size()<<endl;
	cout << " --- Program ended ---" << endl;
	return 0;
}
