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

public:

	//Default constructor 
	GraphNode(){
		_configuration=Q(6, 0,0,0,0,0,0); 
		_ID=-1;
	}

	//Constructor
	GraphNode(Q q_config, int identifier){
		_configuration=q_config; 
		_ID=identifier;
	}

	//Methods
	Q getConfig() const {return(_configuration);}
	int getID() const {return (_ID);}
	double getTempD() const {return(_tempD);}
	vector<int> getConnections() const {return(_connections);}


	double calculateMetrics(Q possibleNeighbour, const State &state, Device::Ptr device) {
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
	Math::seed(time(NULL));
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



int main(int argc, char** argv) {

	//Initializing workcell
	cout << " --- Program started --- " << endl << endl;
	const string wcFile = "/home/ignaciotb/Documents/Semester1/Robotics1/Workcells/Kr16WorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	const State state = wc->getDefaultState();

	//Collision detector and strategy
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

		//Go throught the graph looking for neighbours closer than maxDist
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
			//If distance<=maxDist, we store the node in Nc (priority queue sorted by distance)
			if(newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), state, device)<=maxDist){
				candidateNeighbours.push(PRMgraph.find(it->second.getID())->second);	
				cout<<"Found neighbour: "<<it->second.getID()<<" Distance: "<<newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), state, device)<<endl;
			}
		}

		//Go throught the set of neighbours
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
		
		//Wait one second to avoid generating the same q several times (seed=current time)******** fix this
		sleep(1);
	}

	cout<<"Size of the graph: "<<PRMgraph.size()<<endl;
	cout << " --- Program ended ---" << endl;
	return 0;
}