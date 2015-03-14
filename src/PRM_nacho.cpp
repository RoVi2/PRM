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
#include <iostream>
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

const double maxDist=50.0;


class GraphNode {

private: 

	Q _configuration;
	int _ID;
	vector<int> _connections;
	double _tempD;
	int _localPlannerCalls;
	int _localPlannerFails;
	double _failureRatio;
	double _nFailureRatio; 

public:

	//Default constructor ***********Change for a different robot!!
	GraphNode(){
		_configuration=Q(6, 0,0,0,0,0,0); 
		_ID=-1;
	}

	//Constructor
	GraphNode(Q q_config, int identifier){
		_configuration=q_config; 
		_ID=identifier;
		_localPlannerCalls=0;
		_localPlannerFails=0;
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

	void localPlanner(bool result){
		_localPlannerCalls++;
		if(!result){
			_localPlannerFails++;
		}

		_failureRatio=(float)_localPlannerFails/((float)_localPlannerCalls+1);
	}

	double getFailureRatio(){
		return _failureRatio;
	}

	double getNFailureRatio(){
		return _nFailureRatio;
	}

	void setNFailureRatio(double total){
		_nFailureRatio=_failureRatio/total;
	}

};

//*************Check!!
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
		if (a>10000) {
				//Qrand.zero(6);
				cout << "Fail to find a collision free configuration" << endl;	//*******What shall we do here??
				break; 
    	}
    	a++;
	}
	return Qrand;
}

Q randomBounce(GraphNode nodeInit, Device::Ptr device, const State &state, const CollisionDetector &detector){
	State testState=state;
	CollisionDetector::QueryResult data;
	Q Qfin=nodeInit.getConfig();
	bool collision;

	while(nodeInit.calculateMetrics(Qfin, testState, device)<maxDist){
		Q Qdir=Math::ranDir(6,0.1);
		collision=false;
		while(!collision){
			Qfin+=Qdir;
			cout << Qfin << endl;
			sleep(1);
			testState=state;
			device->setQ(Qfin,testState);
			collision=detector.inCollision(testState,&data);
		}
	}

	return Qfin;
}

int main(int argc, char** argv) {

	//Initial and goal configuration
	Q q_init=Q(6, 0, -120, 110, 0, 100, 0);
	Q q_goal=Q(6, -71, -96, 66, 26, 100, 0);

	//Initializing workcell
	Math::seed(time(NULL));
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
	map <int, GraphNode> PRMgraph;
	PRMgraph.erase(PRMgraph.begin(), PRMgraph.end());
	int dale=0;
	double maxDist=4.;
	int ID=0; 
	int sizeNc;
	Q newQ;
	
	//Create Nc
	priority_queue<GraphNode, vector<GraphNode>, Metrics> candidateNeighbours;

	//***********************************PRM ALGORITHM******************************************

	//***LEARNING PHASE*****//

	//1) CONSTRUCTION STEP
	while(dale<20){	//Limited to the creation of 20 edges (for testing)

		GraphNode newNode(randomConfiguration(device, state, detector), ID);
		cout<<"New configuration: "<<newNode.getConfig()<<" ID: "<<newNode.getID()<<endl;

		sizeNc=0;
		//Go throught the graph looking for neighbours closer than maxDist and creates Nc
		for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
			//If distance<=maxDist, we store the node in Nc (priority queue sorted by distance)
			if(newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), state, device)<=maxDist){
				candidateNeighbours.push(PRMgraph.find(it->second.getID())->second);	
				cout<<"-----Found neighbour: "<<it->second.getID()<<" Distance: "<<newNode.calculateMetrics(PRMgraph.find(it->second.getID())->second.getConfig(), state, device)<<endl;
			}
		}

		//Go through the set of neighbours
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
					sizeNc++;
					//A maximum of 30 neighbours are connected and then the queue is empty
					if(sizeNc>=30){
						while(!candidateNeighbours.empty()){
							candidateNeighbours.pop();
						}
					}
				}
				else{
					cout<<"Collision detected in the edge"<<endl;
				}
			}	
			else{
				cout<<"Nodes already graph connected"<<endl;
			}		
		}
		
		//Add new node to the PRM
		PRMgraph[newNode.getID()]=newNode;
		ID++;
	}
	//END OF CONSTRUCTION STEP
	cout<<"Size of the PRM: "<<PRMgraph.size()<<endl;

	//Just for testing
	/*for(map<int,GraphNode>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
		cout<<"Connections of node: "<<it->second.getID()<<", "<<endl;
		for(int i=0; i<PRMgraph.find(it->second.getID())->second.getConnections().size(); i++){
			cout<<PRMgraph.find(it->second.getID())->second.getConnections()[i]<<endl;
		}
	}*/

	//2) EXPANSION STEP

	//*****************

	//***QUERY PHASE****//

	cout << " --- Program ended ---" << endl;
	return 0;
}