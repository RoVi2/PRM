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
    vector<int> Connections;

public:

    //Constructor
    GraphNode(Q q_config, int identifier){_configuration=q_config; _ID=identifier;}

    //Methods
    Q getConfig() const {
        return(_configuration);
    }

    int getID(){
        return (_ID);
    }

    double calculateMetrics(Q possibleNeighbour) {
        return ((_configuration-possibleNeighbour).norm2());    //***************check this!!! _configuration.norm2()-possibleNeighbour.norm2()?
    }

    void newConnection(int newBrunchID){
        Connections.push_back(newBrunchID);
    }


};

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
                cout << "Fail to find a collision free configuration" << endl;  //*******What shall we do here??
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

    //Graph: created as a map container. The key is the node ID
    static GraphNode* newNode;
    map <int, GraphNode*> PRMgraph;
    PRMgraph.erase(PRMgraph.begin(), PRMgraph.end());
    int dale=0;
    double maxDist=4.;
    int ID=0; 

    //RPM ALGORITHM 
    while(dale!=3){

        //Generation of new collision-free q
        newNode= new GraphNode(randomConfiguration(device, state, detector), ID);
        cout<<"New configuration: "<<newNode->getConfig()<<" ID: "<<newNode->getID()<<endl;

        //Go throught the graph looking for neighbours (3 by now) closer than maxDist
        for(map<int,GraphNode*>::iterator it = PRMgraph.begin(); it != PRMgraph.end(); ++it) {
            //If distance<=masDist
            if(newNode->calculateMetrics(PRMgraph.find(it->second->getID())->second->getConfig())<=maxDist){
                cout<<"Found neighbour: "<<it->second->getID()<<endl;
                //Check edge collision between the possible neighbour
                if(!collisionChecking4(newNode->getConfig(), it->second->getConfig(), device, state, detector)){
                    //If no edge collision, create the connection in the graph
                    newNode->newConnection(it->second->getID());
                    dale++;
                    cout<<"Edge created between "<<newNode->getID()<<" and "<<it->second->getID()<<endl;
                }
            }
        }

        //Add q to the PRM
        PRMgraph[newNode->getID()]=newNode;
        ID++;
        
        //Wait one second to avoid generating the same q several times (seed=current time)********
        sleep(1);
    }

    cout<<"Size of the graph: "<<PRMgraph.size()<<endl;
    cout << " --- Program ended ---" << endl;
    return 0;
}
