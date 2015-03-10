#include <rw/rw.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <fstream>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::loaders;
using namespace rwlibs::proximitystrategies;
using namespace std;

Q randomConfiguration(Device::Ptr device, const State &state, const CollisionDetector &detector){
	State testState;
	Math::seed(time(NULL));
	CollisionDetector::QueryResult data;
	bool collision=true;
	Q Qrand;

	while(collision){
		Qrand=Math::ranQ(device->getBounds());
		testState=state;
		device->setQ(Qrand, testState);
		collision=detector.inCollision(testState,&data);
		cout << collision << endl;
	}
	
	return Qrand;
}

int main(int argc, char** argv) {
	cout << " --- Program started --- " << endl << endl;

	const string wcFile = "/home/moro/Apuntes/ROVI/Robotics/Workcell/KukaKr16/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	const State state = wc->getDefaultState();

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	cout << randomConfiguration(device, state, detector) << endl;

	cout << " --- Program ended ---" << endl;
	return 0;
}
