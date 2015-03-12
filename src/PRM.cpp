#include "PRM.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

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

using namespace rws;

//--------------------------------------------------------
//					 Global Paths
//--------------------------------------------------------
#if (ROBOT == 1) //KUKA
	const string robotName = "KukaKr16";
#elif (ROBOT == 2) //PA10
	const string robotName = "PA10";
#endif

#if (USER == 1) //JORGE
	const string userPath = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/" + robotName + "/Scene.wc.xml";
#elif (USER == 2) //NACHO
	const string userPath = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/" + robotName + "/Scene.wc.xml";
#elif (USER == 3) //CARLOS
	const string userPath = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/" + robotName + "/Scene.wc.xml";
#elif (USER == 4) //KIM
	const string userPath = "/home/veimox/Drive/Robot Systems/RoVi 2/Robotics 2/Projects/PRM/res/" + robotName + "/Scene.wc.xml";
#endif


//--------------------------------------------------------
//					 Plugin Methods
//--------------------------------------------------------
/**
 * Construts the plugin
 */
PRM::PRM():
    		RobWorkStudioPlugin("PRMUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	// Connect stuff from the ui component
	connect(_btn0 ,SIGNAL(pressed()),
			this, SLOT(btnPressed()) );
}

/**
 * Destructs the plugin
 */
PRM::~PRM()
{
}

/**
 * Initialize the plugin
 */
void PRM::initialize() {
	log().info() << "INITALIZE" << "\n";
	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&PRM::stateChangedListener, this, _1), this);
	/*    _rws = getRobWorkStudio();
    _rwWorkCell = _rws->getWorkCell();
    _state = _rwWorkCell->getDefaultState();

    const std::vector<rw::common::Ptr<Device> >& devices = _rwWorkCell->getDevices();
    if (devices.size() == 0)
        return;
    _device = dynamic_cast<SerialDevice*>(devices[0].get());
    std::cout <<"Loaded device " << _device->getName() << std::endl;*/


	//Auto load workcell
	//WorkCell::Ptr wc = WorkCellLoader::Factory::load("res/KukaKr16/Scene.wc.xml");
	getRobWorkStudio()->setWorkCell(WorkCellLoader::Factory::load(userPath));
}

/**
 *
 * @param workcell
 */
void PRM::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	log().info() << workcell->getFilename() << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
	_device = _wc->findDevice(robotName);
}

/**
 * Close the plugin. Mandatory definition!
 */
void PRM::close(){
}

/**
 * When the Botton in pressed the robot goes to the Goal State
 */
void PRM::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		Q qGoal = Q::zero(_device->getDOF());
		_device->setQ(qGoal,_state);
		getRobWorkStudio()->setState(_state);
		log().info() << "Button 0\n";
	}/* else if(obj==_btn1){
		Q qTemp = Q::zero(_device->getDOF());
		for(unsigned int i = 0; i < qTemp.size(); i++){
			qTemp[i] += 1;
		}
		_device->setQ(qTemp,_state);
		_rws->setState(_state);
		log().info() << "Button 1\n";
	}*/
}

void PRM::stateChangedListener(const State& state) {
}


//--------------------------------------------------------
//					 RPM Methods
//--------------------------------------------------------
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

#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1")
#else
Q_EXPORT_PLUGIN(PRM);
#endif
