
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

/*using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;*/

using namespace rw;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::loaders;
using namespace rwlibs::proximitystrategies;
using namespace std;

using namespace rws;



PRM::PRM():
    RobWorkStudioPlugin("PRMUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // Connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
}

PRM::~PRM()
{
}

void PRM::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&PRM::stateChangedListener, this, _1), this);
    _rws = getRobWorkStudio();
    _rwWorkCell = _rws->getWorkCell();
    _state = _rwWorkCell->getDefaultState();

    const std::vector<rw::common::Ptr<Device> >& devices = _rwWorkCell->getDevices();
    if (devices.size() == 0)
        return;
    _device = dynamic_cast<SerialDevice*>(devices[0].get());
    std::cout <<"Loaded device " << _device->getName() << std::endl;
}

void PRM::open(WorkCell* workcell)
{
    _rwWorkCell = workcell;
}

void PRM::close() {
}

void PRM::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
	Q qTemp = Q::zero(_device->getDOF());
        _device->setQ(qTemp,_state);
	_rws->setState(_state);
        log().info() << "Button 0\n";
    } else if(obj==_btn1){
	Q qTemp = Q::zero(_device->getDOF());
	for(int i = 0; i < qTemp.size(); i++){
	    qTemp[i] += 1;        
	}
        _device->setQ(qTemp,_state);
	_rws->setState(_state);
        log().info() << "Button 1\n";
    }


}

void PRM::stateChangedListener(const State& state) {

}

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
