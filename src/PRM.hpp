#ifndef PRM_HPP
#define PRM_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>

#include "ui_PRM.h"

class PRM: public rws::RobWorkStudioPlugin, private Ui::PRM
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
    PRM();
	virtual ~PRM();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:

    rws::RobWorkStudio* _rws;
    rw::kinematics::State _state;
    rw::models::WorkCell::Ptr _rwWorkCell;
    rw::models::SerialDevice *_device;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
