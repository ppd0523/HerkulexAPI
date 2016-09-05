#ifndef _ManipulatorComp_COMPONENT_H
#define _ManipulatorComp_COMPONENT_H
/*
 *  Generated sources by OPRoS Component Generator (OCG V2.0[Symbol])
 *   
 */

#define DEBUG

#include <OPRoSInclude.h>
#include <device/Manipulator.h>
#include "ManipulatorServiceProvided.h"
#include "Herkulex.h"

class ManipulatorComp: public Component, public IManipulatorService {
protected:
// service
// method for provider
	IManipulatorService *ptrManipulatorService;

protected:
	Manipulator *manipulator;
	OprosDeviceHandle hOprosAPI;
	ReturnType lastError;

public:
	virtual ReturnType SetProperty(OPRoS::Property props);
	virtual OPRoS::Property GetProperty();
	virtual ReturnType GetError();
	virtual ReturnType Enable();
	virtual ReturnType Disable();
	virtual ReturnType RunHoming();
	virtual ReturnType SetTorque(std::vector<OPRoS::Float64> torque);
	virtual std::vector<OPRoS::Float64> GetTorque();
	virtual ReturnType SetVelocity(std::vector<OPRoS::Float64> velocity);
	virtual std::vector<OPRoS::Float64> GetVelocity();
	virtual ReturnType SetPosition(std::vector<OPRoS::Float64> position,
			std::vector<uint32_t> time);
	virtual std::vector<OPRoS::Float64> GetPosition();

protected:
// data

//event

// method for provider

// method for required

// symbol value generation

public:
	ManipulatorComp();
	ManipulatorComp(const std::string &compId);
	virtual ~ManipulatorComp();
	virtual void portSetup();

protected:
	virtual ReturnType onInitialize();
	virtual ReturnType onStart();
	virtual ReturnType onStop();
	virtual ReturnType onReset();
	virtual ReturnType onError();
	virtual ReturnType onRecover();
	virtual ReturnType onDestroy();

public:
	virtual ReturnType onEvent(Event *evt);
	virtual ReturnType onExecute();
	virtual ReturnType onUpdated();
	virtual ReturnType onPeriodChanged();

};

#endif
