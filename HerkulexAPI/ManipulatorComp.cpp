/*
 *  Generated sources by OPRoS Component Generator (OCG V2.1 [Symbol,Topic])
 *  
 */

#include <OPRoSInclude.h>
#include "ManipulatorComp.h"


//
// constructor declaration
//
ManipulatorComp::ManipulatorComp() {
#ifdef DEBUG
	std::cout << "ManipulatorComp()" << std::endl;
#endif
	hOprosAPI = NULL;
//	manipulator = NULL;
	manipulator = new Herkulex;
	lastError = OPROS_SUCCESS;

	portSetup();
}

//
// constructor declaration (with name)
//
ManipulatorComp::ManipulatorComp(const std::string &name) :
		Component(name) {
#ifdef DEBUG
	std::cout << "ManipulatorComp(const std::string &name)" << std::endl;
#endif
	hOprosAPI = NULL;
//	manipulator = NULL;
	manipulator = new Herkulex;
	lastError = OPROS_SUCCESS;
	portSetup();
}
//
// destructor declaration
//
ManipulatorComp::~ManipulatorComp() {
	onDestroy();
}
std::vector<OPRoS::Float64> ManipulatorComp::GetPosition() {
	//user code here
	std::vector<OPRoS::Float64> position;

	if (manipulator == NULL) {
		lastError = OPROS_PRECONDITION_NOT_MET;
		return position;
	}

	long ret = manipulator->GetPosition(position);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return position;
}
ReturnType ManipulatorComp::SetPosition(std::vector<OPRoS::Float64> position,
		std::vector<uint32_t> time) {
	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->SetPosition(position, time);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return lastError;
}
std::vector<OPRoS::Float64> ManipulatorComp::GetVelocity() {

	//user code here
	std::vector<OPRoS::Float64> velocity;

	if (manipulator == NULL) {
		lastError = OPROS_PRECONDITION_NOT_MET;
		return velocity;
	}

	long ret = manipulator->GetVelocity(velocity);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return velocity;
}
ReturnType ManipulatorComp::SetVelocity(std::vector<OPRoS::Float64> velocity) {

	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->SetVelocity(velocity);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return lastError;
}
std::vector<OPRoS::Float64> ManipulatorComp::GetTorque() {
	std::vector<OPRoS::Float64> torque;

	if (manipulator == NULL) {
		lastError = OPROS_PRECONDITION_NOT_MET;
		return torque;
	}

	long ret = manipulator->GetTorque(torque);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return torque;
	//user code here
}
ReturnType ManipulatorComp::SetTorque(std::vector<OPRoS::Float64> torque) {
	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->SetTorque(torque);
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return lastError;
}
ReturnType ManipulatorComp::RunHoming() {
	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->RunHoming();
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return lastError;
}
ReturnType ManipulatorComp::Disable() {
	//user code here
	return lastError;
}
ReturnType ManipulatorComp::Enable() {
	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->Enable();
	if (ret == API_SUCCESS) {
		lastError = OPROS_SUCCESS;
	} else if (ret == API_NOT_SUPPORTED) {
		lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		lastError = OPROS_API_EXECUTING;
	} else {
		lastError = OPROS_CALL_API_ERROR;
	}

	return lastError;
}
ReturnType ManipulatorComp::GetError() {
	//user code here

	return lastError;
}
OPRoS::Property ManipulatorComp::GetProperty() {
	//user code here
	OPRoS::Property props;

	if (manipulator == NULL) {
		props.status = lastError = OPROS_PRECONDITION_NOT_MET;
		return props;
	}

	if (manipulator->GetProperty(props) < 0) {
		props.status = OPROS_CALL_API_ERROR;
		lastError = OPROS_CALL_API_ERROR;
	}

	return props;
}
ReturnType ManipulatorComp::SetProperty(OPRoS::Property props) {
	//user code here
	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	if (manipulator->SetProperty(props) < 0) {
		return lastError = OPROS_CALL_API_ERROR;
	}

	return lastError = OPROS_SUCCESS;
}
void ManipulatorComp::portSetup() {
	ProvidedServicePort *pa1;
	pa1 = new ManipulatorServiceProvided(this);
	addPort("ManipulatorService", pa1);

	// export variable setup

}

// Call back Declaration
ReturnType ManipulatorComp::onInitialize() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onInitialize()." << std::endl;
#endif

	// if get deviceAPI.so, release lower comment.
//	OPRoS::Property props;
//	std::map<std::string, std::string> temp = getPropertyMap();
//	props.SetProperty(temp);
//
//	if (props.FindName("ApiName") == false) {
//#ifdef DEBUG
//		std::cout
//				<< "ERROR : ManipulatorComp::onInitialize() -> Can't find the APIName in property\n"
//				<< std::endl;
//#endif
//		return lastError = OPROS_FIND_PROPERTY_ERROR;
//	}
//
//#if defined(WIN32)
//	//	DLL open
//	hOprosAPI = LoadLibrary((LPCSTR)parameter.GetValue("ApiName").c_str());
//	if(hOprosAPI == NULL) {
//#ifdef DEBUG
//		std::cout << "ERROR : ManipulatorComp::onInitialize() -> Can't find the " << props.getProperty("ApiName") << std::endl;
//#endif
//
//		return lastError = OPROS_FIND_DLL_ERROR;
//	}
//
//	//	API get
//	GET_OPROS_API getOprosAPI;
//	getOprosAPI = (GET_OPROS_API)GetProcAddress(hOprosAPI, "GetAPI");
//	if(getOprosAPI == NULL) {
//#ifdef DEBUG
//		std::cout << "ERROR : ManipulatorComp::onInitialize() -> Can't get a handle of GetAPI Function" << std::endl;
//#endif
//		FreeLibrary(hOprosAPI);
//		hOprosAPI = NULL;
//		return lastError = OPROS_LOAD_DLL_ERROR;
//	}
//#else
//
//	hOprosAPI = dlopen(props.GetValue("ApiName").c_str(), RTLD_LAZY);
//	if (hOprosAPI == NULL) {
//#ifdef DEBUG
//		std::cout
//				<< "ERROR : ManipulatorComp::onInitialize() -> Can't find the "
//				<< props.GetValue("ApiName") << std::endl;
//#endif
//
//		return lastError = OPROS_FIND_DLL_ERROR;
//	}
//
//	GET_OPROS_DEVICE getOprosAPI;
//	getOprosAPI = (GET_OPROS_DEVICE) dlsym(hOprosAPI, "GetAPI");
//	char *error = dlerror();
//	if (error != NULL) {
//#ifdef DEBUG
//		std::cout
//				<< "ERROR : ManipulatorComp::onInitialize() -> Can't get a handle of GetAPI Function"
//				<< std::endl;
//#endif
//
//		dlclose(hOprosAPI);
//		hOprosAPI = NULL;
//		return lastError = OPROS_LOAD_DLL_ERROR;
//	}
//#endif
//
//	manipulator = dynamic_cast<Manipulator *>(getOprosAPI());
//	if (manipulator == NULL) {
//#ifdef DEBUG
//		std::cout
//				<< "ERROR : ManipulatorComp::onInitialize() -> Can't get a handle of Actuator API"
//				<< std::endl;
//#endif
//
//#if defined(WIN32)
//		FreeLibrary(hOprosAPI);
//#else
//		dlclose(hOprosAPI);
//#endif
//		hOprosAPI = NULL;
//		return lastError = OPROS_LOAD_DLL_ERROR;
//	}
//
//	if (manipulator->Initialize(props) != API_SUCCESS) {
//		delete manipulator;
//		manipulator = NULL;
//
//#if defined(WIN32)
//		FreeLibrary(hOprosAPI);
//#else
//		dlclose(hOprosAPI);
//#endif
//		hOprosAPI = NULL;
//		return lastError = OPROS_INITIALIZE_API_ERROR;
//	}

	return OPROS_SUCCESS;

}
ReturnType ManipulatorComp::onStart() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onStart()." << std::endl;
#endif

	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->Enable();
	if (ret == API_NOT_SUPPORTED) {
		return lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		return lastError = OPROS_API_EXECUTING;
	} else if (ret == API_ERROR) {
		return lastError = OPROS_ENABLE_API_ERROR;
	}
//
	ret = manipulator->RunHoming();
	if (ret == API_ERROR) {
		return lastError = OPROS_CALL_API_ERROR;
	} else if (ret == API_NOT_SUPPORTED) {
		return lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		return lastError = OPROS_API_EXECUTING;
	}

	return lastError = OPROS_SUCCESS;

}
ReturnType ManipulatorComp::onStop() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onStop()." << std::endl;
#endif

	if (manipulator == NULL) {
		return lastError = OPROS_PRECONDITION_NOT_MET;
	}

	long ret = manipulator->Disable();
	if (ret == API_NOT_SUPPORTED) {
		return lastError = OPROS_API_NOT_SUPPORTED_ERROR;
	} else if (ret == API_EXECUTING) {
		return lastError = OPROS_API_EXECUTING;
	} else if (ret == API_ERROR) {
		return lastError = OPROS_ENABLE_API_ERROR;
	}

	return lastError = OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onReset() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onReset()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onError() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onError()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onRecover() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onRecover()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onDestroy() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onDestroy()." << std::endl;
#endif

	if (manipulator != NULL) {
		manipulator->Finalize();
		delete manipulator;
		manipulator = NULL;
	}

	if (hOprosAPI != NULL) {
#if defined(WIN32)
		FreeLibrary(hOprosAPI);
#else
		dlclose(hOprosAPI);
#endif
		hOprosAPI = NULL;
	}

	return lastError = OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onEvent(Event *evt) {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onEvent()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onExecute() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onExecute()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onUpdated() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onUpdated()." << std::endl;
#endif
	return OPROS_SUCCESS;
}
ReturnType ManipulatorComp::onPeriodChanged() {
	// user code here
#ifdef DEBUG
	std::cout << "INFO : ManipulatorComp::onPeriodChanged()." << std::endl;
#endif
	return OPROS_SUCCESS;
}

#ifndef MAKE_STATIC_COMPONENT
#ifdef WIN32

extern "C"
{
	__declspec(dllexport) Component *getComponent();
	__declspec(dllexport) void releaseComponent(Component *pcom);
}

Component *getComponent()
{
	return new ManipulatorComp();
}

void releaseComponent(Component *com)
{
	delete com;
}

#else
extern "C" {
Component *getComponent();
void releaseComponent(Component *com);
}
Component *getComponent() {
	return new ManipulatorComp();
}

void releaseComponent(Component *com) {
	delete com;
}
#endif
#endif
