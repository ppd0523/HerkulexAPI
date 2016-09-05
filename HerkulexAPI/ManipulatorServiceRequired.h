

#ifndef _ManipulatorService_REQUIRED_PORT_H
#define _ManipulatorService_REQUIRED_PORT_H

#include <OPRoSTypes.h>
#include <InputDataPort.h>
#include <OutputDataPort.h>
#include <InputEventPort.h>
#include <OutputEventPort.h>
#include <ProvidedServicePort.h>
#include <RequiredServicePort.h>
#include <ProvidedMethod.h>
#include <RequiredMethod.h>
#include <device/ApiTypes.h>


#include "OPRoS::Property.h"
		
#include <vector>
		
#include <vector>
		



/*
 * 
 * ManipulatorService : public RequiredServicePort
 *
 */
class ManipulatorServiceRequired
   :public RequiredServicePort
{
protected:

	typedef RequiredMethod<ReturnType> SetPropertyFuncType;
	SetPropertyFuncType *SetPropertyFunc;

	typedef RequiredMethod<OPRoS::Property> GetPropertyFuncType;
	GetPropertyFuncType *GetPropertyFunc;

	typedef RequiredMethod<ReturnType> GetErrorFuncType;
	GetErrorFuncType *GetErrorFunc;

	typedef RequiredMethod<ReturnType> EnableFuncType;
	EnableFuncType *EnableFunc;

	typedef RequiredMethod<ReturnType> DisableFuncType;
	DisableFuncType *DisableFunc;

	typedef RequiredMethod<ReturnType> RunHomingFuncType;
	RunHomingFuncType *RunHomingFunc;

	typedef RequiredMethod<ReturnType> SetTorqueFuncType;
	SetTorqueFuncType *SetTorqueFunc;

	typedef RequiredMethod<std::vector<OPRoS::Float64>> GetTorqueFuncType;
	GetTorqueFuncType *GetTorqueFunc;

	typedef RequiredMethod<ReturnType> SetVelocityFuncType;
	SetVelocityFuncType *SetVelocityFunc;

	typedef RequiredMethod<std::vector<OPRoS::Float64>> GetVelocityFuncType;
	GetVelocityFuncType *GetVelocityFunc;

	typedef RequiredMethod<ReturnType> SetPositionFuncType;
	SetPositionFuncType *SetPositionFunc;

	typedef RequiredMethod<std::vector<OPRoS::Float64>> GetPositionFuncType;
	GetPositionFuncType *GetPositionFunc;

public:
	//
	// Constructor
	//
	ManipulatorServiceRequired()
	{
            SetPropertyFunc = NULL;
            GetPropertyFunc = NULL;
            GetErrorFunc = NULL;
            EnableFunc = NULL;
            DisableFunc = NULL;
            RunHomingFunc = NULL;
            SetTorqueFunc = NULL;
            GetTorqueFunc = NULL;
            SetVelocityFunc = NULL;
            GetVelocityFunc = NULL;
            SetPositionFunc = NULL;
            GetPositionFunc = NULL;
            
	   setup();
	}

	// method implementation for required method
	ReturnType SetProperty(OPRoS::Property props)
	{
            opros_assert(SetPropertyFunc != NULL);
	
            return (*SetPropertyFunc)(props);
		
	}

	OPRoS::Property GetProperty()
	{
            opros_assert(GetPropertyFunc != NULL);
	
            return (*GetPropertyFunc)();
		
	}

	ReturnType GetError()
	{
            opros_assert(GetErrorFunc != NULL);
	
            return (*GetErrorFunc)();
		
	}

	ReturnType Enable()
	{
            opros_assert(EnableFunc != NULL);
	
            return (*EnableFunc)();
		
	}

	ReturnType Disable()
	{
            opros_assert(DisableFunc != NULL);
	
            return (*DisableFunc)();
		
	}

	ReturnType RunHoming()
	{
            opros_assert(RunHomingFunc != NULL);
	
            return (*RunHomingFunc)();
		
	}

	ReturnType SetTorque(std::vector<OPRoS::Float64> torque)
	{
            opros_assert(SetTorqueFunc != NULL);
	
            return (*SetTorqueFunc)(torque);
		
	}

	std::vector<OPRoS::Float64> GetTorque()
	{
            opros_assert(GetTorqueFunc != NULL);
	
            return (*GetTorqueFunc)();
		
	}

	ReturnType SetVelocity(std::vector<OPRoS::Float64> velocity)
	{
            opros_assert(SetVelocityFunc != NULL);
	
            return (*SetVelocityFunc)(velocity);
		
	}

	std::vector<OPRoS::Float64> GetVelocity()
	{
            opros_assert(GetVelocityFunc != NULL);
	
            return (*GetVelocityFunc)();
		
	}

	ReturnType SetPosition(std::vector<OPRoS::Float64> position,std::vector<uint32_t> time)
	{
            opros_assert(SetPositionFunc != NULL);
	
            return (*SetPositionFunc)(position,time);
		
	}

	std::vector<OPRoS::Float64> GetPosition()
	{
            opros_assert(GetPositionFunc != NULL);
	
            return (*GetPositionFunc)();
		
	}

	

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::SetProperty,"SetProperty");
        opros_assert(ptr_method != NULL);
        addMethod("SetProperty",ptr_method);
        SetPropertyFunc = reinterpret_cast<SetPropertyFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::GetProperty,"GetProperty");
        opros_assert(ptr_method != NULL);
        addMethod("GetProperty",ptr_method);
        GetPropertyFunc = reinterpret_cast<GetPropertyFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::GetError,"GetError");
        opros_assert(ptr_method != NULL);
        addMethod("GetError",ptr_method);
        GetErrorFunc = reinterpret_cast<GetErrorFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::Enable,"Enable");
        opros_assert(ptr_method != NULL);
        addMethod("Enable",ptr_method);
        EnableFunc = reinterpret_cast<EnableFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::Disable,"Disable");
        opros_assert(ptr_method != NULL);
        addMethod("Disable",ptr_method);
        DisableFunc = reinterpret_cast<DisableFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::RunHoming,"RunHoming");
        opros_assert(ptr_method != NULL);
        addMethod("RunHoming",ptr_method);
        RunHomingFunc = reinterpret_cast<RunHomingFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::SetTorque,"SetTorque");
        opros_assert(ptr_method != NULL);
        addMethod("SetTorque",ptr_method);
        SetTorqueFunc = reinterpret_cast<SetTorqueFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::GetTorque,"GetTorque");
        opros_assert(ptr_method != NULL);
        addMethod("GetTorque",ptr_method);
        GetTorqueFunc = reinterpret_cast<GetTorqueFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::SetVelocity,"SetVelocity");
        opros_assert(ptr_method != NULL);
        addMethod("SetVelocity",ptr_method);
        SetVelocityFunc = reinterpret_cast<SetVelocityFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::GetVelocity,"GetVelocity");
        opros_assert(ptr_method != NULL);
        addMethod("GetVelocity",ptr_method);
        GetVelocityFunc = reinterpret_cast<GetVelocityFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::SetPosition,"SetPosition");
        opros_assert(ptr_method != NULL);
        addMethod("SetPosition",ptr_method);
        SetPositionFunc = reinterpret_cast<SetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeRequiredMethod(&ManipulatorServiceRequired::GetPosition,"GetPosition");
        opros_assert(ptr_method != NULL);
        addMethod("GetPosition",ptr_method);
        GetPositionFunc = reinterpret_cast<GetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
