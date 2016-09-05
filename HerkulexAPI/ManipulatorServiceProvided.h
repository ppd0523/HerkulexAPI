

#ifndef _ManipulatorService_PROVIDED_PORT_H
#define _ManipulatorService_PROVIDED_PORT_H

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


#include <device/Property.h>
		
#include <vector>
		
#include <vector>
		

#ifndef _IManipulatorService_CLASS_DEF
#define _IManipulatorService_CLASS_DEF
/*
 * IManipulatorService
 *
 *  The comonent inherits this class and implements methods. 
 */
class IManipulatorService
{
 public:
    virtual ReturnType SetProperty(OPRoS::Property props)=0;
    virtual OPRoS::Property GetProperty()=0;
    virtual ReturnType GetError()=0;
    virtual ReturnType Enable()=0;
    virtual ReturnType Disable()=0;
    virtual ReturnType RunHoming()=0;
    virtual ReturnType SetTorque(std::vector<OPRoS::Float64> torque)=0;
    virtual std::vector<OPRoS::Float64> GetTorque()=0;
    virtual ReturnType SetVelocity(std::vector<OPRoS::Float64> velocity)=0;
    virtual std::vector<OPRoS::Float64> GetVelocity()=0;
    virtual ReturnType SetPosition(std::vector<OPRoS::Float64> position,std::vector<uint32_t> time)=0;
    virtual std::vector<OPRoS::Float64> GetPosition()=0;
 };
#endif

/*
 * 
 * ManipulatorService : public ProvidedServicePort
 *
 */
class ManipulatorServiceProvided
	:public ProvidedServicePort, public IManipulatorService
{
protected:
    IManipulatorService *pcom;


   typedef ProvidedMethod<ReturnType> SetPropertyFuncType;
   SetPropertyFuncType *SetPropertyFunc;

   typedef ProvidedMethod<OPRoS::Property> GetPropertyFuncType;
   GetPropertyFuncType *GetPropertyFunc;

   typedef ProvidedMethod<ReturnType> GetErrorFuncType;
   GetErrorFuncType *GetErrorFunc;

   typedef ProvidedMethod<ReturnType> EnableFuncType;
   EnableFuncType *EnableFunc;

   typedef ProvidedMethod<ReturnType> DisableFuncType;
   DisableFuncType *DisableFunc;

   typedef ProvidedMethod<ReturnType> RunHomingFuncType;
   RunHomingFuncType *RunHomingFunc;

   typedef ProvidedMethod<ReturnType> SetTorqueFuncType;
   SetTorqueFuncType *SetTorqueFunc;

   typedef ProvidedMethod<std::vector<OPRoS::Float64> > GetTorqueFuncType;
   GetTorqueFuncType *GetTorqueFunc;

   typedef ProvidedMethod<ReturnType> SetVelocityFuncType;
   SetVelocityFuncType *SetVelocityFunc;

   typedef ProvidedMethod<std::vector<OPRoS::Float64> > GetVelocityFuncType;
   GetVelocityFuncType *GetVelocityFunc;

   typedef ProvidedMethod<ReturnType> SetPositionFuncType;
   SetPositionFuncType *SetPositionFunc;

   typedef ProvidedMethod<std::vector<OPRoS::Float64> > GetPositionFuncType;
   GetPositionFuncType *GetPositionFunc;


public: // default method
   virtual ReturnType SetProperty(OPRoS::Property props)
   {
		opros_assert(SetPropertyFunc != NULL);
		
            return (*SetPropertyFunc)(props);
		
   }
   virtual OPRoS::Property GetProperty()
   {
		opros_assert(GetPropertyFunc != NULL);
		
            return (*GetPropertyFunc)();
		
   }
   virtual ReturnType GetError()
   {
		opros_assert(GetErrorFunc != NULL);
		
            return (*GetErrorFunc)();
		
   }
   virtual ReturnType Enable()
   {
		opros_assert(EnableFunc != NULL);
		
            return (*EnableFunc)();
		
   }
   virtual ReturnType Disable()
   {
		opros_assert(DisableFunc != NULL);
		
            return (*DisableFunc)();
		
   }
   virtual ReturnType RunHoming()
   {
		opros_assert(RunHomingFunc != NULL);
		
            return (*RunHomingFunc)();
		
   }
   virtual ReturnType SetTorque(std::vector<OPRoS::Float64> torque)
   {
		opros_assert(SetTorqueFunc != NULL);
		
            return (*SetTorqueFunc)(torque);
		
   }
   virtual std::vector<OPRoS::Float64> GetTorque()
   {
		opros_assert(GetTorqueFunc != NULL);
		
            return (*GetTorqueFunc)();
		
   }
   virtual ReturnType SetVelocity(std::vector<OPRoS::Float64> velocity)
   {
		opros_assert(SetVelocityFunc != NULL);
		
            return (*SetVelocityFunc)(velocity);
		
   }
   virtual std::vector<OPRoS::Float64> GetVelocity()
   {
		opros_assert(GetVelocityFunc != NULL);
		
            return (*GetVelocityFunc)();
		
   }
   virtual ReturnType SetPosition(std::vector<OPRoS::Float64> position,std::vector<uint32_t> time)
   {
		opros_assert(SetPositionFunc != NULL);
		
            return (*SetPositionFunc)(position,time);
		
   }
   virtual std::vector<OPRoS::Float64> GetPosition()
   {
		opros_assert(GetPositionFunc != NULL);
		
            return (*GetPositionFunc)();
		
   }


public:
    //
    // Constructor
    //
    ManipulatorServiceProvided(IManipulatorService *fn)
    {
        pcom = fn;

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

    // generated setup function
    virtual void setup()
    {
        Method *ptr_method;
    
        ptr_method = makeProvidedMethod(&IManipulatorService::SetProperty,pcom,"SetProperty");

        opros_assert(ptr_method != NULL);
        addMethod("SetProperty",ptr_method);
        SetPropertyFunc = reinterpret_cast<SetPropertyFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::GetProperty,pcom,"GetProperty");

        opros_assert(ptr_method != NULL);
        addMethod("GetProperty",ptr_method);
        GetPropertyFunc = reinterpret_cast<GetPropertyFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::GetError,pcom,"GetError");

        opros_assert(ptr_method != NULL);
        addMethod("GetError",ptr_method);
        GetErrorFunc = reinterpret_cast<GetErrorFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::Enable,pcom,"Enable");

        opros_assert(ptr_method != NULL);
        addMethod("Enable",ptr_method);
        EnableFunc = reinterpret_cast<EnableFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::Disable,pcom,"Disable");

        opros_assert(ptr_method != NULL);
        addMethod("Disable",ptr_method);
        DisableFunc = reinterpret_cast<DisableFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::RunHoming,pcom,"RunHoming");

        opros_assert(ptr_method != NULL);
        addMethod("RunHoming",ptr_method);
        RunHomingFunc = reinterpret_cast<RunHomingFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::SetTorque,pcom,"SetTorque");

        opros_assert(ptr_method != NULL);
        addMethod("SetTorque",ptr_method);
        SetTorqueFunc = reinterpret_cast<SetTorqueFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::GetTorque,pcom,"GetTorque");

        opros_assert(ptr_method != NULL);
        addMethod("GetTorque",ptr_method);
        GetTorqueFunc = reinterpret_cast<GetTorqueFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::SetVelocity,pcom,"SetVelocity");

        opros_assert(ptr_method != NULL);
        addMethod("SetVelocity",ptr_method);
        SetVelocityFunc = reinterpret_cast<SetVelocityFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::GetVelocity,pcom,"GetVelocity");

        opros_assert(ptr_method != NULL);
        addMethod("GetVelocity",ptr_method);
        GetVelocityFunc = reinterpret_cast<GetVelocityFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::SetPosition,pcom,"SetPosition");

        opros_assert(ptr_method != NULL);
        addMethod("SetPosition",ptr_method);
        SetPositionFunc = reinterpret_cast<SetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
        ptr_method = makeProvidedMethod(&IManipulatorService::GetPosition,pcom,"GetPosition");

        opros_assert(ptr_method != NULL);
        addMethod("GetPosition",ptr_method);
        GetPositionFunc = reinterpret_cast<GetPositionFuncType *>(ptr_method);
        ptr_method = NULL;

    
    }
};
#endif
