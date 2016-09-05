#ifndef HERKULEX_H_
#define HERKULEX_H_

#include <device/Manipulator.h>

#define DEBUG

class Herkulex : public Manipulator{
public:
	Herkulex();
	~Herkulex();
public:
	int32_t Initialize(OPRoS::Property props);
	int32_t Finalize(void);
	int32_t Enable(void);
	int32_t Disable(void);
	int32_t SetProperty(OPRoS::Property props);
//	int32_t GetProperty(OPRoS::Property &props);
	int32_t RunHoming(void);
	int32_t SetTorque(std::vector<OPRoS::Float64> torque);
	int32_t GetTorque(std::vector<OPRoS::Float64> &torque);
	int32_t SetVelocity(std::vector<OPRoS::Float64> velocity);
	int32_t GetVelocity(std::vector<OPRoS::Float64> &velocity);
	int32_t SetPosition(std::vector<OPRoS::Float64> position,
				std::vector<uint32_t> time);
	int32_t GetPosition(std::vector<OPRoS::Float64> &position);
};

#endif /* HERKULEX_H_ */
