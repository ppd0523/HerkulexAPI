/*
 * Herkulex.cpp
 *
 *  Created on: 2016. 6. 16.
 *      Author: em
 */

#include <iostream>
#include <device/Property.h>
#include <device/ApiTypes.h>
#include "Herkulex.h"

Herkulex::Herkulex() {
#ifdef DEBUG
	std::cout << "Herkulex::Herkulex()" << std::endl;
#endif
}
//
Herkulex::~Herkulex() {
#ifdef DEBUG
	std::cout << "Herkulex::~Herkulex()" << std::endl;
#endif
}

int32_t Herkulex::Initialize(OPRoS::Property props) {
#ifdef DEBUG
	std::cout << "Herkulex::Initialize()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::Finalize(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Finalize()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::Enable(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Enable()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::Disable(void) {
#ifdef DEBUG
	std::cout << "Herkulex::Disable()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetProperty(OPRoS::Property props) {
#ifdef DEBUG
	std::cout << "Herkulex::SetProperty()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t GetProperty(OPRoS::Property &props) {
#ifdef DEBUG
	std::cout << "Herkulex::GetProperty()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::RunHoming(void) {
#ifdef DEBUG
	std::cout << "Herkulex::RunHoming()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetTorque(std::vector<OPRoS::Float64> torque) {
#ifdef DEBUG
	std::cout << "Herkulex::SetTorque()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::GetTorque(std::vector<OPRoS::Float64> &torque) {
#ifdef DEBUG
	std::cout << "Herkulex::GetTorque()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetVelocity(std::vector<OPRoS::Float64> velocity) {
#ifdef DEBUG
	std::cout << "Herkulex::SetVelocity()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::GetVelocity(std::vector<OPRoS::Float64> &velocity) {
#ifdef DEBUG
	std::cout << "Herkulex::GetVelocity()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::SetPosition(std::vector<OPRoS::Float64> position,
		std::vector<uint32_t> time) {
#ifdef DEBUG
	std::cout << "Herkulex::SetPosition()" << std::endl;
#endif
	return API_SUCCESS;
}

int32_t Herkulex::GetPosition(std::vector<OPRoS::Float64> &position) {
#ifdef DEBUG
	std::cout << "Herkulex::GetPosition()" << std::endl;
#endif
	return API_SUCCESS;

}
