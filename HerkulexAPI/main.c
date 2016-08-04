/*
 * main.c
 *
 *  Created on: 2016. 7. 15.
 *      Author: em
 */


#include <stdio.h>

#include "Herkulex.h"
#include <device/Manipulator.h>
#include <device/ApiTypes.h>

Herkulex h;


int main(void){
	OPRoS::Property props;
	
	h.Initialize(props);
	h.Finalize();
	h.Enable();
	h.Disable();
	//	h.SetProperty();
	//	h.GetProperty();
	h.RunHoming();
	//	h.SetTorque();
	//	h.GetTorque();
	//	h.SetVelocity();
	//	h.SetPosition();

	return 0;
}
