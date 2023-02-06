#ifndef INC_SOLENOID_HH_
#define INC_SOLENOID_HH_

#include <Valve.hh>
#include "stm32f1xx_hal.h"

class Solenoid: public ValveInterface{
	GPIO_TypeDef* DirGPIOPort;
	uint16_t DirPin;
public:
	Solenoid(	GPIO_TypeDef* DirGPIOPort_,
				uint16_t DirPin_);
	void Stop() override;
	void Open() override;
	void Close() override;
};

#endif /* INC_SOLENOID_HH_ */
