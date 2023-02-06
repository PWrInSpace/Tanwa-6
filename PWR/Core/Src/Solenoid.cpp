#include <Solenoid.hh>

Solenoid::Solenoid(GPIO_TypeDef* DirGPIOPort_,
				uint16_t DirPin_)
:	ValveInterface(),
	DirGPIOPort(DirGPIOPort_),
	DirPin(DirPin_)
{
	Close();
}

void Solenoid::Stop(){
	;
}

void Solenoid::Open(){
	HAL_GPIO_WritePin(DirGPIOPort, DirPin, GPIO_PIN_SET);
}

void Solenoid::Close(){
	HAL_GPIO_WritePin(DirGPIOPort, DirPin, GPIO_PIN_RESET);
}
