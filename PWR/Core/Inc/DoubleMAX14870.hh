#ifndef INC_DOUBLEMAX14870_HH_
#define INC_DOUBLEMAX14870_HH_

#include <Valve.hh>
#include <MAX14870.hh>
#include "stm32f1xx_hal.h"


class DoubleMotor: public ValveInterface{
	Motor* Motor1;
	Motor* Motor2;
public:
	DoubleMotor(Motor* Motor1_,
				Motor* Motor2_);
	void Stop() override;
	void Open() override;
	void Close() override;
};

#endif /* INC_DOUBLEMAX14870_HH_ */
