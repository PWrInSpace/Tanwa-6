#include <DoubleMAX14870.hh>

DoubleMotor::DoubleMotor(Motor* Motor1_, Motor* Motor2_)
:	ValveInterface(),
	Motor1(Motor1_),
	Motor2(Motor2_)
{
	Motor1->Stop();
	Motor2->Stop();
}
void DoubleMotor::Stop(){
	Motor1->Stop();
	Motor2->Stop();
}
void DoubleMotor::Open(){
	Motor1->Open();
	Motor2->Open();
}

void DoubleMotor::Close(){
	Motor1->Close();
	Motor2->Close();
}
