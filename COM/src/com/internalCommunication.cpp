#include "../include/com/internalCommunication.h"
#include "../include/config/pinout.h"

template <typename rxType, typename txType>
InternalI2C<rxType, txType>::InternalI2C(TwoWire *_wire, uint8_t _address):
  i2c(_wire),
  address(_address)
{}

/**********************************************************************************************/

template <typename rxType, typename txType>
bool InternalI2C<rxType, txType>::sendCommand(txType *_data){
  size_t writeStatus = 0;
  i2c->beginTransmission(address);
  writeStatus = i2c->write((uint8_t*) _data, sizeof(txType));
  if(i2c->endTransmission() != 0){
    //TODO restart
    ledcWriteTone(0, 5000);
    ledcWrite(0, 255);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ledcWrite(0, 0);

    digitalWrite(RST, LOW);
    Serial.println("################# RESET STM ####################");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(RST, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);

    i2c->~TwoWire();
    i2c->begin(I2C_SDA, I2C_SCL, 100E3);



    // return false;
  }
  
  return writeStatus != 0 ? true : false; 
}


/**********************************************************************************************/

template <typename rxType, typename txType>
bool InternalI2C<rxType, txType>::getData(rxType* _data){
  
  i2c->requestFrom(address, sizeof(rxType));
  if (i2c->available()) {
    if (!i2c->readBytes((uint8_t*)_data, sizeof(rxType))) {
      
      //TODO restart
      ledcWriteTone(0, 5000);
      ledcWrite(0, 255);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ledcWrite(0, 0);


      digitalWrite(RST, LOW);
      Serial.println("################# RESET STM ####################");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(RST, HIGH);
      vTaskDelay(400 / portTICK_PERIOD_MS);
   i2c->~TwoWire();
    i2c->begin(I2C_SDA, I2C_SCL, 100E3);


      // return false;
    }
  }

  return true;

}


/**********************************************************************************************/

template <typename rxType, typename txType>
bool InternalI2C<rxType, txType>::sendCommandMotor(uint8_t _command_valve, uint8_t _command_state, uint16_t _commandValue){
  size_t writeStatus = 0;
  TxData motorMsg{00,0};
  motorMsg.command = _command_valve*10+_command_state;// combinig command valve with state (33,300) - vent vale open_timed for 300ms
  motorMsg.commandValue = _commandValue; //TODO CHECK TIME OPEN WHETHER IT WORKS
  txType *_data = &motorMsg;
  i2c->beginTransmission(address);
  writeStatus = i2c->write((uint8_t*) _data, sizeof(txType));
  if(i2c->endTransmission() != 0){
    //TODO restart
    ledcWriteTone(0, 5000);
    ledcWrite(0, 255);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ledcWrite(0, 0);


    digitalWrite(RST, LOW);
    Serial.println("################# RESET STM ####################");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(RST, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);

      i2c->~TwoWire();
    i2c->begin(I2C_SDA, I2C_SCL, 100E3);



    // return false;
  }
  
  return writeStatus != 0 ? true : false; 
}
//EXAMPLE OF RESET:
//obj.sendCommandMotor(0,RESET_COMMAND);

/**********************************************************************************************/
// template<typename rxType, typename txType>
// bool InternalI2C<rxType, txType>::sendExpander(uint8_t pin, char port, uint8_t ioDir, uint8_t state){
//   size_t writeStatus = 0;


//  if(port=='A'){
//         if(ioDir==OUTPUT){
//             ioDirA &= ~(1<<pin);
//             gppuA &= ~(1<<pin);
//         }
//         else if(ioDir==INPUT){
//             ioDirA |= (1<<pin);
//             gppuA &= ~(1<<pin);
//         }
//         else if(ioDir==INPUT_PULLUP){
//             ioDirA |= (1<<pin);
//             gppuA |= (1<<pin);
//         }
//         if(state==ON2){
//             gpioA |= (1<<pin); 
//         }
//         else if(state==OFF2){
//             gpioA &= ~(1<<pin); 
//         }

//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(GPPUA);
//         i2c->write(gppuA);
//         i2c->endTransmission();
       
//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(IODIRA);
//         i2c->write(ioDirA);
//         i2c->endTransmission();

        
//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(GPIOA);
//         i2c->write(gpioA);
//         i2c->endTransmission();

       
//     }
//     if(port=='B'){
//         if(ioDir==OUTPUT){
//             ioDirB &= ~(1<<pin);
//             gppuB &= ~(1<<pin);
//         }
//         else if(ioDir==INPUT){
//             ioDirB |= (1<<pin);
//             gppuB &= ~(1<<pin);
//         }
//         else if(ioDir==INPUT_PULLUP){
//             ioDirB |= (1<<pin);
//             gppuB |= (1<<pin);
//         }
//         if(state==ON2){
//             gpioB |= (1<<pin); 
//         }
//         else if(state==OFF2){
//             gpioB &= ~(1<<pin); 
//         }


//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(GPPUB);
//         i2c->write(gppuB);
//         i2c->endTransmission();
       
//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(IODIRB);
//         i2c->write(ioDirB);
//         i2c->endTransmission();

        
//         i2c->beginTransmission(I2C_Address);//change to i2c
//         i2c->write(GPIOB);
//         i2c->write(gpioB);
//         i2c->endTransmission();

//     }

//   if(i2c->endTransmission() != 0){
//     return false;
//   }
  
//   return writeStatus != 0 ? true : false; 
// }