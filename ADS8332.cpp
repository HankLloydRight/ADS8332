#include "ADS8332.h"
#include "esp32-hal-spi.h"

extern SPIClass * vspi; //uninitalised pointer to SPI object

ADS8332::ADS8332(uint8_t _SelectPin, uint8_t _ConvertPin, uint8_t _EOCPin) {
	SelectPin = _SelectPin;
	ConvertPin = _ConvertPin;
	EOCPin = _EOCPin;
	pinMode(ConvertPin, OUTPUT);
	digitalWrite(ConvertPin, HIGH);
	pinMode(SelectPin, OUTPUT);
	digitalWrite(SelectPin, HIGH);
	pinMode(EOCPin, INPUT);
	EOCTimeout = 100000;
	ConnectionSettings = SPISettings(40000000, MSBFIRST, SPI_MODE2);
}

void ADS8332::setCommandBuffer(CommandRegister Command) {
	CommandBuffer = static_cast<uint16_t>(Command) << 12;
}

void ADS8332::begin() {
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::SampleRate, false);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, false);
	setConfiguration(ConfigRegisterMap::Reset, true);
	Serial.println("ADC Manual Config: ");
	Serial.println(CommandBuffer,BIN);
	sendCommandBuffer16();
}
void ADS8332::beginauto() {
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, true);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, false);
	setConfiguration(ConfigRegisterMap::SampleRate, true);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, true);
	Serial.println("ADC Auto Mode Config: ");
	Serial.println(CommandBuffer,BIN);
	sendCommandBuffer16();
}


void ADS8332::reset() {
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::SampleRate, false);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, false);
	Serial.println("ADC Reset: ");
	Serial.println(CommandBuffer,BIN);
	sendCommandBuffer16();
}

void ADS8332::setConfiguration(ConfigRegisterMap Option, bool Setting) {
	bitWrite(CommandBuffer, static_cast<uint8_t>(Option), Setting);
}

uint16_t ADS8332::getConfig() {
	setCommandBuffer(CommandRegister::ReadConfig);
	return sendCommandBuffer16();
}

void ADS8332::print_binary(uint32_t v)
{
	int mask = 0;
	int n = 0;
	int num_places = 32;
	for (n=1; n<=num_places; n++)
	{
		mask = (mask << 1) | 0x0001;
	}
	v = v & mask;  // truncate v to specified number of places
	while(num_places)
	{
		if (v & (0x0001 << (num_places-1) ))
		{
			Serial.print("1");
		}
		else
		{
			Serial.print("0");
		}
		--num_places;
	}
}


uint16_t ADS8332::sendCommandBuffer16() {
    REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin); //low
	CommandReply16 = vspi->transfer16(CommandBuffer);
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin); //high
	return CommandReply16;
}

uint8_t ADS8332::sendCommandBuffer8() {
	REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin); //low
	CommandReply8 = vspi->transfer( CommandBuffer>>8 );
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin); //high
	return CommandReply8;
}

void ADS8332::sendCommandBuffer4() {
	uint32_t xcmd=0,*cmd32=&xcmd;
	*cmd32 = static_cast<uint32_t>(CommandBuffer4<<24);
	REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin); //low
	vspi->transferBits(0,cmd32, 4) ; 
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin); //high
}

uint16_t ADS8332::getSample(uint8_t UseChannel) {
	CommandBuffer=static_cast<uint16_t>(UseChannel) << 12;
    sendCommandBuffer8();
	//CommandBuffer4=UseChannel << 4;
	//sendCommandBuffer4();
	return getSampleInteger();
}

void delay_20ns() {
    __asm__ __volatile__("nop\n nop\n nop\n nop\n nop\n");
}
uint16_t ADS8332::getSampleInteger() {
	/*if (!beginsent)	{
		begin();
		beginsent = true;
	}*/
	setCommandBuffer(CommandRegister::ReadData);
	uint32_t starttime = micros();
	
    gpio_set_level((gpio_num_t)ConvertPin,0); //digitalWrite(ConvertPin, LOW);
	//delay_20ns();
	/*
	while(keepwaiting)	{
		if (digitalRead(EOCPin) == 0)	{
			keepwaiting = false;
		}
		else	{
			if ( (micros() - starttime) > EOCTimeout) {
				digitalWrite(ConvertPin, HIGH);
				return 1;
			}
		}
	}*/
	gpio_set_level((gpio_num_t)ConvertPin,1);  //digitalWrite(ConvertPin, HIGH);
	//delay_20ns();	
	starttime = micros();
	//vspi->beginTransaction(ConnectionSettings);
	while(gpio_get_level((gpio_num_t)EOCPin) == 0) {
		if ( (micros() - starttime) > EOCTimeout) return 0;
	}
	starttime = micros();
    REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin);  //digitalWrite(SelectPin,LOW);
	SampleReturn = vspi->transfer16( CommandBuffer );
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin);  //digitalWrite(SelectPin, HIGH);
	//Serial.println("SampleReturn: "+String(SampleReturn));
	//vspi->endTransaction();
	return SampleReturn;
}

#include "esp32/rom/ets_sys.h"


uint16_t ADS8332::getSampleIntegerAndTag(uint32_t *tag) {
	setCommandBuffer(CommandRegister::ReadData);
	uint32_t starttime = micros();
    // gpio_set_level((gpio_num_t)ConvertPin,0); //digitalWrite(ConvertPin, LOW);
	//delayMicroseconds(1);
	
	while(digitalRead(EOCPin) == 1)			if ( (micros() - starttime) > EOCTimeout) return 0;
	
	// gpio_set_level((gpio_num_t)ConvertPin,1);  //digitalWrite(ConvertPin, HIGH);
	//vspi->beginTransaction(ConnectionSettings);
	
	
    REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin); 
	//digitalWrite(SelectPin,LOW);
	//gpio_set_level((gpio_num_t)SelectPin,0);
	//delay_20ns();
	//SampleReturn32 = vspi->transfer32( (uint32_t)CommandBuffer<<16 );
	SampleReturn = vspi->transfer16( CommandBuffer );
	//*tag =(uint8_t) vspi->transfer( CommandBuffer );
	 vspi->transferBits(0, tag, 3) ;
	*tag= (*tag>>5);
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin);  
	//digitalWrite(SelectPin, HIGH);
	//gpio_set_level((gpio_num_t)SelectPin,1);
	//delay_20ns();
	// *tag= (*tag & 0xE0000000) >> 29;
	*tag=(SampleReturn32 >> 13) & 0x07;
	//while(gpio_get_level((gpio_num_t)EOCPin) == 0) ;
	//Serial.println("Tag: "+String(*tag)+" Val: "+String(SampleReturn));
	//vspi->endTransaction();
	return (SampleReturn32 >> 16) & 0xFFFF;;
}

SPISettings* ADS8332::GetSPISettings(){
	return &ConnectionSettings;
}

