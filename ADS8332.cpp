#include "ADS8332.h"

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
	CommandBuffer =            static_cast<uint16_t>(Command) << 12;
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
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, true);
	Serial.println("ADC Configuration: ");
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

uint16_t ADS8332::getSample(uint8_t UseChannel) {
	CommandBuffer=static_cast<uint16_t>(UseChannel) << 12;
	/*
	switch (UseChannel) {
		case(0):
			setCommandBuffer(CommandRegister::SelectCh0);
			break;
		case(1):
			setCommandBuffer(CommandRegister::SelectCh1);
			break;
		case(2):
			setCommandBuffer(CommandRegister::SelectCh2);
			break;
		case(3):
			setCommandBuffer(CommandRegister::SelectCh3);
			break;
		case(4):
			setCommandBuffer(CommandRegister::SelectCh4);
			break;
		case(5):
			setCommandBuffer(CommandRegister::SelectCh5);
			break;
		case(6):
			setCommandBuffer(CommandRegister::SelectCh6);
			break;
		case(7):
			setCommandBuffer(CommandRegister::SelectCh7);
			break;
		default:
			setCommandBuffer(CommandRegister::SelectCh0);
			break;
	}
	*/
    sendCommandBuffer8();
	return getSampleInteger();
}


uint16_t ADS8332::getSampleInteger() {
	/*if (!beginsent)	{
		begin();
		beginsent = true;
	}*/
	setCommandBuffer(CommandRegister::ReadData);
	uint32_t starttime = micros();
	bool keepwaiting = false;
    gpio_set_level((gpio_num_t)ConvertPin,0); //digitalWrite(ConvertPin, LOW);
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
	keepwaiting = false;
	starttime = micros();
	//vspi->beginTransaction(ConnectionSettings);
	while(keepwaiting) {
		if (digitalRead(EOCPin) == 1){
			keepwaiting = false;
		}
		else {
			if ( (micros() - starttime) > EOCTimeout) {
				return 0;
			}
		}
	}
	starttime = micros();
    REG_WRITE ( GPIO_OUT_W1TC_REG, 1<<SelectPin);  //digitalWrite(SelectPin,LOW);
	SampleReturn = vspi->transfer16( CommandBuffer );
	REG_WRITE ( GPIO_OUT_W1TS_REG, 1<<SelectPin);  //digitalWrite(SelectPin, HIGH);
	//Serial.println("SampleReturn: "+String(SampleReturn));
	//vspi->endTransaction();
	return SampleReturn>>0;
}

SPISettings* ADS8332::GetSPISettings(){
	return &ConnectionSettings;
}

