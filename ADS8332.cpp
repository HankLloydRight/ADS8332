#include "ADS8332.h"

ADS8332::ADS8332(uint8_t _SelectPin, uint8_t _ConvertPin, uint8_t _EOCPin)
{
	SelectPin = _SelectPin;
	ConvertPin = _ConvertPin;
	EOCPin = _EOCPin;
	pinMode(ConvertPin, OUTPUT);
	digitalWrite(ConvertPin, HIGH);
	pinMode(SelectPin, OUTPUT);
	digitalWrite(SelectPin, HIGH);
	pinMode(EOCPin, INPUT);
	Vref = 2.5;
	EOCTimeout = 100000;
	ConnectionSettings = SPISettings(12000000, MSBFIRST, SPI_MODE1);
}

void ADS8332::setCommandBuffer(CommandRegister Command)
{
	CommandBuffer = 0;
	CommandBuffer = ((uint16_t)static_cast<uint8_t>( Command )) << 12;
}

void ADS8332::begin()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::SampleRate, true);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, true);
	//Serial.println(CommandBuffer,BIN);
	sendCommandBuffer(true);
	//sendWriteCommandBuffer();
}

void ADS8332::reset()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::SampleRate, true);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, false);
	//Serial.println(CommandBuffer,BIN);
	sendCommandBuffer(true);
	//sendWriteCommandBuffer();
}

void ADS8332::setConfiguration(ConfigRegisterMap Option, bool Setting)
{
	bitWrite(CommandBuffer, static_cast<uint8_t>(Option), Setting);
}

void ADS8332::setVref(float NewVref)
{
	Vref = NewVref;
}

float ADS8332::getVref()
{
	return Vref;
}

uint16_t ADS8332::getConfig()
{
	setCommandBuffer(CommandRegister::ReadConfig);
	return sendCommandBuffer(true);
	//return sendReadCommandBuffer();
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

uint16_t ADS8332::sendCommandBuffer(bool SendLong)
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	TempOutput.UIntLargeData = CommandBuffer;
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	if (SendLong)
	{
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
		TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
	}
	else
	{
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
	}
	digitalWrite(SelectPin, HIGH);
	SPI.endTransaction();
/*	Serial.print("O:");
	Serial.print(TempOutput.UIntSmallData[1]);
	Serial.print(":");
	Serial.print(TempOutput.UIntSmallData[0]);
	Serial.print(";\n");*/
	return TempInput.UIntLargeData;
}

uint8_t ADS8332::getSample(float* WriteVariable, uint8_t UseChannel)
{
	uint16_t IntegerValue = 0;
	uint8_t status = getSample(&IntegerValue, UseChannel);
	*WriteVariable = Vref * ( (float)(IntegerValue) / 65535.0);
	return status;
}

uint8_t ADS8332::getSample(uint16_t* WriteVariable, uint8_t UseChannel)
{
	Channel = (uint8_t)( constrain(UseChannel,0,7) );
	setSampleChannel();
	return getSampleInteger(WriteVariable);
}

void ADS8332::setSampleChannel()
{
	switch (Channel)
	{
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
	sendCommandBuffer(false);
}

uint8_t ADS8332::getSampleInteger(uint16_t* WriteVariable)
{
	if (!beginsent)
	{
		begin();
		beginsent = true;
	}
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	setCommandBuffer(CommandRegister::ReadData);
	TempOutput.UIntLargeData = CommandBuffer;
	uint32_t starttime = micros();
	bool keepwaiting = true;
	digitalWrite(ConvertPin, LOW);
	while(keepwaiting)
	{
		if (digitalRead(EOCPin) == 0)
		{
			keepwaiting = false;
		}
		else
		{
			if ( (micros() - starttime) > EOCTimeout)
			{
				digitalWrite(ConvertPin, HIGH);
				return 1;
			}
		}
	}
	digitalWrite(ConvertPin, HIGH);
	keepwaiting = true;
	starttime = micros();
	SPI.beginTransaction(ConnectionSettings);
	while(keepwaiting)
	{
		if (digitalRead(EOCPin) == 1)
		{
			keepwaiting = false;
		}
		else
		{
			if ( (micros() - starttime) > EOCTimeout)
			{
				return 2;
			}
		}
	}
	starttime = micros();
	keepwaiting = true;
	uint8_t TAGData = 255;
	bool ChannelCorrect = false;
	bool TagBlank = false;
	uint8_t ChannelTag = 255;
	while(keepwaiting)
	{
		digitalWrite(SelectPin,LOW);
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
		TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
		TAGData = SPI.transfer( 0 );
		digitalWrite(SelectPin, HIGH);
		SPI.endTransaction();
		ChannelTag = (uint8_t)(TAGData>>5);
		ChannelCorrect = ( ChannelTag == Channel );
		TagBlank = (uint8_t)(TAGData << 3) == (uint8_t)(0);
		if (ChannelCorrect && TagBlank)
		{
			/*Serial.print("ADCS ");
			Serial.print(ChannelTag);
			Serial.print(",");
			Serial.print(Channel);
			Serial.print(",");
			Serial.print(TempInput.UIntLargeData);
			Serial.print("\n");*/
			*WriteVariable = TempInput.UIntLargeData;
			return 0;
		}
		else
		{
			if ( (micros() - starttime) > EOCTimeout)
			{
				/*
				Serial.print("ADCE ");
				Serial.print(ChannelTag);
				Serial.print(",");
				Serial.print(Channel);
				Serial.print(",");
				Serial.print(TempInput.UIntLargeData);
				Serial.print("\n");
				*/
				return 3;
			}
			else
			{
				setSampleChannel();
			}
		}
	}
	return 4;
}

SPISettings* ADS8332::GetSPISettings()
{
	return &ConnectionSettings;
}

/*


void ADS8330::sendShortCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataOutput.UInt16Data[1] = CommandBuffer;
	bitBangData(DataOutput.UInt32Data,ShortCommandLength);
}

void ADS8330::sendWriteCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataOutput.UInt16Data[1] = CommandBuffer;
	bitBangData(DataOutput.UInt32Data,WriteCommandLength);
}

uint16_t ADS8330::sendReadCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataConverter DataReturn;
	DataOutput.UInt16Data[1] = CommandBuffer;
	DataReturn.UInt32Data = bitBangData(DataOutput.UInt32Data,WriteCommandLength);
	return DataReturn.UInt16Data[0];
}

uint32_t ADS8330::bitBangData(uint32_t _send, uint8_t bitcount)
{
	pinMode(MISOPin, INPUT);
	pinMode(SelectPin, OUTPUT);
	pinMode(SCKPin, OUTPUT);
	pinMode(MOSIPin, OUTPUT);
	digitalWrite(SCKPin, LOW);
	uint32_t _receive = 0;
	uint8_t limit = 31-bitcount;
	Serial.print("O:");
	print_binary(_send);
	Serial.print("\n");
	for(int8_t i = 31; i>limit; i--)
	{
		digitalWrite(SelectPin, LOW);
		digitalWrite(MOSIPin, bitRead(_send, i));
		digitalWrite(SCKPin, HIGH);
		bitWrite(_receive, i, digitalRead(MISOPin));
		digitalWrite(SCKPin, LOW);
		if (bitRead(_send, i))
		{
			Serial.print("1");
		}
		else
		{
			Serial.print("0");
		}
	}
	//Serial.print("\n");
	_receive = _receive;
	digitalWrite(SelectPin, HIGH);
	Serial.print("I:");
	print_binary(_receive);
	Serial.print("\n");
	return _receive;
}

void ADS8330::getSamples(float* Channel0, float* Channel1)
{
	uint16_t Channel0Int;
	uint16_t Channel1Int;
	getSamples(&Channel0Int, &Channel1Int);
	*Channel0 = Vref * ( (float)(Channel0Int) / 65535.0);
	*Channel1 = Vref * ( (float)(Channel1Int) / 65535.0);
}


uint16_t ADS8330::sendCommandBuffer()
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	TempOutput.UIntLargeData = CommandBuffer;
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
	TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
	SPI.endTransaction();
	digitalWrite(SelectPin, HIGH);
	return TempInput.UIntLargeData;
}

void ADS8330::getSamples(uint16_t* Channel0, uint16_t* Channel1)
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	bool HaveChannel0Data = false;
	bool HaveChannel1Data = false;
	uint8_t AttemptCount = 0;
	uint8_t ResetCount = 0;
	DataConverter TempInput;
	DataConverter TempOutput;
	setCommandBuffer(CommandRegister::ReadData);
	TempOutput.UIntLargeData = CommandBuffer;
	while ( !( HaveChannel0Data && HaveChannel1Data ) )
	{
		SPI.beginTransaction(ConnectionSettings);
		SPI.transfer( 0 );
		digitalWrite(SelectPin,LOW);
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
		TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
		uint8_t TAGData = SPI.transfer( 0 );
		uint8_t BlankData = SPI.transfer( 0 );
		print_binary(TempInput.UIntSmallData[1]);
		Serial.print(":");
		print_binary(TempInput.UIntSmallData[0]);
		Serial.print(":");
		print_binary(TAGData);
		Serial.print(":");
		print_binary(BlankData);
		Serial.print("=");
		if ( ( (uint8_t)(TAGData << 1) == (uint8_t)(0) ) && ( (uint8_t)(BlankData)==(uint8_t)(0) ) )
		{
			if (bitRead(TAGData,7))
			{
				*Channel1 = TempInput.UIntLargeData;
				HaveChannel1Data = true;
				Serial.print("A");
			}
			else
			{
				*Channel0 = TempInput.UIntLargeData;
				HaveChannel0Data = true;
				Serial.print("B");
			}
		}
		else
		{
			Serial.print("N");
		}
		Serial.print("\n");
		digitalWrite(SelectPin, HIGH);
		AttemptCount++;
		if (AttemptCount > 16)
		{
			Serial.print("Failed to collect data for ADS8330.");
			Serial.print("\n");
			digitalWrite(SelectPin, HIGH);
			SPI.endTransaction();
			setCommandBuffer(CommandRegister::WakeUp);
			sendCommandBuffer();
			reset();
			begin();
			SPI.beginTransaction(ConnectionSettings);
			SPI.transfer( 0 );
			digitalWrite(SelectPin, LOW);
			ResetCount++;
			if (ResetCount > 4)
			{
				HaveChannel1Data = true;
				HaveChannel0Data = true;
				Serial.print("Failed to reboot ADS8330.");
				Serial.print("\n");
			}
		}
		SPI.endTransaction();
	}
}
*/

/*
uint16_t ADS8330::getSampleInteger(bool UseChannel0)
{
	if (UseChannel0)
	{
		setCommandBuffer(CommandRegister::SelectCh0);
	}
	else
	{
		setCommandBuffer(CommandRegister::SelectCh1);
	}
	sendCommandBuffer();
	setCommandBuffer(CommandRegister::ReadData);
	return sendCommandBuffer();
}
void ADS8330::begin()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, false);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, false);
	setConfiguration(ConfigRegisterMap::Reset, true);
	Serial.println(CommandBuffer,BIN);
	sendCommandBuffer();
}
void ADS8330::getSample(float* WriteVariable, bool UseChannel0)
{
	uint16_t IntegerValue = getSampleInteger(UseChannel0);
	*WriteVariable = Vref * ( (float)(IntegerValue) / 65535.0);
}
void ADS8330::getSample(uint16_t* WriteVariable, bool UseChannel0)
{
	*WriteVariable = getSampleInteger(UseChannel0);
}
*/