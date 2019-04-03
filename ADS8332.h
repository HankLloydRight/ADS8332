#ifndef __ARDUINO_ADS8332
#define __ARDUINO_ADS8332

#include <Arduino.h>
#include <SPI.h>

class ADS8332
{
	public:
		enum class CommandRegister : uint8_t
		{
			SelectCh0 = 0,
			SelectCh1 = 1,
			SelectCh2 = 2,
			SelectCh3 = 3,
			SelectCh4 = 4,
			SelectCh5 = 5,
			SelectCh6 = 6,
			SelectCh7 = 7,
			WakeUp = 11,
			ReadConfig = 12,
			ReadData = 13,
			WriteConfig = 14,
			DefaultConfig = 15,
		};
		enum class ConfigRegisterMap : uint8_t
		{
			ChannelSelectMode = 11, //0 => manual, 1 => automatic, default => 1
			ClockSource = 10, //0 => SPI SCLK, 1 => internal clock, default => 1
			TriggerMode = 9, //0 => auto trigger, 1 => CONVST trigger, default => 1
			EOCINTPolarity = 7, //0 => EOCINT active high, 1 => EOCINT active low, default 1
			EOCINTMode = 6, //0 => pin used as INT, 1 => pin used as EOC, default 1
			ChainMode = 5, //0 => use CDI input, 1 => use EOCINT, default 1
			AutoNap = 4, //0 => Auto nap enabled, 1 => auto nap disabled, default 1
			Nap = 3, //0 => Enable nap, 1 => wake up, default => 1
			Sleep = 2, //0 => enable sleep, 1=> wake up, default => 1
			TAG = 1, //0 => Tag disabled, 1=> tag enabled, default => 1
			Reset = 0, //0 => reset, 1 => normal, default => 1
		};
		ADS8332(uint8_t SelectPin, uint8_t ConvertPin, uint8_t EOCPin);
		void begin();
		void reset();
		uint8_t getSample(float* WriteVariable, uint8_t UseChannel);
		uint8_t getSample(uint16_t* WriteVariable, uint8_t UseChannel);
		uint16_t getConfig();
		void setVref(float NewVref);
		float getVref();
		SPISettings* GetSPISettings();
	private:
		float convertVrefRange(int16_t Value);
		uint16_t sendManualSingle(uint8_t Channel);
		void setCommandBuffer(CommandRegister Command);
		void setConfiguration(ConfigRegisterMap Option, bool Setting);
		uint16_t sendCommandBuffer(bool SendLong);
		void print_binary(uint32_t v);
		void setSampleChannel();
		uint32_t bitBangData(uint32_t _send, uint8_t bitcount);
		uint8_t getSampleInteger(uint16_t* WriteVariable);
		uint8_t Channel = 0;
		bool beginsent = false;
		uint32_t EOCTimeout;
		uint16_t CommandBuffer;
		uint8_t SelectPin;
		uint8_t ConvertPin;
		uint8_t EOCPin;
		float Vref;
		SPISettings ConnectionSettings;
};

#endif