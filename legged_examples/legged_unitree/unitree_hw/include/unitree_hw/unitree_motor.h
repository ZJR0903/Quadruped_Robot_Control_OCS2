#pragma once
#include <stdint.h>
#include <string.h>

namespace motor{
	class UnitreeMotor
	{
		public:
			static UnitreeMotor m_Instance;
			uint8_t ID;
			uint8_t Mode;
			float T;
			float W;
			float Pos;
			uint16_t Acc;
			uint8_t Temp;    //temprature

			UnitreeMotor(){}
			~UnitreeMotor(){}
			static UnitreeMotor& GetInstance()
			{
				return m_Instance;
			}
			void parseMsg(char *msg)
			{
				char * pmsg = msg;
				// m_Instance.ID = *(uint8_t*)(pmsg+2);
				// m_Instance.Mode = *(uint8_t*)(pmsg+4);
				// m_Instance.T = float(*(uint16_t*)(pmsg+12))/256;
				// m_Instance.W = float(*(uint16_t*)(pmsg+14))/128;
				// m_Instance.Pos = float(*(int*)(pmsg+30))*6.28/16384;
				// m_Instance.Acc = *(uint16_t*)(pmsg+26);
				// m_Instance.Temp = *(uint8_t*)(pmsg+6);

				ID = *(uint8_t*)(pmsg+2);
				Mode = *(uint8_t*)(pmsg+4);
				T = float(*(int16_t*)(pmsg+12))/256;
				W = float(*(int16_t*)(pmsg+14))/128;
				Pos = float(*(int*)(pmsg+30))*6.28/16384;
				Acc = *(uint16_t*)(pmsg+26);
				Temp = *(uint8_t*)(pmsg+6);
				//printf("ID:%d,Pos:%f\n",m_Instance.ID,m_Instance.Pos);
			}
	};
}