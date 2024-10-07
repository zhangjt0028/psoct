#include "stdafx.h"
#include "windows.h"
#include "stdio.h"
#include "conio.h"
#include "math.h"
#include "stdlib.h"
#include "Wave.h"

int GenerateWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle, int WaveType)
{
	switch(WaveType)
	{
	case WAVE_SINE:
		GenerateSineWave(DataBuffer, BufferSize, Cycle);
		break;
	case WAVE_RAND:
		GenerateRandWave(DataBuffer, BufferSize);
		break;
	case WAVE_PULSE:
		GeneratePulseWave(DataBuffer, BufferSize, Cycle);
		break;
	case WAVE_TRIGANGLE:
		TriangleWave(DataBuffer, 2047, BufferSize, Cycle);
		break;
	case WAVE_DC:
		GenerateDCWave(DataBuffer, BufferSize);
		break;
	}
	return 1;
}

// �������Ҳ�
int GenerateSineWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle)
{
	// int m_Cycle=64;
	// �����Һ����������Ҳ�����
	for(int i=0; i<BufferSize; i++)
	{
		DataBuffer[i]=(USHORT)(sin(6.28*((i)%Cycle)/Cycle)*2048+2047);
	}
	return 1;
}

// ���������
int GenerateRandWave(PUSHORT DataBuffer, LONG BufferSize)
{
	int m_Cycle=32;
	//srand(100);
	// �����Һ����������Ҳ�����
	for(int i=0; i<BufferSize; i++)
	{
		DataBuffer[i] = rand();		
	}
	return 1;
}

// �������岨
int GeneratePulseWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle)
{
	int m_Cycle=32;
	BOOL bPulseDir = 0;
	// �����Һ����������Ҳ�����
	for(int i=0; i<BufferSize; i++)
	{
		if((i%m_Cycle)==0) bPulseDir = !bPulseDir;
		if(bPulseDir)
			DataBuffer[i] = 0x0FFF;
		else
			DataBuffer[i] = 0x00;
	}
	return 1;
}

// �������ǲ�
int TriangleWave(PUSHORT pBuffer, int PeakValue, LONG BufferSize, int Cycle) 
{
	int DataPos;
	SHORT nRandom;
	for(int Index=0; Index<BufferSize; Index++)
	{
		nRandom = 0;//(SHORT)(-10.0 + 20.0*rand()/RAND_MAX);
		DataPos = Index % Cycle; // ��һ�������ڵĵ�λ��
		if( DataPos>=0 && DataPos<= Cycle/4) // 1/4����
		{
			pBuffer[Index] = 4*(PeakValue*DataPos)/Cycle + 2047 + nRandom;
		}

		if( DataPos>Cycle/4 && DataPos<=3*Cycle/4) // 1/2 ~ 3/4����
		{
			pBuffer[Index] = 4*(PeakValue*(Cycle/2 - DataPos))/Cycle + 2047 + nRandom;
		}

		if( DataPos>3*Cycle/4 && DataPos<=Cycle) // 3/4 ~ 1����
		{
			pBuffer[Index] = -4*(PeakValue*(Cycle-DataPos))/Cycle + 2047 + nRandom;
		}			
	}
	return 1;
}


// ���ɺ㶨ֵ
int GenerateDCWave(PUSHORT DataBuffer, LONG BufferSize)
{
	int m_Cycle=256;
	// �����Һ����������Ҳ�����
	for(int i=0; i<BufferSize; i+=4)
	{
		//DataBuffer[i]=(SHORT)(sin(6.28*((i)%m_Cycle)/m_Cycle)*2048+2047);
		DataBuffer[i]=0;
	}
	for(int i=0; i<BufferSize; i++)
	{
		DataBuffer[i]=4095;
	}	
	return 1;
}
