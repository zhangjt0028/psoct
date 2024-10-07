// ��������
#define WAVE_SINE 0 // ���Ҳ�
#define WAVE_RAND 1 // �����
#define WAVE_PULSE 2 // ����
#define WAVE_TRIGANGLE 3 // ���ǲ�
#define WAVE_DC 4 // ֱ����


int GenerateWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle, int WaveType);
int GenerateSineWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle);
int GenerateRandWave(PUSHORT DataBuffer, LONG BufferSize);
int GeneratePulseWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle);
int TriangleWave(PUSHORT pBuffer, int PeakValue, LONG BufferSize, int Cycle);
int GenerateDCWave(PUSHORT DataBuffer, LONG BufferSize);
