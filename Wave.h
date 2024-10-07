// 波形类型
#define WAVE_SINE 0 // 正弦波
#define WAVE_RAND 1 // 随机波
#define WAVE_PULSE 2 // 方波
#define WAVE_TRIGANGLE 3 // 三角波
#define WAVE_DC 4 // 直流波


int GenerateWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle, int WaveType);
int GenerateSineWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle);
int GenerateRandWave(PUSHORT DataBuffer, LONG BufferSize);
int GeneratePulseWave(PUSHORT DataBuffer, LONG BufferSize, int Cycle);
int TriangleWave(PUSHORT pBuffer, int PeakValue, LONG BufferSize, int Cycle);
int GenerateDCWave(PUSHORT DataBuffer, LONG BufferSize);
