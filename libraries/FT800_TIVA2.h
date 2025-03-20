// This code is provided as an example only and is not guaranteed by FTDI. FTDI accept no responsibility for any 
// issues resulting from its use. The developer of the final application incorporating any parts of this sample project 
// is entirely responsible for ensuring its safe and correct operation and any consequences resulting from its use.

#define VM800B50


#define CMDBUF_SIZE             4096
#define CMD_APPEND              4294967070
#define CMD_BGCOLOR             4294967049
#define CMD_BITMAP_TRANSFORM    4294967073
#define CMD_BUTTON              4294967053
#define CMD_CALIBRATE           4294967061
#define CMD_CLOCK               4294967060
#define CMD_COLDSTART           4294967090
#define CMD_CRC                 4294967043
#define CMD_DIAL                4294967085
#define CMD_DLSTART             4294967040
#define CMD_EXECUTE             4294967047
#define CMD_FGCOLOR             4294967050
#define CMD_GAUGE               4294967059
#define CMD_GETMATRIX           4294967091
#define CMD_GETPOINT            4294967048
#define CMD_GETPROPS            4294967077
#define CMD_GETPTR              4294967075
#define CMD_GRADCOLOR           4294967092
#define CMD_GRADIENT            4294967051
#define CMD_HAMMERAUX           4294967044
#define CMD_IDCT                4294967046
#define CMD_INFLATE             4294967074
#define CMD_INTERRUPT           4294967042
#define CMD_KEYS                4294967054
#define CMD_LOADIDENTITY        4294967078
#define CMD_LOADIMAGE           4294967076
#define CMD_LOGO                4294967089
#define CMD_MARCH               4294967045
#define CMD_MEMCPY              4294967069
#define CMD_MEMCRC              4294967064
#define CMD_MEMSET              4294967067
#define CMD_MEMWRITE            4294967066
#define CMD_MEMZERO             4294967068
#define CMD_NUMBER              4294967086
#define CMD_PROGRESS            4294967055
#define CMD_REGREAD             4294967065
#define CMD_ROTATE              4294967081
#define CMD_SCALE               4294967080
#define CMD_SCREENSAVER         4294967087
#define CMD_SCROLLBAR           4294967057
#define CMD_SETFONT             4294967083
#define CMD_SETMATRIX           4294967082
#define CMD_SKETCH              4294967088
#define CMD_SLIDER              4294967056
#define CMD_SNAPSHOT            4294967071
#define CMD_SPINNER             4294967062
#define CMD_STOP                4294967063
#define CMD_SWAP                4294967041
#define CMD_TEXT                4294967052
#define CMD_TOGGLE              4294967058
#define CMD_TOUCH_TRANSFORM     4294967072
#define CMD_TRACK               4294967084
#define CMD_TRANSLATE           4294967079

/* Definition of parameter OPTION*/
#define OPT_3D                  0
#define OPT_565                 0
#define OPT_MONO                1
#define OPT_NODL                2
#define OPT_FLAT                256
#define OPT_SIGNED              256
#define OPT_CENTERX             512
#define OPT_CENTERY             1024
#define OPT_CENTER              1536
#define OPT_RIGHTX              2048
#define OPT_NOBACK              4096
#define OPT_NOTICKS             8192
#define OPT_NOHM                16384
#define OPT_NOPOINTER           16384
#define OPT_NOSECS              32768
#define OPT_NOHANDS             49152

/*Graphics primitive operation definition*/
#define CMD_BEGIN_BITMAPS       0x1F000001
#define CMD_BEGIN_POINTS 	    0x1F000002
#define CMD_BEGIN_LINES 	    0x1F000003
#define CMD_BEGIN_LINE_STRIP    0x1F000004
#define CMD_BEGIN_EDGE_STRIP_R  0x1F000005
#define CMD_BEGIN_EDGE_STRIP_L  0x1F000006
#define CMD_BEGIN_EDGE_STRIP_A  0x1F000007
#define CMD_BEGIN_EDGE_STRIP_B  0x1F000008
#define CMD_BEGIN_RECTS 	    0x1F000009
#define CMD_END                 0x21000000

#define CMD_DISPLAY             0x00000000
#define CMD_JUMP                0x1E000000
#define CMD_LINEWIDTH		    0x0E000000
#define CMD_POINT_SIZE          0x0D000000

#define RAM_CMD                 1081344
#define RAM_DL                  1048576
#define RAM_G                   0
#define RAM_PAL                 1056768
#define RAM_REG                 1057792


#define REG_ANALOG              1058104
#define REG_ANA_COMP            1058160
#define REG_BIST_CMD            1058124
#define REG_BIST_EN             1058132
#define REG_BIST_RESULT         1058128
#define REG_BUSYBITS            1058008
#define REG_CLOCK               1057800
#define REG_CMD_DL              1058028
#define REG_CMD_READ            1058020
#define REG_CMD_WRITE           1058024
#define REG_CPURESET            1057820
#define REG_CRC                 1058152
#define REG_CSPREAD             1057892
#define REG_CYA0                1058000
#define REG_CYA1                1058004
#define REG_CYA_TOUCH           1058100
#define REG_DATESTAMP           1058108
#define REG_DITHER              1057884
#define REG_DLSWAP              1057872
#define REG_FRAMES              1057796
#define REG_FREQUENCY           1057804
#define REG_GPIO                1057936
#define REG_GPIO_DIR            1057932
#define REG_HCYCLE              1057832
#define REG_HOFFSET             1057836
#define REG_HSIZE               1057840
#define REG_HSYNC0              1057844
#define REG_HSYNC1              1057848
#define REG_ID                  1057792
#define REG_INT_EN              1057948
#define REG_INT_FLAGS           1057944
#define REG_INT_MASK            1057952
#define REG_MACRO_0             1057992
#define REG_MACRO_1             1057996
#define REG_MARCH_ACC           1058144
#define REG_MARCH_DIR           1058136
#define REG_MARCH_OP            1058140
#define REG_MARCH_WIDTH         1058148
#define REG_OUTBITS             1057880
#define REG_PCLK                1057900
#define REG_PCLK_POL            1057896
#define REG_PLAY                1057928
#define REG_PLAYBACK_FORMAT     1057972
#define REG_PLAYBACK_FREQ       1057968
#define REG_PLAYBACK_LENGTH     1057960
#define REG_PLAYBACK_LOOP       1057976
#define REG_PLAYBACK_PLAY       1057980
#define REG_PLAYBACK_READPTR    1057964
#define REG_PLAYBACK_START      1057956
#define REG_PWM_DUTY            1057988
#define REG_PWM_HZ              1057984
#define REG_RENDERMODE          1057808
#define REG_ROMSUB_SEL          1058016
#define REG_ROTATE              1057876
#define REG_SNAPSHOT            1057816
#define REG_SNAPY               1057812
#define REG_SOUND               1057924
#define REG_SWIZZLE             1057888
#define REG_TAG                 1057912
#define REG_TAG_X               1057904
#define REG_TAG_Y               1057908
#define REG_TAP_CRC             1057824
#define REG_TAP_MASK            1057828
#define REG_TOUCH_ADC_MODE      1058036
#define REG_TOUCH_CHARGE        1058040
#define REG_TOUCH_DIRECT_XY     1058164
#define REG_TOUCH_DIRECT_Z1Z2   1058168
#define REG_TOUCH_MODE          1058032
#define REG_TOUCH_OVERSAMPLE    1058048
#define REG_TOUCH_RAW_XY        1058056
#define REG_TOUCH_RZ            1058060
#define REG_TOUCH_RZTHRESH      1058052
#define REG_TOUCH_SCREEN_XY     1058064
#define REG_TOUCH_SETTLE        1058044
#define REG_TOUCH_TAG           1058072
#define REG_TOUCH_TAG_XY        1058068
#define REG_TOUCH_TRANSFORM_A   1058076
#define REG_TOUCH_TRANSFORM_B   1058080
#define REG_TOUCH_TRANSFORM_C   1058084
#define REG_TOUCH_TRANSFORM_D   1058088
#define REG_TOUCH_TRANSFORM_E   1058092
#define REG_TOUCH_TRANSFORM_F   1058096
#define REG_TRACKER             1085440
#define REG_TRIM                1058156
#define REG_VCYCLE              1057852
#define REG_VOFFSET             1057856
#define REG_VOL_PB              1057916
#define REG_VOL_SOUND           1057920
#define REG_VSIZE               1057860
#define REG_VSYNC0              1057864
#define REG_VSYNC1              1057868

#define GRIS_CLARO              0xE0E0E0
#define GRIS_OSCURO             0x606060

#define S_TRIANG	            0x04
#define S_BEEP 		            0x05
#define S_ALARM 	            0x06
#define S_WARBLE 	            0x07
#define S_PIPS		            0x10
#define S_XILO		            0x41
#define S_PIANO		            0x46

#define N_DO 		            60
#define N_REB		            61
#define N_RE		            62
#define N_MIB		            63
#define N_MI		            64
#define N_FA		            65
#define N_SOLB		            66
#define N_SOL		            67
#define N_LAB		            68
#define N_LA		            69
#define N_SIB		            70
#define N_SI		            71

#if !defined(VM800B35) && !defined(VM800B43) && !defined(VM800B50)
#define VM800B35
#endif

#ifdef VM800B50
#define HCYCLE      548
#define HOFFSET     43
#define HSYNC0      0
#define HSYNC1      41
#define VCYCLE      292
#define VOFFSET     12
#define VSYNC0      0
#define VSYNC1      10
#define PCLK_POL    1
#define HSIZE       480
#define VSIZE       272
#define PCLK        5
#endif

#ifdef VM800B43
#define HCYCLE      548
#define HOFFSET     43
#define HSYNC0      0
#define HSYNC1      41
#define VCYCLE      292
#define VOFFSET     12
#define VSYNC0      0
#define VSYNC1      10
#define PCLK_POL    1
#define HSIZE       480
#define VSIZE       272
#define PCLK        5
#endif

#ifdef VM800B35
#define HCYCLE      408
#define HOFFSET     70
#define HSYNC0      0
#define HSYNC1      10
#define VCYCLE      263
#define VOFFSET     13
#define VSYNC0      0
#define VSYNC1      2
#define PCLK_POL    0
#define HSIZE       320
#define VSIZE       240
#define PCLK        5
#endif


#define 	FT_GPU_INTERNAL_OSC 0x48 //default
#define 	FT_GPU_EXTERNAL_OSC 0x44

#define 	FT_GPU_PLL_48M      0x62  //default
#define 	FT_GPU_PLL_36M      0x61
#define 	FT_GPU_PLL_24M      0x64

#define 	FT_GPU_ACTIVE_M     0x00
#define 	FT_GPU_STANDBY_M    0x41//default
#define 	FT_GPU_SLEEP_M      0x42
#define 	FT_GPU_POWERDOWN_M  0x50

#define     FT_GPU_CORE_RESET   0x68

#define dword long
#define byte char

/**Generic Functions**/
void Delay(void);

/********* Hardware Abstraction Layer *******
 * Hardware abstraction functions, specific to mcu used
 * Access to GPIO, SPI and configuration
 ********************************************/

void HAL_Init_SPI(uint8_t Port, uint32_t SYSCLOCK);
unsigned char HAL_SPI_ReadWrite(unsigned char);
void HAL_SPI_CSLow(void);
void HAL_SPI_CSHigh(void);
void HAL_SPI_PDLow(void);
void HAL_SPI_PDHigh(void);

/********* Low-level functions **********
 * Send read/write address
 *
 ********************************************/
void FT800_SPI_SendAddressWR(dword);
void FT800_SPI_SendAddressRD(dword);

/******* Next abstraction layer: data read/write
 * of 8, 16 or 32 bits, to/from any display internal register
 *
 ********************************************************/
void FT800_SPI_Write32(dword);
void FT800_SPI_Write16(unsigned int);
void FT800_SPI_Write8(byte);
long FT800_SPI_Read32(void);
char FT800_SPI_Read8(void);

void FT800_SPI_HostCommand(byte);
void FT800_SPI_HostCommandDummyRead(void);
unsigned int FT800_IncCMDOffset(unsigned int, byte);

void ComEsperaFin(void);
void WriteRam32(unsigned long data);
void WriteRam16(unsigned int data);
void WriteRam8(char data);
void WriteRamTxt(char* text);
void PadFIFO(void);

void Execute_Cmd_List(void);
void Draw(void);

uint32_t read32_reg(uint32_t addr);
uint8_t read8_reg(uint32_t addr);

void write32_reg(unsigned long addr, unsigned long value);

/******** High-level (top layer) functions:
 * Display initialization, general command sending, and 
 * specific commands high-level functions.
 *
 ***************************************************/
void DisplayInit(void);
void Command(unsigned long comm);
void ComVertex2ff(int x,int y);
void ComColor(int R, int G, int B);
void NewScreen(int R, int G, int B);
void ComLineWidth(float width);
void ComPointSize(int size);
void ComPoint(uint16_t x, uint16_t y, uint16_t R);
void ComTXT(int x, int y, int font, int ops, char *text);
void ComNum(int x, int y, int font, int ops, unsigned long num);
void ComTeclas(int x, int y, int w, int h, int font, unsigned int ops, char *keys);

void ReadTouchScreen(void);
void ComFgcolor(int R, int G, int B);
void ComBgcolor(int R, int G, int B);
void ComScrollbar(int x, int y, int w, int h, int ops, int val, int size, int range);
void ComButton(int x, int y, int w, int h, int font, int ops, char *cadena);
char Button(int x, int y, int w, int h, int font, char *text);
void ComGradient(int x0,int y0, long color0, int x1, int y1, int color1);

void ComGauge(int x, int y, int r, int ops, int major, int minor, int val, int range);
void ComToggle(int x, int y, int w, int font, int ops, int state, char *s);
void ComSlider(int x, int y, int w, int h, int ops, int val, int range);
void ComLine(int x1, int y1, int x2, int y2);

void WaitForTouch(void);
void CalibTouch(void);

void NoteVol(unsigned char volume);
void NotePlay( int instr, int note);
void NoteEnd(void);
void Fadeout(void);
void Fadein(void);
