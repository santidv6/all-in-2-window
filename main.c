#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/hibernate.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/i2c.h"

#include "utils/ustdlib.h"
#include "utils/uartstdio.h"

#include "FT800_TIVA2.h"
#include "iofuncs.h"

#include "HAL_I2C.h"
#include "sensorlib2.h"

#define FGDHH CMD_BEGIN_LINESTRIP

////  SYSTEM & GP PARAMETERS  ////
#define BP 1
#define MSEC 40000
#define PWM_FREQ 50         //FREQUENCY OF SERVOMOTOR PWM
#define POS_INI deg_to_cycles(10); //INITIAL POSITION OF THE SERVOMOTOR
#define DISPATT1_TIME  30   //seconds
#define DISPATT1_VALUE 50   //(0-100) and 128 is totally on
#define DISPATT2_TIME  (DISPATT1_TIME+5)   //seconds
#define DISPATT2_VALUE 20   //(0-100) and 128 is totally on
#define CLOCKSCR_TIME  (DISPATT2_TIME+5)   //seconds
#define CLOCKSCR_VALUE 20   //(0-100) and 128 is totally on
#define MINPLOT_TEMP 15     //REAL RANGE MIN -40 ºC
#define MAXPLOT_TEMP 34     //REAL RANGE MAX 85 ºC
#define MINPLOT_HUM 20      //REAL RANGE MIN 0 %RH
#define MAXPLOT_HUM 80      //REAL RANGE MAX 100 %RH
#define MINPLOT_LUM 0       //REAL RANGE MIN 0.01 lux
#define MAXPLOT_LUM 120     //REAL RANGE MAX 83865.60 lux
#define T_VECTORSIZE 60     //SAMPLES FOR PLOT
#define H_VECTORSIZE 60     //SAMPLES FOR PLOT
#define L_VECTORSIZE 60     //SAMPLES FOR PLOT
#define L_THRESHOLD 60      //LIGHT THRESHOLD
//////////////////////////////////

////  DISPLAY PARAMETERS  ////
//    COLORS     //
#define CLOCKSCR_R 0
#define CLOCKSCR_G 0
#define CLOCKSCR_B 0
#define MAINBG_R 220
#define MAINBG_G 120
#define MAINBG_B 40
#define LANGSELSCR_R 66
#define LANGSELSCR_G 84
#define LANGSELSCR_B 147
#define OBJECTBGGRAY 0x40
#define FONT1COLOR_R 0xFA
#define FONT1COLOR_G 0xFA
#define FONT1COLOR_B 0xFA
#define DISABLEDGRAY_R 0xBB
#define DISABLEDGRAY_G 0xBB
#define DISABLEDGRAY_B 0xBB
///////////////////
//   POSITIONS   //
#define DAYARROW_X HSIZE*3/16
#define MONTHARROW_X HSIZE*7/16
#define YEARARROW_X HSIZE*11/16
#define HOURARROW_X HSIZE*5/16
#define MINARROW_X HSIZE*9/16
#define DATEUPARROWS_Y VSIZE*2/16
#define DATEDOWNARROWS_Y VSIZE*7/16
#define TIMEUPARROWS_Y VSIZE*9/16
#define TIMEDOWNARROWS_Y VSIZE*14/16

#define MANCTLTOGLOC_X HSIZE*7/32
#define MANCTLTOGLOC_Y VSIZE*5/16
#define MANCTLTOGLOC_WD HSIZE*3/16
#define MANCTLTOGLOC_WBT HSIZE*6/16
#define MANCTLTOGLOC_HBT VSIZE*4/16

#define AVSLILOC_X HSIZE*4/32.0
#define AVSLILOC_Y VSIZE*17/32.0
#define AVSLILOC_WD HSIZE*0.9/32.0
#define AVSLILOC_HG VSIZE*16/32.0
#define AVSLILOC_WBS HSIZE*8/32.0
///////////////////
////////////////////////////////////

//////////////////////////////////////////////
////       SYSTEM & GENERAL PURPOSE       ////
////  Variable and Function Declarations  ////
//////////////////////////////////////////////
uint32_t ui32SysClock, SYSCLOCK;
uint8_t language=0;
bool lang_change=false;
uint8_t disp_state=0, manaccess_touch=0;
uint8_t EN_but, ES_but, SET_but, NOSET_but, BACK_but, DEFAULT_but;
uint8_t SETTINGS_but, MANUALCTL_but, PLOT_but;
uint8_t LANGSET_but, CLOCKSET_but, BACKLIGHTSET_but, CALIBRATE_but;
uint8_t TEMP_but, HUM_but, LUM_but;
bool openingmode_state=false,opening_state=false,transparencymode_state=false,transparency_state=false;
int8_t tempslider=0,humidslider=0,lightslider=0;
uint8_t backlight_lvl=100;
bool window_servo_state=false, window_esg_state=false;
uint8_t window_state=0, esg_state=0; //vars for the state machine that controls the opening and transparency state of the window
uint32_t PWMClock, PWMPeriod;
uint32_t pos;
uint8_t end_calib=0;
uint32_t ms_count=0;                        //variable for milliseconds counting by SysTick for checking time lapsed between events
uint32_t cur_t, prev_t=0;                   //current and previous milliseconds count
uint8_t powersavemode=0, actualscr_state;   //vars for attenuation-screens/backlight-setting machine state
bool first_clockscr=true;                   //variable for checking whether the clockscreen is first time displayed or not
bool change_to_settings=false;
float T_int,H_int,L_int;
float T_int_v[T_VECTORSIZE],H_int_v[H_VECTORSIZE],L_int_v[L_VECTORSIZE];    //ambient variables vectors for plotting
char auxstr1[20],auxstr2[20];               //auxiliar strings for different purposes
float T_ext, H_ext;

////////AUX TEST/////////////////////
char SampleBell=0;
char string[50];
uint8_t DevID=0, DevID77=0, DevID76=0;
uint8_t cod_err=0;

// BME280
int returnRslt;
int g_s32ActualTemp   = 0;
unsigned int g_u32ActualPress  = 0;
unsigned int g_u32ActualHumidity = 0;
struct bme280_t bme280;
uint8_t Bme_OK;
//OPT3001
uint8_t Opt_OK;

/////////////////////////////////////

// GENERAL ENUMS //
enum {
    ENGLISH,
    SPANISH
};
enum {
    AUTO,
    MANUAL
};
///////////////////
// STATE MACHINES ENUMS //
enum {
    LANGSELSCR,
    AMBVARSSCR,
    SETTINGSSCR,
    LANGSETSCR,
    CLOCKSETSCR,
    BACKLIGHTSETSCR,
    CALIBRATESCR,
    MANCTLSCR,
    PLOTSCR,
    PLOTSCR_T,
    PLOTSCR_H,
    PLOTSCR_L,
    CLOCKSCR
};
enum {
    SETDAYUP,
    SETDAYDOWN,
    SETMONTHUP,
    SETMONTHDOWN,
    SETYEARUP,
    SETYEARDOWN,
    SETHOURUP,
    SETHOURDOWN,
    SETMINUP,
    SETMINDOWN
};
enum {
    UNTOUCHED,
    OPENINGAUTO,
    OPENINGMANUAL,
    OPENINGCLOSE,
    OPENINGOPEN,
    TRANSPAUTO,
    TRANSPMANUAL,
    TRANSPOPAQUE,
    TRANSPTRANSP
};
enum {
    SLIDERHOT=1,
    SLIDERTIGNORE,
    SLIDERCOLD,
    SLIDERHUMID,
    SLIDERHIGNORE,
    SLIDERDRY,
    SLIDERBRIGHT,
    SLIDERLIGNORE,
    SLIDERDARK
};
enum {
    CLOSEWINDOW,
    OPENINGWINDOW,
    OPENWINDOW,
    CLOSINGWINDOW
};
enum {
    OPAQUEWINDOW,
    CLEARINGWINDOW,
    TRANSPARENTWINDOW,
    SHADINGWINDOW
};
enum {
    NORMAL,
    DIMMED1,
    DIMMED2,
    DIMMEDCLOCKSCR
};
//////////////////////////
//LANGUAGE DEPENDANT WITH OPTIONS//
static char *settings_txt[2][4]=
{
 "LANGUAGE","CLOCK","BACKLIGHT","CALIBRATE",
 "IDIOMA","RELOJ","BRILLO","CALIBRAR"
};
static char *calib_txt[2][5]=
{
 "Tap to continue","Tap on the circle","Repeat","Finish","Press to test",
 "Pulsa para continuar","Pulsa en el punto","Repetir","Terminar","Pulsa para comprobar"
};
static char *title[2][7]=
{
 "AMBIENT VARIABLES","SETTINGS","MANUAL CONTROL","TIME EVOLUTION","TEMPERATURE","HUMIDITY","LUMINOSITY",
 "VARIABLES AMBIENTALES","AJUSTES","CONTROL MANUAL","EVOLUCION TEMPORAL","TEMPERATURA","HUMEDAD","LUMINOSIDAD"
};
enum {
    AMBIENTTITLE,
    SETTINGSTITLE,
    MANCTLTITLE,
    PLOTTITLE,
    TEMPTITLE,
    HUMTITLE,
    LUMTITLE
};
static char *manualctlops_txt[2][2]=
{
 "OPENING","TRANSPARENCY",
 "APERTURA","TRANSPARENCIA"
};
enum {
    OPENINGTXT,
    TRANSPARENCYTXT
};
static char *ambientvars_txt[2][6]=
{
 "HOT","COLD","HUMID","DRY","BRIGHT","DARK",
 "CALIDO","FRIO","HUMEDO","SECO","LUMINOSO","TENUE"
};
enum {
    HOTTXT,
    COLDTXT,
    MOISTTXT,
    DRYTXT,
    BRIGHTTXT,
    DARKTXT
};
/////////////////////////////////////////////////////
//LANGUAGE DEPENDANT ONLY//
static char *lang_abrev[2]=
{
 "EN",
 "ES"
};
static char *set_txt[2]=
{
 "SET",
 "AJUSTAR"
};
static char *noset_txt[2]=
{
 "CANCEL",
 "CANCELAR"
};
static char *back_txt[2]=
{
 "BACK",
 "ATRÁS"
};
static char *default_txt[2]=
{
 "DEFAULT",
 "POR DEFECTO"
};
static char *backlightset_txt[2]=
{
 "BACKLIGHT LEVEL",
 "NIVEL DE BRILLO"
};
///////////////////////////
void open_window(void){
    int i;
    for(i=10; i<55; i++){
        pos=deg_to_cycles(i);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pos);
        SysCtlDelay(20*MSEC);
    }
    SysCtlDelay(100*MSEC);
}
void close_window(void){
    int i;
    for(i=55; i>10; i--){
        pos=deg_to_cycles(i);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pos);
        SysCtlDelay(10*MSEC);
    }
    SysCtlDelay(100*MSEC);
}
void clear_window(void){
    //GPIOLOW
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);
    SysCtlDelay(100*MSEC);
}
void white_window(void){
    //GPIOHIGH
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 1<<7);
    SysCtlDelay(100*MSEC);
}
void vector_null(char var){             //initialization of ambient vars vectors
    int i;
    switch(var){
    case 'T': for(i=0;i<T_VECTORSIZE;i++) {T_int_v[i]=0;} break;
    case 'H': for(i=0;i<H_VECTORSIZE;i++) {H_int_v[i]=0;} break;
    case 'L': for(i=0;i<L_VECTORSIZE;i++) {L_int_v[i]=0;} break;
    }
}
void vector_leftshift(char var){        //update (shift) of ambient vars vectors
    int i;
    switch(var){
    case 'T':
        for(i=0;i<T_VECTORSIZE-1;i++){
            T_int_v[i]=T_int_v[i+1];
        }
        T_int_v[T_VECTORSIZE-1]=T_int;
        break;
    case 'H':
        for(i=0;i<H_VECTORSIZE-1;i++){
            H_int_v[i]=H_int_v[i+1];
        }
        H_int_v[H_VECTORSIZE-1]=H_int;
        break;
    case 'L':
        for(i=0;i<L_VECTORSIZE-1;i++){
            L_int_v[i]=L_int_v[i+1];
        }
        L_int_v[L_VECTORSIZE-1]=L_int;
        break;
    }
}
////**************************************////

////////////////////////////////////////////////////
////  FT800 Variable and Function Declarations  ////
////////////////////////////////////////////////////
char chipid = 0;                            // Holds value of Chip ID read from the FT800
unsigned long cmdBufferRd = 0x00000000;     // Store the value read from the REG_CMD_READ register
unsigned long cmdBufferWr = 0x00000000;     // Store the value read from the REG_CMD_WRITE register
unsigned int t=0;

unsigned long POSX, POSY, BufferXY;
unsigned long PREVPOSX, PREVPOSY;
unsigned int CMD_Offset = 0;
#ifdef VM800B35
const unsigned long REG_CAL[6]={21959,177,4294145463,14,4294950369,16094853};   //3.5" display
#endif
#ifdef VM800B50
const long REG_CAL[6]={32209,57,-866231,-42,-19136,18385406};                //5.0" display
#endif

void brightness(uint8_t value)
{
    HAL_SPI_CSLow();
    FT800_SPI_SendAddressWR(REG_PWM_HZ);
    FT800_SPI_Write16(10000);
    HAL_SPI_CSHigh();
    SysCtlDelay(20*MSEC);

    if(value>128) value=128;

    HAL_SPI_CSLow();
    FT800_SPI_SendAddressWR(REG_PWM_DUTY);
    FT800_SPI_Write8(value);
    HAL_SPI_CSHigh();
    SysCtlDelay(20*MSEC);
}
void backlight_slider_check(void){
    if(POSX>=HSIZE/2-HSIZE*4/16-VSIZE/32 && POSX<HSIZE/2+HSIZE*4/16+VSIZE/32 && POSY>VSIZE*8/16-VSIZE/16 && POSY<VSIZE*8/16+VSIZE/16){
        backlight_lvl=map(POSX,(HSIZE/2-HSIZE*4/16-VSIZE/32),(HSIZE/2+HSIZE*4/16+VSIZE/32),20,128);
    }
}
void frame_create(int x1, int y1, int x2, int y2)
{
    Command(CMD_BEGIN_LINE_STRIP);
    ComVertex2ff(x1,y1);
    ComVertex2ff(x2,y1);
    ComVertex2ff(x2,y2);
    ComVertex2ff(x1,y2);
    ComVertex2ff(x1,y1);
    Command(CMD_END);
}
void triangle_create(int x, int y, int b, int h){
    Command(CMD_BEGIN_LINE_STRIP);
    ComVertex2ff(x-b/2.0,y+h/2.0);
    ComVertex2ff(x,y-h/2.0);
    ComVertex2ff(x+b/2.0,y+h/2.0);
    ComVertex2ff(x-b/2.0,y+h/2.0);
    Command(CMD_END);
}
void gear_create(int x, int y, int r){
    Command(CMD_BEGIN_LINE_STRIP);
    ComVertex2ff(x-r/4,y-r-1);
    ////////////////////////////////
    ComVertex2ff(x+r/4,y-r-1);
    ComVertex2ff(x+r/4,y-3*r/4);
    ComVertex2ff(x+1.5*r/4,y-2.5*r/4);
    ComVertex2ff(x+2.5*r/4,y-3.5*r/4);
    ComVertex2ff(x+3.5*r/4,y-2.5*r/4);
    ComVertex2ff(x+2.5*r/4,y-1.5*r/4);
    ComVertex2ff(x+3*r/4,y-r/4);
    ComVertex2ff(x+r+1,y-r/4);
    ////////////////////////////////
    ComVertex2ff(x+r+1,y+r/4);
    ComVertex2ff(x+3*r/4,y+r/4);
    ComVertex2ff(x+2.5*r/4,y+1.5*r/4);
    ComVertex2ff(x+3.5*r/4,y+2.5*r/4);
    ComVertex2ff(x+2.5*r/4,y+3.5*r/4);
    ComVertex2ff(x+1.5*r/4,y+2.5*r/4);
    ComVertex2ff(x+r/4,y+3*r/4);
    ComVertex2ff(x+r/4,y+r+1);
    /////////////////////////////////////
    ComVertex2ff(x-r/4,y+r+1);
    ComVertex2ff(x-r/4,y+3*r/4);
    ComVertex2ff(x-1.5*r/4,y+2.5*r/4);
    ComVertex2ff(x-2.5*r/4,y+3.5*r/4);
    ComVertex2ff(x-3.5*r/4,y+2.5*r/4);
    ComVertex2ff(x-2.5*r/4,y+1.5*r/4);
    ComVertex2ff(x-3*r/4,y+r/4);
    ComVertex2ff(x-r-1,y+r/4);
    //////////////////////////////////////
    ComVertex2ff(x-r-1,y-r/4);
    ComVertex2ff(x-3*r/4,y-r/4);
    ComVertex2ff(x-2.5*r/4,y-1.5*r/4);
    ComVertex2ff(x-3.5*r/4,y-2.5*r/4);
    ComVertex2ff(x-2.5*r/4,y-3.5*r/4);
    ComVertex2ff(x-1.5*r/4,y-2.5*r/4);
    ComVertex2ff(x-r/4,y-3*r/4);
    ComVertex2ff(x-r/4,y-r-1);
    Command(CMD_END);

    ComPoint(x,y,r/3);
    ComColor(MAINBG_R,MAINBG_G,MAINBG_B);
    ComPoint(x,y,r/3-2);
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
}
void accent_create(int x, int y, int font){
    Command(CMD_BEGIN_LINES);
    ComVertex2ff(x,y-font/4.0 -2);
    ComVertex2ff(x+font/8.0,y-font/4.0 -2 -font/16.0);
    Command(CMD_END);
}
void minitoggle_create(int x, int y, int w, int line_h){
    Command(CMD_BEGIN_LINES);
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    ComLineWidth(line_h);
    ComVertex2ff(x-w/2.0,y);
    ComVertex2ff(x+w/2.0,y);
    ComColor(MAINBG_R,MAINBG_G,MAINBG_B);
    ComLineWidth(line_h-1);
    ComVertex2ff(x-w/2.0,y);
    ComVertex2ff(x+w/2.0,y);
    Command(CMD_END);
    ComLineWidth(1);
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    ComPoint(x+w/2.0, y, line_h);
}
void axis_create(int x, int y, int w, int h){
    Command(CMD_BEGIN_LINE_STRIP);
    ComVertex2ff(x-w/2.0,y-h/2.0);
    ComVertex2ff(x-w/2.0,y+h/2.0);
    ComVertex2ff(x+w/2.0,y+h/2.0);
    Command(CMD_END);
}
void false_plot_create(x,y,w_axis,h_axis){
    Command(CMD_BEGIN_LINE_STRIP);
    ComVertex2ff(x-w_axis*3/6.0,y+h_axis*3/8.0);
    ComVertex2ff(x-w_axis*2/6.0,y+h_axis*1/8.0);
    ComVertex2ff(x-w_axis*1/6.0,y+h_axis*2/8.0);
    ComVertex2ff(x+w_axis*1/6.0,y-h_axis*2/8.0);
    ComVertex2ff(x+w_axis*2/6.0,y-h_axis*1/8.0);
    ComVertex2ff(x+w_axis*3/6.0,y-h_axis*3/8.0);
    Command(CMD_END);
}
void sliders_check(void){
    static uint8_t sliders_state=0;
    switch(sliders_state){
    case UNTOUCHED:
        if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERTIGNORE;
        else if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERHOT;
        else if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5 && PREVPOSY==0x8000)
            sliders_state=SLIDERCOLD;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERHIGNORE;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERHUMID;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5 && PREVPOSY==0x8000)
            sliders_state=SLIDERDRY;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERLIGNORE;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0 && PREVPOSY==0x8000)
            sliders_state=SLIDERBRIGHT;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5 && PREVPOSY==0x8000)
            sliders_state=SLIDERDARK;
        break;
    case SLIDERTIGNORE:
        tempslider=0;
        if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0){
            sliders_state=SLIDERHOT;
        }
        else if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5){
            sliders_state=SLIDERCOLD;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERHOT:
        tempslider=1;
        if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0){
            sliders_state=SLIDERTIGNORE;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERCOLD:
        tempslider=-1;
        if(POSX>=AVSLILOC_X-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0){
            sliders_state=SLIDERTIGNORE;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERHIGNORE:
        humidslider=0;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0){
            sliders_state=SLIDERHUMID;
        }
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5){
            sliders_state=SLIDERDRY;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERHUMID:
        humidslider=1;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0){
            sliders_state=SLIDERHIGNORE;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERDRY:
        humidslider=-1;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0){
            sliders_state=SLIDERHIGNORE;
        }
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERLIGNORE:
        lightslider=0;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/2.0-5 && POSY<AVSLILOC_Y-AVSLILOC_HG/4.0)
            sliders_state=SLIDERBRIGHT;
        else if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y+AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/2.0+5)
            sliders_state=SLIDERDARK;
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERBRIGHT:
        lightslider=1;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0)
            sliders_state=SLIDERLIGNORE;
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    case SLIDERDARK:
        lightslider=-1;
        if(POSX>=AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD && POSX<AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD && POSY>=AVSLILOC_Y-AVSLILOC_HG/4.0 && POSY<AVSLILOC_Y+AVSLILOC_HG/4.0)
            sliders_state=SLIDERLIGNORE;
        else if(POSY==0x8000)
            sliders_state=UNTOUCHED;
        break;
    }
}
void sliders_lines(void){
    ComLine(AVSLILOC_X-AVSLILOC_WD-10,AVSLILOC_Y,AVSLILOC_X-AVSLILOC_WD+3,AVSLILOC_Y);
    ComLine(AVSLILOC_X+AVSLILOC_WD-3,AVSLILOC_Y,AVSLILOC_X+AVSLILOC_WD+10,AVSLILOC_Y);
    ComLine(AVSLILOC_X-AVSLILOC_WD-4,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X-AVSLILOC_WD+3,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WD-3,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WD+4,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X-AVSLILOC_WD-4,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X-AVSLILOC_WD+3,AVSLILOC_Y+AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WD-3,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WD+4,AVSLILOC_Y+AVSLILOC_HG/2.0);

    ComLine(AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD-10,AVSLILOC_Y,AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD+3,AVSLILOC_Y);
    ComLine(AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD-3,AVSLILOC_Y,AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD+10,AVSLILOC_Y);
    ComLine(AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD-4,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD+3,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD-3,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD+4,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD-4,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD+3,AVSLILOC_Y+AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD-3,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS+AVSLILOC_WD+4,AVSLILOC_Y+AVSLILOC_HG/2.0);

    ComLine(AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD-10,AVSLILOC_Y,AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD+3,AVSLILOC_Y);
    ComLine(AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD-3,AVSLILOC_Y,AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD+10,AVSLILOC_Y);
    ComLine(AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD-4,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD+3,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD-3,AVSLILOC_Y-AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD+4,AVSLILOC_Y-AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD-4,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD+3,AVSLILOC_Y+AVSLILOC_HG/2.0);
    ComLine(AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD-3,AVSLILOC_Y+AVSLILOC_HG/2.0,AVSLILOC_X+AVSLILOC_WBS*2+AVSLILOC_WD+4,AVSLILOC_Y+AVSLILOC_HG/2.0);
}
void arrows_drawing(void){
    triangle_create(DAYARROW_X,DATEUPARROWS_Y,20,17);   //DAY
    triangle_create(DAYARROW_X,DATEDOWNARROWS_Y,20,-17);  //
    triangle_create(MONTHARROW_X,DATEUPARROWS_Y,20,17);   //MONTH
    triangle_create(MONTHARROW_X,DATEDOWNARROWS_Y,20,-17);  //
    triangle_create(YEARARROW_X,DATEUPARROWS_Y,20,17);  //YEAR
    triangle_create(YEARARROW_X,DATEDOWNARROWS_Y,20,-17); //
    triangle_create(HOURARROW_X,TIMEUPARROWS_Y,20,17);   //HOUR
    triangle_create(HOURARROW_X,TIMEDOWNARROWS_Y,20,-17); //
    triangle_create(MINARROW_X,TIMEUPARROWS_Y,20,17);   //MINUTE
    triangle_create(MINARROW_X,TIMEDOWNARROWS_Y,20,-17); //
}
uint8_t arrows_check(void){
    uint8_t ret=127;
    int w=20,h=17;
    if(POSX>DAYARROW_X-w && POSX<DAYARROW_X+w && POSY>DATEUPARROWS_Y-h && POSY<DATEUPARROWS_Y+h && PREVPOSY==0x8000)
        ret=SETDAYUP;
    else if(POSX>DAYARROW_X-w && POSX<(DAYARROW_X+w) && POSY>DATEDOWNARROWS_Y-h && POSY<(DATEDOWNARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETDAYDOWN;
    else if(POSX>MONTHARROW_X-w && POSX<(MONTHARROW_X+w) && POSY>DATEUPARROWS_Y-h && POSY<(DATEUPARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETMONTHUP;
    else if(POSX>MONTHARROW_X-w && POSX<(MONTHARROW_X+w) && POSY>DATEDOWNARROWS_Y-h && POSY<(DATEDOWNARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETMONTHDOWN;
    else if(POSX>YEARARROW_X-w && POSX<(YEARARROW_X+w) && POSY>DATEUPARROWS_Y-h && POSY<(DATEUPARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETYEARUP;
    else if(POSX>YEARARROW_X-w && POSX<(YEARARROW_X+w) && POSY>DATEDOWNARROWS_Y-h && POSY<(DATEDOWNARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETYEARDOWN;
    else if(POSX>HOURARROW_X-w && POSX<(HOURARROW_X+w) && POSY>TIMEUPARROWS_Y-h && POSY<(TIMEUPARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETHOURUP;
    else if(POSX>HOURARROW_X-w && POSX<(HOURARROW_X+w) && POSY>TIMEDOWNARROWS_Y-h && POSY<(TIMEDOWNARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETHOURDOWN;
    else if(POSX>MINARROW_X-w && POSX<(MINARROW_X+w) && POSY>TIMEUPARROWS_Y-h && POSY<(TIMEUPARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETMINUP;
    else if(POSX>MINARROW_X-w && POSX<(MINARROW_X+w) && POSY>TIMEDOWNARROWS_Y-h && POSY<(TIMEDOWNARROWS_Y+h) && PREVPOSY==0x8000)
        ret=SETMINDOWN;
    return ret;
}
uint8_t LANGSEL_button_touched(int x, int y, int w, int h, int font, char *cadena)
{
    uint8_t result;
    if(POSX>x && POSX<(x+w) && POSY>y && POSY<(y+h) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x,y,w,h,font,OPT_FLAT,cadena);
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x,y,w,h,font,OPT_FLAT," ");
        if(disp_state==LANGSETSCR)
            ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        else if(disp_state==LANGSELSCR)
            ComFgcolor(LANGSELSCR_R, LANGSELSCR_G, LANGSELSCR_B);
        ComButton(x+1,y+1,w-2,h-2,font,OPT_FLAT,cadena);
        result=0;
    }
    return result;
}
uint8_t SET_button_touched(int x, int y, int w, int h, int font, char *cadena)
{
    uint8_t result;
    if(POSX>x && POSX<(x+w) && POSY>y && POSY<(y+h) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x,y,w,h,font,OPT_FLAT,cadena);
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x,y,w,h,font,OPT_FLAT," ");
        if(disp_state==CLOCKSCR)
            ComFgcolor(0x00,0x00,0x00);
        else
            ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        ComButton(x+1,y+1,w-2,h-2,font,OPT_FLAT,cadena);
        result=0;
    }
    return result;
}
uint8_t SETTINGS_button_touched(int x, int y, int w, int h)
{
    uint8_t result;
    if(POSX>x-w/2.0 && POSX<(x+w/2.0) && POSY>y-h/2.0 && POSY<(y+h/2.0) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT,"");
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT," ");
        ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        ComButton(x-w/2.0+2,y-h/2.0+1,w-2,h-2,26,OPT_FLAT,"");
        result=0;
    }
    ComLineWidth(0.8);
    gear_create(x,y,w/3.0);
    ComLineWidth(1);
    if(result){
        ComColor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
        ComPoint(x,y,(w/3.0)/3.0-2);
        ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    }
    return result;
}
uint8_t MANCTL_button_touched(int x, int y, int w, int h)
{
    uint8_t result;
    int w_toggle=w-HSIZE/32-2;
    if(POSX>x-w/2.0 && POSX<(x+w/2.0) && POSY>y-h/2.0 && POSY<(y+h/2.0) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT,"");
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT," ");
        ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        ComButton(x-w/2.0+2,y-h/2.0+1,w-2,h-2,26,OPT_FLAT,"");
        result=0;
    }
    minitoggle_create(x,y,w_toggle,h/8.0);
    ComTXT(x,y-VSIZE*1.5/32, 26, OPT_CENTER,"AUTO");
    ComTXT(x,y+VSIZE*1.5/32+1, 26, OPT_CENTER,"MAN");
    if(result){
        ComColor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
        Command(CMD_BEGIN_LINES);
        ComLineWidth(4);
        ComVertex2ff(x-w_toggle/2.0+1,y);
        ComVertex2ff(x+w_toggle/2.0-1,y);
        Command(CMD_END);
        ComLineWidth(1);
        ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComPoint(x+w_toggle/2.0, y, 5);
    }
    return result;
}
uint8_t PLOT_button_touched(int x, int y, int w, int h)
{
    uint8_t result;
    int w_axis=w-HSIZE/32-2;
    if(POSX>x-w/2.0 && POSX<(x+w/2.0) && POSY>y-h/2.0 && POSY<(y+h/2.0) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT,"");
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x-w/2.0+1,y-h/2.0,w,h,26,OPT_FLAT," ");
        ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        ComButton(x-w/2.0+2,y-h/2.0+1,w-2,h-2,26,OPT_FLAT,"");
        result=0;
    }
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    ComLineWidth(0.8);
    axis_create(x,y,w_axis,w_axis);
    false_plot_create(x,y,w_axis,w_axis);
    ComLineWidth(1);
    return result;
}
uint8_t SETTINGOPT_button_touched(int x, int y, int w, int h, int font, char *cadena)
{
    uint8_t result;
    if(POSX>x && POSX<(x+w) && POSY>y && POSY<(y+h) && PREVPOSY==0x8000){
        ComFgcolor(OBJECTBGGRAY, OBJECTBGGRAY, OBJECTBGGRAY);
        ComButton(x,y,w,h,font,OPT_FLAT,cadena);
        result=1;
    }
    else{
        ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
        ComButton(x,y,w,h,font,OPT_FLAT," ");
        ComFgcolor(MAINBG_R, MAINBG_G, MAINBG_B);
        ComButton(x+1,y+1,w-2,h-2,font,OPT_FLAT,cadena);
        result=0;
    }
    return result;
}
////********************************************////

///////////////////////////////////////////////////////////////
////  Hibernate Module Variable and Function Declarations  ////
///////////////////////////////////////////////////////////////

static char *ppcMonth[2][12] =
{
 "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" ,
 "Ene", "Feb", "Mar", "Abr", "May", "Jun", "Jul", "Ago", "Sep", "Oct", "Nov", "Dic"
};
// Flag that informs that date and time have to be set.
volatile bool setDate=false;
// Buffers to store display information.
char WakeBuf[40], HibBuf[40], DateBuf[20], TimeBuf[20];
bool RTC_updated, RTC_setted;

// Variables that keep track of the date and time.
uint32_t g_ui32MonthIdx, g_ui32DayIdx, g_ui32YearIdx;
uint32_t g_ui32HourIdx, g_ui32MinIdx, g_ui32SecIdx;

bool DateTimeGet(struct tm *sTime)
{
    // Get the latest time.
    HibernateCalendarGet(sTime);
    // Is valid data read?
    if(((sTime->tm_sec < 0) || (sTime->tm_sec > 59)) ||
            ((sTime->tm_min < 0) || (sTime->tm_min > 59)) ||
            ((sTime->tm_hour < 0) || (sTime->tm_hour > 23)) ||
            ((sTime->tm_mday < 1) || (sTime->tm_mday > 31)) ||
            ((sTime->tm_mon < 0) || (sTime->tm_mon > 11)) ||
            ((sTime->tm_year < 100) || (sTime->tm_year > 199)))
    {
        // No - Let the application know the same by returning relevant
        // message.
        return false;
    }
    // Return that new data is available so that it can be displayed.
    return true;
}
bool DateTimeDisplayGet(char *pcDBuf, char *pcTBuf, uint32_t ui32DBufSize, uint32_t ui32TBufSize, bool rtc_setted)
{
    static uint32_t ui32prevMinutes = 0xFF;
    struct tm sTime;
    uint32_t ui32DLen,ui32TLen;
    // Get the latest date and time and check the validity.
    if(DateTimeGet(&sTime) == false)
    {
        // Invalid - Force set the date and time to default values and return
        // false to indicate no information to display.
        setDate = true;
        return false;
    }
    // If date and time is valid, check if seconds have updated from previous
    // visit.
    if(ui32prevMinutes == sTime.tm_min && !rtc_setted && !lang_change)
    {
        // No information to display.
        return false;
    }
    // If valid new date and time is available, update a local variable to keep
    // track of minutes to determine new data for next visit.
    ui32prevMinutes = sTime.tm_min;
    // Format the date and time into a user readable format.
    ui32DLen = usnprintf(pcDBuf, ui32DBufSize, "%02u %s 20%02u", sTime.tm_mday, ppcMonth[language][sTime.tm_mon], sTime.tm_year - 100);
    ui32TLen = usnprintf(pcTBuf, ui32TBufSize, "%02u : %02u", sTime.tm_hour, sTime.tm_min);
    lang_change=false;
    // Return true to indicate new information to display.
    return true;
}
void DateTimeSet(void)
{
    struct tm sTime;
    // Get the latest date and time.  This is done here so that unchanged
    // parts of date and time can be written back as is.
    HibernateCalendarGet(&sTime);
    // Set the date and time values that are to be updated.
    sTime.tm_hour = g_ui32HourIdx;
    sTime.tm_min = g_ui32MinIdx;
    sTime.tm_sec = g_ui32SecIdx;
    sTime.tm_mon = g_ui32MonthIdx;
    sTime.tm_mday = g_ui32DayIdx;
    sTime.tm_year = 100 + g_ui32YearIdx;
    // Update the calendar logic of hibernation module with the requested data.
    HibernateCalendarSet(&sTime);
}
void DateTimeDefaultSet(void)
{
    g_ui32DayIdx = 1;
    g_ui32MonthIdx = 0;
    g_ui32YearIdx = 19;
    g_ui32HourIdx = 10;
    g_ui32MinIdx = 0;
    g_ui32SecIdx = 0;
}
bool DateTimeUpdateGet(void)
{
    struct tm sTime;

    // Get the latest date and time and check the validity.
    if(DateTimeGet(&sTime) == false)
    {
        // Invalid - Return here with false as no information to update.  So
        // use default values.
        DateTimeDefaultSet();
        return false;
    }
    // If date and time is valid, copy the date and time values into respective
    // indexes.
    g_ui32MonthIdx = sTime.tm_mon;
    g_ui32DayIdx = sTime.tm_mday;
    g_ui32YearIdx = sTime.tm_year - 100;
    g_ui32HourIdx = sTime.tm_hour;
    g_ui32MinIdx = sTime.tm_min;
    // Return true to indicate new information has been updated.
    return true;
}
uint32_t GetDaysInMonth(uint32_t ui32Year, uint32_t ui32Mon)
{
    // Return the number of days based on the month.
    if(ui32Mon == 1)
    {
        // For February return the number of days based on the year being a
        // leap year or not.
        if((ui32Year % 4) == 0)
        {
            // If leap year return 29.
            return 29;
        }
        else
        {
            // If not leap year return 28.
            return 28;
        }
    }
    else if((ui32Mon == 3) || (ui32Mon == 5) || (ui32Mon == 8) ||
            (ui32Mon == 10))
    {
        // For April, June, September and November return 30.
        return 30;
    }
    // For all the other months return 31.
    return 31;
}
////*******************************************************////

void lang_date_time_frame(void){
    ComTXT(HSIZE/32,VSIZE*1.5/32,26,OPT_CENTER,lang_abrev[language]);
    DateTimeDisplayGet(DateBuf, TimeBuf, sizeof(DateBuf), sizeof(TimeBuf),RTC_setted);
    ComTXT(HSIZE*25/32,VSIZE*1.5/32,26,OPT_CENTER,DateBuf);
    ComTXT(HSIZE*30/32,VSIZE*1.5/32, 26, OPT_CENTER,TimeBuf);
    frame_create(HSIZE*0.5/32,VSIZE*3/32,HSIZE*31.5/32,VSIZE*31/32);
}

////  Interrupt handler prototypes  ////
void SysTickIntHandler(void);

////////////////////////////////////////
uint8_t Test_I2C_dir(uint32_t I2C_Base, uint8_t dir);

//#define DEBUG //uncomment this line for debug purposes and serial comm. feedback
int main(void)
{
    uint8_t i;
    char auxstr0[5]; //auxiliar string for axis numbers on plot screens
    char dayBuf[3],monthnameBuf[4],yearBuf[5],hourBuf[3],minBuf[3];
    ui32SysClock=SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    SYSCLOCK=ui32SysClock;

    Conf_Boosterpack(BP, ui32SysClock);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 1<<7);

    ////Hibernate Module Configuration////
    HibernateEnableExpClk(ui32SysClock);
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
    HibernateCounterMode(HIBERNATE_COUNTER_24HR);
    HibernateRTCTrimSet(0x8060);
    ////PWM Configuration////
    PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_64);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMClock = ui32SysClock/64; //1875000
    PWMPeriod = PWMClock/PWM_FREQ - 1;   //37499
    pos=POS_INI;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWMPeriod);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pos);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    DateTimeUpdateGet();
    HAL_Init_SPI(1, ui32SysClock);
    ////SysTick Timer Configuration////
    SysTickPeriodSet(ui32SysClock/1000);
    SysTickIntRegister(SysTickIntHandler);
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, ui32SysClock);

#ifdef DEBUG
    UARTprintf("Inicializando los BME280... ");
#endif
    cod_err=Test_I2C_dir(BP, BME280_I2C_ADDRESS1)+Test_I2C_dir(BP,BME280_I2C_ADDRESS2);
    if(cod_err)
    {
#ifdef DEBUG
        UARTprintf("Error 0X%x en BME280\n", cod_err);
#endif
        Bme_OK=0;
    }
    else
    {
#ifdef DEBUG
        UARTprintf("NO COD_ERR. ALL FINE MAN :)");
#endif
        bme280_data_readout_template();
        bme280_set_oversamp_humidity(BME280_OVERSAMP_8X);
        bme280_set_filter(BME280_FILTER_COEFF_4);

        bme280.dev_addr=0x76;
        bme280_init(&bme280);
        bme280_set_power_mode(BME280_NORMAL_MODE);
        bme280_set_oversamp_humidity(BME280_OVERSAMP_8X);
        bme280_set_oversamp_pressure(BME280_OVERSAMP_2X);
        bme280_set_oversamp_temperature(BME280_OVERSAMP_4X);
        bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
        bme280_set_filter(BME280_FILTER_COEFF_4);

#ifdef DEBUG
        UARTprintf("Hecho! \nLeyendo DevIDs... ");
#endif
        readI2C(BME280_I2C_ADDRESS2,BME280_CHIP_ID_REG, &DevID77, 1);
        readI2C(BME280_I2C_ADDRESS1,BME280_CHIP_ID_REG, &DevID76, 1);
#ifdef DEBUG
        UARTprintf("DevID77= 0X%x \n", DevID77);
        UARTprintf("DevID76= 0X%x \n", DevID76);
#endif
        Bme_OK=1;
    }
#ifdef DEBUG
    UARTprintf("Inicializando OPT3001... ");
#endif
    cod_err=Test_I2C_dir(BP, OPT3001_SLAVE_ADDRESS);
    if(cod_err)
    {
#ifdef DEBUG
        UARTprintf("Error 0X%x en OPT3001\n", cod_err);
#endif
        Opt_OK=0;
    }
    else
    {
        OPT3001_init();
#ifdef DEBUG
        UARTprintf("Hecho!\n");
        UARTprintf("Leyendo DevID... ");
#endif
        DevID=OPT3001_readDeviceId();
#ifdef DEBUG
        UARTprintf("DevID= 0X%x \n", DevID);
#endif
        Opt_OK=1;
    }
    vector_null('T');
    vector_null('H');
    vector_null('L');

    DisplayInit();
    SysCtlDelay(ui32SysClock/3);
    brightness(backlight_lvl);

    ////////////////// LAUNCH SCREEN //////////////////
    NewScreen(LANGSELSCR_R,LANGSELSCR_G,LANGSELSCR_B);
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    ComLineWidth(1);
    frame_create(HSIZE*1/16,VSIZE*2/16,HSIZE*15/16,VSIZE*14/16);
    ComTXT(HSIZE/2,VSIZE*5/16, 28, OPT_CENTER,"Initializing...");
    Draw();
    SysCtlDelay(1000*MSEC);

    NewScreen(LANGSELSCR_R,LANGSELSCR_G,LANGSELSCR_B);
    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
    ComLineWidth(1);
    frame_create(HSIZE*1/16,VSIZE*2/16,HSIZE*15/16,VSIZE*14/16);
    ComTXT(HSIZE/2,VSIZE*5/16, 28, OPT_CENTER,"Ready");
    ComTXT(HSIZE/2,VSIZE*9/16, 28, OPT_CENTER,"Tap to continue");
    Draw();
    WaitForTouch();
    /////////////////////////////////////////////////////////
    for(i=0;i<6;i++)    write32_reg(REG_TOUCH_TRANSFORM_A+4*i, REG_CAL[i]);

    while(1)
    {
        if(SampleBell==1){
            SampleBell=0;
            if(Bme_OK)
            {
                bme280.dev_addr=BME280_I2C_ADDRESS2;
                returnRslt = bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumidity);
                T_int=(float)g_s32ActualTemp/100.0;
                H_int=(float)g_u32ActualHumidity/1000.0;
                bme280.dev_addr=BME280_I2C_ADDRESS1;
                returnRslt = bme280_read_pressure_temperature_humidity(&g_u32ActualPress, &g_s32ActualTemp, &g_u32ActualHumidity);
                T_ext=(float)g_s32ActualTemp/100.0;
                H_ext=(float)g_u32ActualHumidity/1000.0;

#ifdef DEBUG
                UARTprintf("-------------------------------------------------\n");
                sprintf(string, " T:%.2f degC\t H:%.2f o/oRH\t",T_int,H_int);
                UARTprintf(string);
#endif
            }
            vector_leftshift('T');
            vector_leftshift('H');
            if(Opt_OK)
            {
                L_int=OPT3001_getLux();
#ifdef DEBUG
                sprintf(string," L: %.3f Lux\n",L_int);
                UARTprintf(string);
                UARTprintf("-------------------------------------------------\n");
#endif
            }
            vector_leftshift('L');
        }


        ReadTouchScreen();
        if(POSY!=0x8000 || PREVPOSY!=0x8000)
            prev_t=ms_count;

        switch(powersavemode){
        case NORMAL:
            brightness(backlight_lvl);
            if(ms_count-prev_t>=DISPATT1_TIME*1000)
                powersavemode=DIMMED1;
            break;
        case DIMMED1:
            if(backlight_lvl>DISPATT1_VALUE)
                brightness(DISPATT1_VALUE);
            if(ms_count-prev_t>=DISPATT2_TIME*1000)
                powersavemode=DIMMED2;
            if(POSY!=0x8000)
                powersavemode=NORMAL;
            break;
        case DIMMED2:
            if(backlight_lvl>DISPATT2_VALUE)
                brightness(DISPATT2_VALUE);
            if(ms_count-prev_t>=CLOCKSCR_TIME*1000)
                powersavemode=DIMMEDCLOCKSCR;
            if(POSY!=0x8000)
                powersavemode=NORMAL;
            break;
        case DIMMEDCLOCKSCR:
            brightness(CLOCKSCR_VALUE);
            if(disp_state!=CLOCKSCR){
                actualscr_state=disp_state;
                disp_state=CLOCKSCR;
                first_clockscr=true;
            }
            if(POSY!=0x8000)
                powersavemode=NORMAL;
            break;
        }

        switch(disp_state){

        case LANGSELSCR:
            NewScreen(LANGSELSCR_R,LANGSELSCR_G,LANGSELSCR_B);
            ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
            ComTXT(HSIZE/2,VSIZE*5/16, 27, OPT_CENTER," SELECT LANGUAGE / SELECCIONE IDIOMA ");
            ComLineWidth(1);
            frame_create(HSIZE*1/16,VSIZE*2/16,HSIZE*15/16,VSIZE*14/16);
            EN_but=LANGSEL_button_touched(60,160,120,40,27,"ENGLISH");
            ES_but=LANGSEL_button_touched(300,160,120,40,27,"ESPANOL");
            ComTXT(360+10,180-10, 27, OPT_CENTER,"~");
            Draw();

            if(EN_but){
                language=ENGLISH;
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=AMBVARSSCR;
            }
            else if(ES_but){
                language=SPANISH;
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=AMBVARSSCR;
            }
            prev_t=ms_count;
            break;

        case AMBVARSSCR:
            NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
            ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
            lang_date_time_frame();
            ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][AMBIENTTITLE]);

            SETTINGS_but=SETTINGS_button_touched(HSIZE*29.5/32,VSIZE*6.5/32,HSIZE*3/32,VSIZE*5/32);
            MANUALCTL_but=MANCTL_button_touched(HSIZE*29.5/32,VSIZE*12.5/32,HSIZE*3/32,VSIZE*5/32);
            PLOT_but=PLOT_button_touched(HSIZE*29.5/32,VSIZE*18.5/32,HSIZE*3/32,VSIZE*5/32);

            ComTXT(HSIZE*2/16,VSIZE*6/32, 27, OPT_CENTER,ambientvars_txt[language][HOTTXT]);
            ComTXT(HSIZE*2/16,VSIZE*28/32, 27, OPT_CENTER,ambientvars_txt[language][COLDTXT]);
            ComTXT(HSIZE*6/16,VSIZE*6/32, 27, OPT_CENTER,ambientvars_txt[language][MOISTTXT]);
            ComTXT(HSIZE*6/16,VSIZE*28/32, 27, OPT_CENTER,ambientvars_txt[language][DRYTXT]);
            ComTXT(HSIZE*10/16,VSIZE*6/32, 27, OPT_CENTER,ambientvars_txt[language][BRIGHTTXT]);
            ComTXT(HSIZE*10/16,VSIZE*28/32, 27, OPT_CENTER,ambientvars_txt[language][DARKTXT]);
            if(language==SPANISH){
                ComLineWidth(0.75);
                accent_create(HSIZE*3/32 +2,VSIZE*6/32,27);
                accent_create(HSIZE*4/32 +4,VSIZE*28/32,27);
                accent_create(HSIZE*11/32 -3,VSIZE*6/32,27);
                ComLineWidth(1);
            }

            ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
            ComColor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
            ComBgcolor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
            sliders_check();
            sliders_lines();
            ComSlider(AVSLILOC_X-AVSLILOC_WD/2.0, AVSLILOC_Y-AVSLILOC_HG/2.0, AVSLILOC_WD, AVSLILOC_HG, OPT_FLAT, -tempslider+1, 2);
            ComSlider(AVSLILOC_X+AVSLILOC_WBS-AVSLILOC_WD/2.0, AVSLILOC_Y-AVSLILOC_HG/2.0, AVSLILOC_WD, AVSLILOC_HG, OPT_FLAT, -humidslider+1, 2);
            ComSlider(AVSLILOC_X+AVSLILOC_WBS*2-AVSLILOC_WD/2.0, AVSLILOC_Y-AVSLILOC_HG/2.0, AVSLILOC_WD, AVSLILOC_HG, OPT_FLAT, -lightslider+1, 2);

            Draw();

            if(SETTINGS_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=SETTINGSSCR;
                prev_t=ms_count;
            }
            if(MANUALCTL_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=MANCTLSCR;
                prev_t=ms_count;
            }
            if(PLOT_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=PLOTSCR;
                prev_t=ms_count;
            }
            DateTimeUpdateGet();
            break;

        case SETTINGSSCR:
            NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
            ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
            lang_date_time_frame();
            ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][SETTINGSTITLE]);

            LANGSET_but=SETTINGOPT_button_touched(HSIZE*3/16,VSIZE*5/32, HSIZE*10/16, VSIZE*2/16, 28, settings_txt[language][0]);
            CLOCKSET_but=SETTINGOPT_button_touched(HSIZE*3/16,VSIZE*10/32, HSIZE*10/16, VSIZE*2/16, 28, settings_txt[language][1]);
            BACKLIGHTSET_but=SETTINGOPT_button_touched(HSIZE*3/16,VSIZE*15/32, HSIZE*10/16, VSIZE*2/16, 28, settings_txt[language][2]);
            CALIBRATE_but=SETTINGOPT_button_touched(HSIZE*3/16,VSIZE*20/32, HSIZE*10/16, VSIZE*2/16, 28, settings_txt[language][3]);
            BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*13/16,HSIZE*5/32,VSIZE*2/16,26,back_txt[language]);
            if(language==SPANISH){
                ComLineWidth(0.75);
                accent_create(HSIZE*4/32 +2,VSIZE*14/16 +1,26);
                ComLineWidth(1);
            }
            Draw();

            if(LANGSET_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=LANGSETSCR;
                prev_t=ms_count;
            }
            else if(CLOCKSET_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=CLOCKSETSCR;
                prev_t=ms_count;
            }
            else if(BACKLIGHTSET_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=BACKLIGHTSETSCR;
                prev_t=ms_count;
            }
            else if(CALIBRATE_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=CALIBRATESCR;
                prev_t=ms_count;
            }
            else if(BACK_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=AMBVARSSCR;
                prev_t=ms_count;
            }
            DateTimeUpdateGet();
            break;

        case LANGSETSCR:
            NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
            ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);

            frame_create(HSIZE*1/16,VSIZE*2/16,HSIZE*15/16,VSIZE*14/16);
            ComTXT(HSIZE/2,VSIZE*5/16, 27, OPT_CENTER," SELECT LANGUAGE / SELECCIONE IDIOMA ");
            EN_but=LANGSEL_button_touched(60,160,120,40,27,"ENGLISH");
            ES_but=LANGSEL_button_touched(300,160,120,40,27,"ESPANOL");
            ComTXT(360+10,180-10, 27, OPT_CENTER,"~");

            Draw();

            if(EN_but){
                if(language!=ENGLISH)
                    lang_change=true;
                language=ENGLISH;
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=SETTINGSSCR;
                prev_t=ms_count;
            }
            else if(ES_but){
                if(language!=SPANISH)
                    lang_change=true;
                language=SPANISH;
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=SETTINGSSCR;
                prev_t=ms_count;
            }
            break;

        case CLOCKSETSCR:
            NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
            ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
            arrows_drawing();

            // ARROWS TOUCH CHECK //
            switch(arrows_check()){
            case SETDAYUP:
                if(g_ui32DayIdx<31) g_ui32DayIdx++; break;
            case SETDAYDOWN:
                if(g_ui32DayIdx>1) g_ui32DayIdx--; break;
            case SETMONTHUP:
                if(g_ui32MonthIdx<11) g_ui32MonthIdx++; break;
            case SETMONTHDOWN:
                if(g_ui32MonthIdx>0) g_ui32MonthIdx--; break;
            case SETYEARUP:
                if(g_ui32YearIdx<99) g_ui32YearIdx++; break;
            case SETYEARDOWN:
                if(g_ui32YearIdx>0) g_ui32YearIdx--; break;
            case SETHOURUP:
                if(g_ui32HourIdx<23) g_ui32HourIdx++; break;
            case SETHOURDOWN:
                if(g_ui32HourIdx>0) g_ui32HourIdx--; break;
            case SETMINUP:
                if(g_ui32MinIdx<59) g_ui32MinIdx++; break;
            case SETMINDOWN:
                if(g_ui32MinIdx>0) g_ui32MinIdx--; break;
            }
            ///////////////////////
            // DATE AND TIME CHARACTERS PRINT //
            usnprintf(dayBuf, 3, "%02u", g_ui32DayIdx);
            usnprintf(monthnameBuf, 4, "%s", ppcMonth[language][g_ui32MonthIdx]);
            usnprintf(yearBuf, 5, "20%02u", g_ui32YearIdx);

            ComTXT(HSIZE*3/16,VSIZE*4.5/16,30,OPT_CENTER,dayBuf);
            ComTXT(HSIZE*5/16,VSIZE*4.5/16,30,OPT_CENTER,"/");
            ComTXT(HSIZE*7/16,VSIZE*4.5/16,30,OPT_CENTER,monthnameBuf);
            ComTXT(HSIZE*9/16,VSIZE*4.5/16,30,OPT_CENTER,"/");
            ComTXT(HSIZE*11/16,VSIZE*4.5/16,30,OPT_CENTER,yearBuf);

            usnprintf(hourBuf, 3, "%02u", g_ui32HourIdx);
            usnprintf(minBuf, 3, "%02u", g_ui32MinIdx);

            ComTXT(HSIZE*5/16,VSIZE*11.5/16,30,OPT_CENTER,hourBuf);
            ComTXT(HSIZE*7/16,VSIZE*11.5/16,30,OPT_CENTER,":");
            ComTXT(HSIZE*9/16,VSIZE*11.5/16,30,OPT_CENTER,minBuf);
            ////////////////////////////////////
            SET_but=SET_button_touched(HSIZE*13/16,VSIZE*13/16,HSIZE*5/32,VSIZE*2/16,26,set_txt[language]);
            NOSET_but=SET_button_touched(HSIZE*13/16,VSIZE*10/16,HSIZE*5/32,VSIZE*2/16,26,noset_txt[language]);

            Draw();

            if(SET_but){
                DateTimeSet();
                RTC_setted=true;
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=SETTINGSSCR;
                prev_t=ms_count;
            }
            else if(NOSET_but){
                while(POSY!=0x8000){ReadTouchScreen();}
                disp_state=SETTINGSSCR;
                prev_t=ms_count;
            }
            break;

            case BACKLIGHTSETSCR:
                NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);

                ComTXT(HSIZE/2,VSIZE*5/16, 27, OPT_CENTER,backlightset_txt[language]);

                ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                ComColor(0xCC,0x60,0x50);
                ComBgcolor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
                backlight_slider_check();
                ComSlider(HSIZE/2-HSIZE*4/16, VSIZE*8/16-VSIZE/32, HSIZE*8/16, VSIZE*1/16, OPT_FLAT, backlight_lvl-20, 108);
                ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);

                BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*13/16,HSIZE*5/32,VSIZE*2/16,26,back_txt[language]);
                if(language==SPANISH){
                    ComLineWidth(0.75);
                    accent_create(HSIZE*4/32 +2,VSIZE*14/16 +1,26);
                    ComLineWidth(1);
                }
                DEFAULT_but=SET_button_touched(HSIZE*24/32,VSIZE*13/16,HSIZE*7/32,VSIZE*2/16,26,default_txt[language]);

                Draw();

                if(DEFAULT_but){
                    while(POSY!=0x8000){ReadTouchScreen();}
                    backlight_lvl=100;
                    prev_t=ms_count;
                }
                else if(BACK_but){
                    while(POSY!=0x8000){ReadTouchScreen();}
                    disp_state=SETTINGSSCR;
                    prev_t=ms_count;
                }
                break;

            case CALIBRATESCR:
                end_calib=0;
                NewScreen(LANGSELSCR_R,LANGSELSCR_G,LANGSELSCR_B);
                ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                ComLineWidth(1);
                frame_create(HSIZE*1/16,VSIZE*2/16,HSIZE*15/16,VSIZE*14/16);
                ComTXT(HSIZE/2,VSIZE*8/16, 28, OPT_CENTER,calib_txt[language][0]);
                Draw();
                do{
                    ReadTouchScreen();
                }
                while(POSY!=0x8000);
                WaitForTouch();
                SysCtlDelay(SYSCLOCK/6);

                NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                ComTXT(HSIZE/2,VSIZE/2,28,OPT_CENTER,calib_txt[language][1]);

                WriteRam32(CMD_CALIBRATE);
                Draw();
                ComEsperaFin();
                while(!end_calib){
                    NewScreen(LANGSELSCR_R,LANGSELSCR_G,LANGSELSCR_B);
                    ComLine(40,0,40,VSIZE);
                    ComTXT(40+HSIZE/32, VSIZE*15/16, 27, OPT_CENTERY, calib_txt[language][2]);
                    ComLine(HSIZE-40,0,HSIZE-40,VSIZE);
                    ComTXT(HSIZE-40-HSIZE/32, VSIZE*15/16, 27, OPT_CENTERY+OPT_RIGHTX, calib_txt[language][3]);
                    ComTXT(HSIZE/2, VSIZE*1/16, 27, OPT_CENTER, calib_txt[language][4]);
                    ReadTouchScreen();
                    if(POSY!=0x8000){
                        ComPoint(POSX, POSY, 20);
                        if(POSX>HSIZE-40){
                            end_calib=1;
                            disp_state=SETTINGSSCR;
                        }
                        else if(POSX<40){
                            end_calib=1;
                            disp_state=CALIBRATESCR;
                        }
                    }
                    Draw();
                }
                prev_t=ms_count;
                break;

            case MANCTLSCR:
                NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                lang_date_time_frame();
                ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][MANCTLTITLE]);

                ComFgcolor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                ComBgcolor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);

                switch(manaccess_touch){
                case UNTOUCHED:
                    if(POSX>MANCTLTOGLOC_X-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16 && PREVPOSY==0x8000)
                        manaccess_touch=OPENINGAUTO;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16 && PREVPOSY==0x8000)
                        manaccess_touch=OPENINGMANUAL;
                    else if(POSX>MANCTLTOGLOC_X-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16 && PREVPOSY==0x8000)
                        manaccess_touch=TRANSPAUTO;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16 && PREVPOSY==0x8000)
                        manaccess_touch=TRANSPMANUAL;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16 && PREVPOSY==0x8000 && openingmode_state)
                        manaccess_touch=OPENINGCLOSE;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16 && PREVPOSY==0x8000 && openingmode_state)
                        manaccess_touch=OPENINGOPEN;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16 && PREVPOSY==0x8000 && transparencymode_state)
                        manaccess_touch=TRANSPOPAQUE;
                    else if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16 && PREVPOSY==0x8000 && transparencymode_state)
                        manaccess_touch=TRANSPTRANSP;
                    break;
                case OPENINGAUTO:
                    openingmode_state=false;
                    opening_state=false;
                    window_servo_state=opening_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16)
                        manaccess_touch=OPENINGMANUAL;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case OPENINGMANUAL:
                    openingmode_state=true;
                    window_servo_state=opening_state;
                    if(POSX>MANCTLTOGLOC_X-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16)
                        manaccess_touch=OPENINGAUTO;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case TRANSPAUTO:
                    transparencymode_state=false;
                    transparency_state=false;
                    window_esg_state=transparency_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16)
                        manaccess_touch=TRANSPMANUAL;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case TRANSPMANUAL:
                    transparencymode_state=true;
                    window_esg_state=transparency_state;
                    if(POSX>MANCTLTOGLOC_X-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16)
                        manaccess_touch=TRANSPAUTO;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case OPENINGCLOSE:
                    opening_state=false;
                    window_servo_state=opening_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16)
                        manaccess_touch=OPENINGOPEN;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case OPENINGOPEN:
                    opening_state=true;
                    window_servo_state=opening_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*9/32 && POSY<VSIZE*6/16)
                        manaccess_touch=OPENINGCLOSE;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case TRANSPOPAQUE:
                    transparency_state=false;
                    window_esg_state=transparency_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSX<MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD+HSIZE/32 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16)
                        manaccess_touch=TRANSPTRANSP;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                case TRANSPTRANSP:
                    transparency_state=true;
                    window_esg_state=transparency_state;
                    if(POSX>MANCTLTOGLOC_X+MANCTLTOGLOC_WBT-HSIZE/32 && POSX<=MANCTLTOGLOC_X+MANCTLTOGLOC_WBT+MANCTLTOGLOC_WD/2.0 && POSY>VSIZE*17/32 && POSY<VSIZE*10/16)
                        manaccess_touch=TRANSPOPAQUE;
                    else if(POSY==0x8000)
                        manaccess_touch=UNTOUCHED;
                    break;
                }
                ComTXT(MANCTLTOGLOC_X-10,MANCTLTOGLOC_Y-VSIZE/16, 27, OPT_CENTERY, manualctlops_txt[language][OPENINGTXT]);
                ComToggle(MANCTLTOGLOC_X, MANCTLTOGLOC_Y, MANCTLTOGLOC_WD, 27, OPT_FLAT, openingmode_state*0xFFFF, "AUTO" "\xff" "MANUAL");
                if(!openingmode_state)
                    ComBgcolor(DISABLEDGRAY_R,DISABLEDGRAY_G,DISABLEDGRAY_B);
                if(language==ENGLISH)
                    ComToggle(MANCTLTOGLOC_X+MANCTLTOGLOC_WBT, MANCTLTOGLOC_Y, MANCTLTOGLOC_WD, 27, OPT_FLAT, opening_state*0xFFFF, "CLOSE" "\xff" "OPEN");
                else if(language==SPANISH)
                    ComToggle(MANCTLTOGLOC_X+MANCTLTOGLOC_WBT, MANCTLTOGLOC_Y, MANCTLTOGLOC_WD, 27, OPT_FLAT, opening_state*0xFFFF, "CERRADO" "\xff" "ABIERTO");

                ComBgcolor(OBJECTBGGRAY,OBJECTBGGRAY,OBJECTBGGRAY);
                ComTXT(MANCTLTOGLOC_X-10,MANCTLTOGLOC_Y-VSIZE/16+MANCTLTOGLOC_HBT, 27, OPT_CENTERY, manualctlops_txt[language][TRANSPARENCYTXT]);
                ComToggle(MANCTLTOGLOC_X, MANCTLTOGLOC_Y+MANCTLTOGLOC_HBT, MANCTLTOGLOC_WD, 27, OPT_FLAT, transparencymode_state*0xFFFF, "AUTO" "\xff" "MANUAL");
                if(!transparencymode_state)
                    ComBgcolor(DISABLEDGRAY_R,DISABLEDGRAY_G,DISABLEDGRAY_B);
                if(language==ENGLISH)
                    ComToggle(MANCTLTOGLOC_X+MANCTLTOGLOC_WBT, MANCTLTOGLOC_Y+MANCTLTOGLOC_HBT, MANCTLTOGLOC_WD, 27, OPT_FLAT, transparency_state*0xFFFF, "OPAQUE" "\xff" "TRANSP");
                else if(language==SPANISH)
                    ComToggle(MANCTLTOGLOC_X+MANCTLTOGLOC_WBT, MANCTLTOGLOC_Y+MANCTLTOGLOC_HBT, MANCTLTOGLOC_WD, 27, OPT_FLAT, transparency_state*0xFFFF, "OPACO" "\xff" "TRANSP");

                BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*13/16,HSIZE*5/32,VSIZE*2/16,26,back_txt[language]);
                if(language==SPANISH){
                    ComLineWidth(0.75);
                    accent_create(HSIZE*4/32 +2,VSIZE*14/16 +1,26);
                    ComLineWidth(1);
                }
                Draw();
                if(BACK_but){
                    while(POSY!=0x8000){ReadTouchScreen();}
                    disp_state=AMBVARSSCR;
                    prev_t=ms_count;
                }
                break;

                case PLOTSCR:
                    NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                    lang_date_time_frame();
                    ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][PLOTTITLE]);
                    if(language==SPANISH){
                        ComLineWidth(0.75);
                        accent_create(HSIZE*9/32,VSIZE*1.5/32,27);
                        ComLineWidth(1);
                    }

                    axis_create(HSIZE*4/16,VSIZE*5/16,HSIZE*5/16,VSIZE*5/16);
                    false_plot_create(HSIZE*4/16,VSIZE*5/16,HSIZE*5/16,VSIZE*5/16);
                    axis_create(HSIZE*11/16,VSIZE*5/16,HSIZE*5/16,VSIZE*5/16);
                    false_plot_create(HSIZE*11/16,VSIZE*5/16,HSIZE*5/16,VSIZE*5/16);
                    axis_create(HSIZE*11/16,VSIZE*12/16,HSIZE*5/16,VSIZE*5/16);
                    false_plot_create(HSIZE*11/16,VSIZE*12/16,HSIZE*5/16,VSIZE*5/16);

                    ComColor(0xFF,0xA0,0xA0);
                    TEMP_but=SET_button_touched(HSIZE*5.5/32,VSIZE*7/32, HSIZE*5/32, VSIZE*6/32,26, "TEMP");
                    sprintf(auxstr1,"Tint: %.2f",T_int);
                    ComTXT(HSIZE*2/32,VSIZE*18/32,27,OPT_CENTERY,auxstr1);
                    sprintf(auxstr2,"Text: %.2f",T_ext);
                    ComTXT(HSIZE*2/32,VSIZE*21/32,27,OPT_CENTERY,auxstr2);

                    ComColor(0xA0,0xA0,0xFF);
                    HUM_but=SET_button_touched(HSIZE*19.5/32,VSIZE*7/32, HSIZE*5/32, VSIZE*6/32,26, "HUM");
                    sprintf(auxstr1, "Hint: %.2f",H_int);
                    ComTXT(HSIZE*10/32,VSIZE*18/32,27,OPT_CENTERY,auxstr1);
                    sprintf(auxstr2,"Hext: %.2f",H_ext);
                    ComTXT(HSIZE*10/32,VSIZE*21/32,27,OPT_CENTERY,auxstr2);

                    ComColor(0xA0,0xFF,0xA0);
                    LUM_but=SET_button_touched(HSIZE*19.5/32,VSIZE*21/32, HSIZE*5/32, VSIZE*6/32,26, "LUM");
                    sprintf(auxstr1, "L: %.2f",L_int);
                    ComTXT(HSIZE*10/32,VSIZE*24/32,27,OPT_CENTERY,auxstr1);
                    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);

                    BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*13/16,HSIZE*5/32,VSIZE*4/32,26,back_txt[language]);
                    if(language==SPANISH){
                        ComLineWidth(0.75);
                        accent_create(HSIZE*4/32 +2,VSIZE*14/16 +1,26);
                        ComLineWidth(1);
                    }

                    Draw();
                    if(TEMP_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR_T;
                        prev_t=ms_count;
                    }

                    if(HUM_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR_H;
                        prev_t=ms_count;
                    }

                    if(LUM_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR_L;
                        prev_t=ms_count;
                    }

                    if(BACK_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=AMBVARSSCR;
                        prev_t=ms_count;
                    }
                    break;

                case PLOTSCR_T:
                    NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                    lang_date_time_frame();
                    ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][TEMPTITLE]);

                    ComLineWidth(2);
                    axis_create(HSIZE*17/32,VSIZE*15/32 +1,HSIZE*24/32,VSIZE*22/32);
                    ComLineWidth(1);

                    for(i=MINPLOT_TEMP+1; i<=MAXPLOT_TEMP;i+=2){
                        usnprintf(auxstr0,sizeof(auxstr0),"%u",i);
                        Command(CMD_BEGIN_LINES);
                        ComVertex2ff(HSIZE*5/32-8,(VSIZE*22/32)*(MAXPLOT_TEMP-i)/(MAXPLOT_TEMP-MINPLOT_TEMP)+VSIZE*4/32);
                        ComVertex2ff(HSIZE*5/32+8,(VSIZE*22/32)*(MAXPLOT_TEMP-i)/(MAXPLOT_TEMP-MINPLOT_TEMP)+VSIZE*4/32);
                        Command(CMD_END);
                        ComTXT(HSIZE*3/32, (VSIZE*22/32)*(MAXPLOT_TEMP-i)/(MAXPLOT_TEMP-MINPLOT_TEMP)+VSIZE*4/32, 26, OPT_CENTER, auxstr0);
                    }

                    Command(CMD_BEGIN_LINE_STRIP);
                    for(i=0; i<T_VECTORSIZE;i++){
                        ComVertex2ff((HSIZE*24/32)*i/(T_VECTORSIZE-1) + HSIZE*5/32,(VSIZE*22/32)*(MAXPLOT_TEMP-(T_int_v[i]<MINPLOT_TEMP?MINPLOT_TEMP:(T_int_v[i]>MAXPLOT_TEMP?MAXPLOT_TEMP:T_int_v[i])))/(MAXPLOT_TEMP-MINPLOT_TEMP)+VSIZE*4/32);
                    }
                    Command(CMD_END);

                    BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*27.5/32,HSIZE*3/32,VSIZE*3/32,26,back_txt[language]);
                    if(language==SPANISH){
                        ComLineWidth(0.75);
                        accent_create(HSIZE*3/32 +2,VSIZE*29/32 +1,26);
                        ComLineWidth(1);
                    }
                    Draw();
                    if(BACK_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR;
                        prev_t=ms_count;
                    }
                    break;

                case PLOTSCR_H:
                    NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                    lang_date_time_frame();
                    ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][HUMTITLE]);

                    ComLineWidth(2);
                    axis_create(HSIZE*17/32,VSIZE*15/32 +1,HSIZE*24/32,VSIZE*22/32);
                    ComLineWidth(1);

                    for(i=MINPLOT_HUM+5; i<=MAXPLOT_HUM;i+=5){
                        usnprintf(auxstr0,sizeof(auxstr0),"%u",i);
                        Command(CMD_BEGIN_LINES);
                        ComVertex2ff(HSIZE*5/32-8,(VSIZE*22/32)*(MAXPLOT_HUM-i)/(MAXPLOT_HUM-MINPLOT_HUM)+VSIZE*4/32);
                        ComVertex2ff(HSIZE*5/32+8,(VSIZE*22/32)*(MAXPLOT_HUM-i)/(MAXPLOT_HUM-MINPLOT_HUM)+VSIZE*4/32);
                        Command(CMD_END);
                        ComTXT(HSIZE*3/32, (VSIZE*22/32)*(MAXPLOT_HUM-i)/(MAXPLOT_HUM-MINPLOT_HUM)+VSIZE*4/32, 26, OPT_CENTER, auxstr0);
                    }

                    Command(CMD_BEGIN_LINE_STRIP);
                    for(i=0; i<H_VECTORSIZE;i++){
                        ComVertex2ff((HSIZE*24/32)*i/(H_VECTORSIZE-1) + HSIZE*5/32,(VSIZE*22/32)*(MAXPLOT_HUM-(H_int_v[i]<MINPLOT_HUM?MINPLOT_HUM:(H_int_v[i]>MAXPLOT_HUM?MAXPLOT_HUM:H_int_v[i])))/(MAXPLOT_HUM-MINPLOT_HUM)+VSIZE*4/32);
                    }
                    Command(CMD_END);

                    BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*27.5/32,HSIZE*3/32,VSIZE*3/32,26,back_txt[language]);
                    if(language==SPANISH){
                        ComLineWidth(0.75);
                        accent_create(HSIZE*3/32 +2,VSIZE*29/32 +1,26);
                        ComLineWidth(1);
                    }
                    Draw();
                    if(BACK_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR;
                        prev_t=ms_count;
                    }
                    break;

                case PLOTSCR_L:
                    NewScreen(MAINBG_R,MAINBG_G,MAINBG_B);
                    ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                    lang_date_time_frame();
                    ComTXT(HSIZE*2/16,VSIZE*1.5/32, 27, OPT_CENTERY, title[language][LUMTITLE]);

                    ComLineWidth(2);
                    axis_create(HSIZE*17/32,VSIZE*15/32 +1,HSIZE*24/32,VSIZE*22/32);
                    ComLineWidth(1);

                    for(i=MINPLOT_LUM+10; i<=MAXPLOT_LUM;i+=10){
                        usnprintf(auxstr0,sizeof(auxstr0),"%u",i);
                        Command(CMD_BEGIN_LINES);
                        ComVertex2ff(HSIZE*5/32-8,(VSIZE*22/32)*(MAXPLOT_LUM-i)/(MAXPLOT_LUM-MINPLOT_LUM)+VSIZE*4/32);
                        ComVertex2ff(HSIZE*5/32+8,(VSIZE*22/32)*(MAXPLOT_LUM-i)/(MAXPLOT_LUM-MINPLOT_LUM)+VSIZE*4/32);
                        Command(CMD_END);
                        ComTXT(HSIZE*3/32, (VSIZE*22/32)*(MAXPLOT_LUM-i)/(MAXPLOT_LUM-MINPLOT_LUM)+VSIZE*4/32, 26, OPT_CENTER, auxstr0);
                    }

                    Command(CMD_BEGIN_LINE_STRIP);
                    for(i=0; i<L_VECTORSIZE;i++){
                        ComVertex2ff((HSIZE*24/32)*i/(L_VECTORSIZE-1) + HSIZE*5/32,(VSIZE*22/32)*(MAXPLOT_LUM-(L_int_v[i]<MINPLOT_LUM?MINPLOT_LUM:(L_int_v[i]>MAXPLOT_LUM?MAXPLOT_LUM:L_int_v[i])))/(MAXPLOT_LUM-MINPLOT_LUM)+VSIZE*4/32);
                    }
                    Command(CMD_END);

                    BACK_but=SET_button_touched(HSIZE*1/32,VSIZE*27.5/32,HSIZE*3/32,VSIZE*3/32,26,back_txt[language]);
                    if(language==SPANISH){
                        ComLineWidth(0.75);
                        accent_create(HSIZE*3/32 +2,VSIZE*29/32 +1,26);
                        ComLineWidth(1);
                    }
                    Draw();
                    if(BACK_but){
                        while(POSY!=0x8000){ReadTouchScreen();}
                        disp_state=PLOTSCR;
                        prev_t=ms_count;
                    }
                    break;

                case CLOCKSCR:
                    if(setDate)
                    {
                        DateTimeDefaultSet();
                        DateTimeSet();
                        setDate = false;
                    }
                    RTC_updated = DateTimeDisplayGet(DateBuf, TimeBuf, sizeof(DateBuf), sizeof(TimeBuf),RTC_setted);
                    if(RTC_updated || first_clockscr)
                    {
                        if(RTC_setted){
                            RTC_setted=false;
                        }
                        NewScreen(0,0,0);
                        ComColor(FONT1COLOR_R,FONT1COLOR_G,FONT1COLOR_B);
                        ComTXT(HSIZE/32,VSIZE*1.5/32,21,OPT_CENTER,lang_abrev[language]);
                        ComTXT(HSIZE*28/32,VSIZE*1.5/32,21,OPT_CENTER,DateBuf);
                        ComTXT(HSIZE/2,VSIZE/2, 31, OPT_CENTER,TimeBuf);
                        Draw();
                        first_clockscr=false;
                    }
                    if(POSY==0x8000 && PREVPOSY!=0x8000)
                        disp_state=actualscr_state;
                    break;
        }

        switch(window_state){
        case CLOSEWINDOW:
            if(openingmode_state==MANUAL){
                if(window_servo_state)
                    window_state=OPENINGWINDOW;
            }
            else{
                if( (tempslider>0 && T_int<T_ext && humidslider==0) || (tempslider<0 && T_int>T_ext && humidslider==0) ||     \
                        (humidslider>0 && H_int<H_ext && tempslider==0) || (humidslider<0 && H_int>H_ext && tempslider==0) || \
                        (tempslider>0 && T_int<T_ext && (humidslider>0 && H_int<H_ext || humidslider<0 && H_int>H_ext)) ||    \
                        (tempslider<0 && T_int>T_ext && (humidslider>0 && H_int<H_ext || humidslider<0 && H_int>H_ext)) )
                    window_state=OPENINGWINDOW;
            }
            break;
        case OPENINGWINDOW:
            open_window();
            window_state=OPENWINDOW;
            break;
        case OPENWINDOW:
            if(openingmode_state==MANUAL){
                if(!window_servo_state)
                    window_state=CLOSINGWINDOW;
            }
            else{
                if( (tempslider>0 && T_int>T_ext && humidslider==0) || (tempslider<0 && T_int<T_ext && humidslider==0) ||     \
                        (humidslider>0 && H_int>H_ext && tempslider==0) || (humidslider<0 && H_int<H_ext && tempslider==0) || \
                        (tempslider>0 && T_int>T_ext && (humidslider>0 && H_int>H_ext || humidslider<0 && H_int<H_ext)) ||    \
                        (tempslider<0 && T_int<T_ext && (humidslider>0 && H_int>H_ext || humidslider<0 && H_int<H_ext)) ||    \
                        (tempslider==0 && humidslider==0) )
                    window_state=CLOSINGWINDOW;
            }
            break;
        case CLOSINGWINDOW:
            close_window();
            window_state=CLOSEWINDOW;
            break;
        }

        switch(esg_state){
        case OPAQUEWINDOW:
            if(transparencymode_state==MANUAL){
                if(window_esg_state)
                    esg_state=CLEARINGWINDOW;
            }
            else{
                if(lightslider>0 && L_int<L_THRESHOLD && g_ui32HourIdx>9 && g_ui32HourIdx<21) //it differences day and night working
                    esg_state=CLEARINGWINDOW;
            }
            break;
        case CLEARINGWINDOW:
            clear_window();
            esg_state=TRANSPARENTWINDOW;
            break;
        case TRANSPARENTWINDOW:
            if(transparencymode_state==MANUAL){
                if(!window_esg_state)
                    esg_state=SHADINGWINDOW;
            }
            else{
                if(lightslider<0 && L_int>L_THRESHOLD) //it doesn't bear day or night for night-time privacy with lights turned on
                    esg_state=SHADINGWINDOW;
            }
            break;
        case SHADINGWINDOW:
            white_window();
            esg_state=OPAQUEWINDOW;
            break;
        }

        PREVPOSX=POSX;
        PREVPOSY=POSY;
    }
}
uint8_t Test_I2C_dir(uint32_t pos, uint8_t dir)
{
    uint8_t error=100;
    uint32_t I2C_Base=0;
    uint32_t *I2CMRIS;
    if(pos==1) I2C_Base=I2C0_BASE;
    if(pos==2) I2C_Base=I2C2_BASE;
    if(I2C_Base){
        I2CMasterSlaveAddrSet(I2C_Base, dir, 1);  //Modo LECTURA
        I2CMasterControl(I2C_Base, I2C_MASTER_CMD_SINGLE_RECEIVE); //Lect. Simple

        while(!(I2CMasterBusy(I2C_Base)));  //Espera que empiece
        while((I2CMasterBusy(I2C_Base)));   //Espera que acabe
        I2CMRIS= (uint32_t *)(I2C_Base+0x014);
        error=(uint8_t)((*I2CMRIS) & 0x10);
        if(error){
            I2CMRIS=(uint32_t *)(I2C_Base+0x01C);
            *I2CMRIS=0x00000010;
        }
    }
    return error;
}
void SysTickIntHandler()
{
    ms_count++;
    if(ms_count>2419200000)
        ms_count-=2419200000;
    if(ms_count%1000==0)
        SampleBell=1;
}
