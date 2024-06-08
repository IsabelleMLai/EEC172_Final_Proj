//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required 
//                        initialization sequence to enable the CC3200 SPI 
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdio.h>

// Driverlib includes
#include "hw_apps_rcm.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "Adafruit_SSD1351.h"
#include "oled_test.h"
#include "Adafruit_GFX.h"
#include "glcdfont.h"
#include "i2c_if.h"

#include "gpio.h"
#include "utils.h"


#include "hw_nvic.h"
#include "systick.h"

//#include "controller.h"
//#include "Adafruit_OLED.c"



//-----------------------------------------------------------------------------
//UART
#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME             "UART Echo"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80

//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"
#define I2C_BASE                I2CA0_BASE
#define SYS_CLK                 80000000
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}


//-----------------------------------------------------------------------------
//SYSTICK

#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 8000000UL

//-----------------------------------------------------------------------------
//CONTROLLER
#define BUTTON_ONE -1749723041
#define BUTTON_TWO 66219167
#define BUTTON_THREE -1474275809
#define BUTTON_FOUR 1106806847
#define BUTTON_FIVE -433688129
#define BUTTON_SIX 1382254079
#define BUTTON_SEVEN -158240897
#define BUTTON_EIGHT -936618033
#define BUTTON_NINE 1817854287
#define BUTTON_ZERO -209228065
#define BUTTON_LAST -661170801
#define BUTTON_ENTER  891055645

//-----------------------------------------------------------------------------
//OLEED IMAGE

//axis = 100  pix
    //136 Fahrenheit (58 Celsius)
    //-126 Fahrenheit (-88 Celsius)
    //C = 150 deg range;  F = 262 deg range

#define ORIGIN_X        10
#define  ORIGIN_Y       120
#define GRAPH_MX      110
#define GRAPH_MY        20
//fillCircle(10, 10, 3, RED); //TOP LEFT
//    fillCircle(10, 100, 3, BLACK);//BOTTOM LEFT
//    fillCircle(100, 10, 3, BLUE);   //top right



//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

volatile int g_iCounter = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//-----------------------------------------------------------------------------
//SYSTICK
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);


volatile unsigned long SW2_intcount;
volatile unsigned char SW2_intflag;

volatile uint64_t delta_previous;
volatile uint64_t delta_current;
volatile uint64_t delta_us;


// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting our_input = { .port = GPIOA3_BASE, .pin = 0x2};


//-----------------------------------------------------------------------------

static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//-----------------------------------------------------------------------------
//CONTROLLER
unsigned long ulStatus;

int msb = 0;

int button_data = 0;
int button_readable  = -1;

int full_num[100];
int full_num_int = 0;
int place_value_arr_size = 0;

//-----------------------------------------------------------------------------
//OLEED IMAGE
int x_coordinate = 64;
int y_coordinate = 64;
int prev_x_coordinate = 64;
int prev_y_coordinate = 64;

char celsius[7] = {'C', 'e', 'l',  's', 'i', 'u', 's'};
int C_num_let = 7;
int C_x_axis_loc = ORIGIN_Y-60;

char fahrenheit[10] = {'F', 'a', 'h',  'r', 'e', 'n', 'h', 'e', 'i', 't'};
int F_num_let = 10;
int F_x_axis_loc = ORIGIN_Y-48;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus_slave;

    ulStatus_slave = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus_slave & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus_slave & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}


//---------------------------------------------------------------


/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    Report("reset cnt = %d\n\n", systick_cnt);
    HWREG(NVIC_ST_CURRENT) = 1;


    // clear the global count variable
    systick_cnt = 0;

}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
//    Report("handle cnt = %d\n", systick_cnt);
}


static void GPIOA3IntHandler(void) {
    Report("GPIOHANDLER\n");
//    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (our_input.port, true);
    MAP_GPIOIntClear(our_input.port, ulStatus);     // clear interrupts on GPIOA2
    SW2_intcount++;
    SW2_intflag=1;


    // read the countdown register and compute elapsed cycles
    delta_current = SYSTICK_RELOAD_VAL - SysTickValueGet();

    // convert elapsed cycles to microseconds
    delta_us = TICKS_TO_US(delta_current);

    SysTickReset();


}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

void controller_setup(void) {
//    unsigned long ulStatus;

    Report("\t****************************************************\n\r");
//    Message("\t\t\t LAB 3\n\r");
//    Message("\t\t ****************************************************\n\r");
//    Message("\n\n\n\r");


}


static int ButtonPressed(int b_data) {
    int button = -1;

    switch(b_data)  {
        case(BUTTON_ONE) :
                button = 1;
        break;
        case(BUTTON_TWO):
                button = 2;
        break;
        case(BUTTON_THREE):
                button = 3;
        break;
        case(BUTTON_FOUR):
                button = 4;
        break;
        case(BUTTON_FIVE):
                button = 5;
        break;
        case(BUTTON_SIX):
                button = 6;
        break;
        case(BUTTON_SEVEN):
                button = 7;
        break;
        case(BUTTON_EIGHT):
                button = 8;
        break;
        case(BUTTON_NINE):
                button = 9;
        break;
        case(BUTTON_ZERO):
                button = 0;
        break;
        case(BUTTON_LAST):
                button = 10;
        break;
        case(BUTTON_ENTER):
                button = 11;
        break;

    }
    return button;
}


static int ConvertArrToFullNum(int arr[], int a_size) {
    if(a_size<1) {
        return -1;
    }

    int last_ind = a_size-1;

    int result = 0;

    while(last_ind>=0) {
        int mult_pwr = 1;
        int pwr = a_size-1;

        while(pwr>last_ind) {
            mult_pwr *= 10;
            pwr-=1;
        }

        result+= (mult_pwr*arr[last_ind]);

        last_ind-=1;
    }
    return result;
}


void button_flag_raised(volatile unsigned char* SW2_intflag, uint64_t delta_us) {
    Report("Flag %c\n", *SW2_intflag);
    if ((*SW2_intflag)) {
        *SW2_intflag=0;

        msb = delta_us/1000;        //should truncate
        Report("msb = %d\n", msb);
        if(msb>0) {

            if(msb == 1) {              //bit=1
                button_data = button_data*10;       //button_Data =binary code of signal
            }
            else if (msb == 2) {        //bit =2
                button_data = (button_data*10) + 1;
            }
            else {                      //extra signal we don't need

                if(button_data!=0)  {           //if we got a readable button press

                    button_readable = ButtonPressed(button_data);
//                    Report("button read: %d, msb: %d\n ", button_readable, msb);



                    if(button_readable == 10) {     //delete presseed
                        if(place_value_arr_size>1) {
                            place_value_arr_size -=1;
//                            Report("deelete, Ind: %d", place_value_arr_size);
                        } else {
//                            Report("empty delete\n");
                        }

                    } else if(button_readable == 11) {      //enter
//                        Report("place val  = %d \n", place_value_arr_size);
                        full_num_int = ConvertArrToFullNum(full_num, place_value_arr_size);
                        Report("full num  = %d \n", full_num_int);
    //                            Report("enter");

                        int i=0;
                        while(full_num[i]  != '\0')  {
                            full_num[i]  = '\0';
                            i+=1;
                        }

                        place_value_arr_size = 0;

                    } else if(button_readable != -1) {      //button in bounds
                            place_value_arr_size +=1;
                            full_num[place_value_arr_size-1]=button_readable;

                            Report("button read = %d\n",  button_readable);
                    }
                }
                button_data = 0;
                button_readable = -1;
            }

        }
    }
}

//-----------------------------------------------------------------------------
//OLED IMAGE

void CreateTitle(char arr[], int arr_size) {
    int curr_ind = 0;
    int font_size = 6;

    int x_loc = 10;
    int y_loc = x_loc;

    while(curr_ind<arr_size) {
        drawChar(x_loc, y_loc, arr[curr_ind],BLACK, WHITE, 1);
        x_loc+=font_size;
        curr_ind+=1;
    }
}

//only used in other functoin, not main
void MakePoint(int point_val, int nth_point, char temp, int color)  {

      if(temp=='F') {
          //axis = 100  pix
          //2.7 deg per pix
              //52*2.7=140 deg TOP
              //48*2.7=129.6 deg BOTTOM
          int loc = point_val/2.7;
          fillCircle((nth_point*10), (F_x_axis_loc-loc), 2, color);

      }  else if(temp=='C')  {
          int loc = point_val/1.5;
          fillCircle((nth_point*10), (C_x_axis_loc-loc), 2, color);
      }
}



void DisplayData(char temp_type, int arr[], int arr_size, int color) {
    int curr_ind = 0;

    while(curr_ind<arr_size) {
        MakePoint(arr[curr_ind], (curr_ind+1), temp_type,  color);

        if(curr_ind>0) {
            int prev_temp = arr[curr_ind-1];

            if(temp_type=='F') {
                int prev_temp_loc = prev_temp/2.7;
                int curr_temp_loc = arr[curr_ind]/2.7;
                drawLine((curr_ind*10), (F_x_axis_loc-prev_temp_loc), ((curr_ind+1)*10), (F_x_axis_loc-curr_temp_loc), color);
            } else if(temp_type == 'C') {
                int prev_temp_loc = prev_temp/1.5;
                int curr_temp_loc = arr[curr_ind]/1.5;
                drawLine((curr_ind*10), (C_x_axis_loc-prev_temp_loc), ((curr_ind+1)*10), (C_x_axis_loc-curr_temp_loc), color);

            }

        }

        curr_ind+=1;
    }
}

void UpdateData(int new_temp, int arr[], int arr_size) {
    //x-axis = 100 pixels, 10 data points  = 10pixel horizontal  increments
    int curr_ind = 0;
    while(curr_ind<arr_size) {
        if(curr_ind<arr_size-1) {
            arr[curr_ind] = arr[(curr_ind+1)];
        } else {
            arr[curr_ind] = new_temp;
        }
        curr_ind+=1;
    }
}

void ConvertFCorCF(char prev_temp, char temp, int arr[], int arr_size) {
//    (32°F − 32) × 5/9 = 0°C
    int  curr_ind = 0;
    while(curr_ind<arr_size) {

        if((prev_temp=='F')  && (temp=='C')) {
            int C_temp = ((arr[curr_ind]-32)*5)/9;
            arr[curr_ind] = C_temp;

        } else {
            int F_temp = ((arr[curr_ind]*9)/5)+32;
            arr[curr_ind] = F_temp;
        }
        curr_ind+=1;
    }
}



void DisplayGraph(char temp_type) {
    if(temp_type == 'F') {
        CreateTitle(fahrenheit, F_num_let);
        //2.7 deg per pix
        drawLine(ORIGIN_X, ORIGIN_Y, ORIGIN_X, GRAPH_MY, BLACK);    //y-axis
        drawLine(ORIGIN_X, F_x_axis_loc, GRAPH_MX, F_x_axis_loc, BLACK);    //x-axis

    } else if(temp_type == 'C') {
        CreateTitle(celsius, C_num_let);
        //1.5 deg per pix
        drawLine(ORIGIN_X, ORIGIN_Y, ORIGIN_X, GRAPH_MY, BLACK);    //y-axis
        drawLine(ORIGIN_X, C_x_axis_loc, GRAPH_MX, C_x_axis_loc, BLACK);    //x-axis

    }

}

void RedrawXAxis(char temp) {
    if(temp == 'F') {
        drawLine(ORIGIN_X, F_x_axis_loc, GRAPH_MX, F_x_axis_loc, BLACK);    //x-axis
    } else if(temp=='C') {
        drawLine(ORIGIN_X, C_x_axis_loc, GRAPH_MX, C_x_axis_loc, BLACK);    //x-axis
    }
}

int ConvertCharInt(char cStr[], int iStringLength) {
    int whole_val = 0;
    int negative =0;
    if(cStr[0]=='-') {
        negative = 1;
    }

    int curr_ind = iStringLength-1;
    while(curr_ind>=negative) {
        int digit = cStr[curr_ind] - '0';

        int multiplier = iStringLength - (curr_ind+1);
        while(multiplier>0) {
            digit *= 10;
            multiplier-=1;
        }

        whole_val+=digit;
        curr_ind-=1;
    }
    if(negative) {
        whole_val*=-1;
    }
    return whole_val;
}



//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{
    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    // MOSI = channel 0
    // MISO = channel 1
    // clock = channel 2
    // enable (OC) = channel 3

    Adafruit_Init();

    fillScreen(WHITE);


//
//    UpdateData( 20, F_arr, 10);
//    UpdateData( 27, F_arr, 10);
//    UpdateData( -120, F_arr, 10);
//    UpdateData( 130, F_arr, 10);
////    ConvertFC(F_arr, 10);
//
//    DisplayData('F', F_arr, 10);

    I2C_IF_Open(I2C_MASTER_MODE_FST);


    char cString[MAX_STRING_LENGTH+1];
    char cCharacter;
    int iStringLength = 0;

    int temp_arr[10] = {0};
    char temp_type = 'A';
    char prev_temp_type = 'A';
    int loop_count = 0;


    Report("Type any key to begin \n");

    while(1){

        //
        // Fetching the input from the terminal.
        //
        cCharacter = UartGetChar();
        g_iCounter++;



        if(cCharacter == '\r' || cCharacter == '\n' ||
                (iStringLength >= MAX_STRING_LENGTH -1))
        {
            if(iStringLength >= MAX_STRING_LENGTH - 1)
            {
                UartPutChar(cCharacter);
                cString[iStringLength] = cCharacter;
                iStringLength++;
            }
            cString[iStringLength] = '\0';
            loop_count+=1;


            int whole_val = 0;

            if(loop_count%2 == 1) {
//                Report("\nType 'C' for Celsius, or 'F' for Fahrenheit:  \n");
//                prev_temp_type = temp_type;

                if(loop_count>1) {

                    whole_val = ConvertCharInt(cString, iStringLength);
                    int valid_temp = 1;
                    //if temmp out of bounds, try  again
                       //136 Fahrenheit (58 Celsius)
                          //-126 Fahrenheit (-88 Celsius)
                    if(temp_type == 'C') {
                        if((whole_val>58) || (whole_val<-88)) {
                            Report("\nThe coldest and hottest C temperatures ever recorded are -88C and 58C.\n");
                            Report("\nType a temperature value in this range: \n");
                            loop_count-=1;
                            valid_temp = 0;
                        } else {
                            Report("\nType 'C' for Celsius, or 'F' for Fahrenheit:  \n");
                            prev_temp_type = temp_type;
                            valid_temp = 1;
                        }
                    } else  if(temp_type == 'F') {
                        if((whole_val>136) || (whole_val<-126)) {
                            Report("\nThe coldest and hottest F temperatures ever recorded are -126F and 136F.\n");
                            Report("\nType a temperature value in this range: \n");
                            loop_count-=1;
                            valid_temp = 0;
                        } else {
                            Report("\nType 'C' for Celsius, or 'F' for Fahrenheit:  \n");
                            prev_temp_type = temp_type;
                            valid_temp = 1;
                        }

                    }

                    if(valid_temp) {
//                        Report("\ntemp  val = %c,  prev = %c\n", temp_type, prev_temp_type);
                        DisplayData(temp_type, temp_arr, 10, WHITE);
                        RedrawXAxis(temp_type);
                        UpdateData( whole_val, temp_arr, 10);
                        DisplayData(temp_type, temp_arr, 10, RED);
                    }
                } else {
                    Report("\nType 'C' for Celsius, or 'F' for Fahrenheit:  \n");
                    prev_temp_type = temp_type;
                }


            } else {
                if(cString[0] == 'C') {
                    if(temp_type != 'A') {
                        prev_temp_type =  temp_type;
                    }
                    temp_type =  'C';

                } else if(cString[0] ==  'F') {
                    if(temp_type != 'A') {
                        prev_temp_type =  temp_type;
                    }
                    temp_type = 'F';

                } else {
                    Report("\nType 'C' for Celsius, or 'F' for Fahrenheit:  \n");
                    loop_count-=1;
                    temp_type  = 'A';
                }
//                Report("\ntemp  val = %c,  prev = %c\n", temp_type, prev_temp_type);

                if((temp_type == 'C') || (temp_type=='F')) {
                    //136 Fahrenheit (58 Celsius)
                        //-126 Fahrenheit (-88 Celsius)
                    if(temp_type == 'C') {
                        Report("\nThe coldest and hottest C temperatures ever recorded are -88C and 58C.\n");
                    } else  if(temp_type == 'F') {
                        Report("\nThe coldest and hottest F temperatures ever recorded are -126F and 136F.\n");
                    }
                    Report("Type a temperature value in this range: \n");

                    if(prev_temp_type == 'A') {
                        DisplayGraph(temp_type);
                        DisplayData(temp_type, temp_arr, 10, RED);
                    } else if(prev_temp_type!=temp_type) {
                        ConvertFCorCF(prev_temp_type, temp_type, temp_arr, 10);
                        fillScreen(WHITE);
                        DisplayGraph(temp_type);
                        DisplayData(temp_type, temp_arr, 10, RED);
                    }

                }
            }

//            Report("Type 'C' for Celsius, or 'F' for Fahrenheit:  ");


            iStringLength = 0;
            //
            // Echoes the input string
            //%s = cString
//            Report("%s, %d  \n", cString, whole_val);
//            DisplayData('F', temp_arr, 10, WHITE);
//            RedrawXAxis('F');
//            UpdateData( whole_val, temp_arr, 10);
//            DisplayData('F', temp_arr, 10, RED);

        }
        else
        {
            UartPutChar(cCharacter);
            cString[iStringLength] = cCharacter;
            iStringLength++;
        }




////  flagisn't being raised, GPIO innterrupt isnt workinng
//        if(SW2_intflag) {
//            Report("delta us %d", delta_us);
//            button_flag_raised(&SW2_intflag, delta_us);
//
//        }
//
//
//////        button_flag_raised(&SW2_intflag, delta_us);
//////        Report("num = %d\n", GetNumPressed());
////
////        I2C_IF_ReadFrom(ucDevAddr, pucWrDataBuf_X, ucWrLen, pucRdDataBuf_X,ucRdLen);
////
////        // 0 - 127 (positive)
////        // 128 - 255 (negative)
////
////        I2C_IF_ReadFrom(ucDevAddr, pucWrDataBuf_Y, ucWrLen, pucRdDataBuf_Y,ucRdLen);
////
////
////        char x_acc = 0;
////        char y_acc = 0;
////
////        x_acc = pucRdDataBuf_X[0];
////        y_acc = pucRdDataBuf_Y[0];
////
////
////        int int_x_acc = (int)x_acc;
////        int int_y_acc = (int)y_acc;
////
////        // moving negative x direction
////        if(int_x_acc > 127 && int_x_acc < 250){
////            int_x_acc = (int_x_acc - 255);
////        } else if(int_x_acc > 5 && int_x_acc <= 126){
////            int_x_acc = pucRdDataBuf_X[0];
////        } else {
////            int_x_acc = 0;
////        }
////
////        // moving negative y direction
////        if(int_y_acc > 127 && int_y_acc < 250){
////            int_y_acc = (int_y_acc - 255);
////        } else if(int_y_acc > 5 && int_y_acc <= 126){
////            int_y_acc = pucRdDataBuf_Y[0];
////        } else {
////            int_y_acc = 0;
////        }
////
////        fillCircle(x_coordinate, y_coordinate, 3, WHITE);
////
////         int prev_x_coordinate = x_coordinate;
////         int prev_y_coordinate = y_coordinate;
////
//////        Report("%d\n", int_x_acc);
////
////        x_coordinate = x_coordinate - int_x_acc * 0.1;
////
////        if(x_coordinate < 3){
////            x_coordinate = 3;
////        } else if (x_coordinate > 124){
////            x_coordinate = 124;
////        }
////
////
////        y_coordinate = y_coordinate + int_y_acc * 0.1;
////        if(y_coordinate < 3){
////            y_coordinate = 3;
////        } else if (y_coordinate > 124){
////            y_coordinate = 124;
////        }
////
////        fillCircle(x_coordinate, y_coordinate, 3, RED);
////
////        // x cannot be smaller than 3
////        // x cannot be greater than 124
////
////        // y cannot be smaller than 3
////        // y cannot be greater than 124
//
//
//
    }







}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
//#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
//#endif


    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{


//    unsigned long ulStatus;

    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

//    GetNumPressed();
    // Enable SysTick
    SysTickInit();


    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();





////////////////////////
//    controller_setup();
    // Register the interrupt handlers
       //

       MAP_GPIOIntRegister(our_input.port, GPIOA3IntHandler);
       Report("controller setup\n");


       //
       // Configure fallling edge interrupts on button presses
       //
       MAP_GPIOIntTypeSet(our_input.port, our_input.pin, GPIO_FALLING_EDGE);    // our input


       ulStatus = MAP_GPIOIntStatus(our_input.port, false);
       MAP_GPIOIntClear(our_input.port, ulStatus);         // clear interrupts on our input GPIO

       // clear global variables
   //    SW2_example=0;
       SW2_intcount=0;
       SW2_intflag=0;
       delta_current = 0;
       delta_us = 0;

       MAP_GPIOIntEnable(our_input.port, our_input.pin);

   //    SW2_example = MAP_GPIOPinRead(our_input.port, our_input.pin);
   //    Report("SW2 ex = %d\n",SW2_example);
////////////////



//    Report("ulstat 1 = %d", ulStatus);

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 SPI Demo Application  \n\r");
    Message("\t\t   *************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);


#if MASTER_MODE

    MasterMain();

#else

    SlaveMain();

#endif

    while(1)
    {


    }

}

