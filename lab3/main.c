//


// lab 3

//*****************************************************************************
//
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

// Standard includes
#include <stdint.h>
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"


#include "hw_nvic.h"
#include "systick.h"


// Common interface includes
#include "uart_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
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


// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);

extern void (* const g_pfnVectors[])(void);

//-----------------------------------------------------------------------------

volatile unsigned long SW2_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned long SW2_example;


volatile uint64_t delta_previous;
volatile uint64_t delta_current;
volatile uint64_t delta_us;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);


//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
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
}


// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting our_input = { .port = GPIOA0_BASE, .pin = 0x40};



static void GPIOA2IntHandler(void) {
    unsigned long ulStatus;

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
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
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


//PUT BUTTON INTO A  READABLE FORMAT
//          need to check that the return value is greateer than -1before using output
//          last = 10, enter = 11, none = -1
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


//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    // Enable SysTick
    SysTickInit();

    // Initialize UART terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    //
    // Register the interrupt handlers
    //

    MAP_GPIOIntRegister(our_input.port, GPIOA2IntHandler);

    //
    // Configure fallling edge interrupts on button presses
    //
    MAP_GPIOIntTypeSet(our_input.port, our_input.pin, GPIO_FALLING_EDGE);    // our input


    ulStatus = MAP_GPIOIntStatus(our_input.port, false);
    MAP_GPIOIntClear(our_input.port, ulStatus);         // clear interrupts on our input GPIO

    // clear global variables
    SW2_example=0;
    SW2_intcount=0;
    SW2_intflag=0;
    delta_current = 0;
    delta_us = 0;

    MAP_GPIOIntEnable(our_input.port, our_input.pin);

//    SW2_example = MAP_GPIOPinRead(our_input.port, our_input.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t LAB 3\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");


    int msb = 0;

    int button_data = 0;
    int button_readable  = -1;


    int full_num[100];
    int full_num_int = 0;
    int place_value_arr_size = 0;


    while (1) {
        //get time between pressees

        if (SW2_intflag) {
            SW2_intflag=0;

            msb = delta_us/1000;        //should truncate


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
                        Report("button read: %d, msb: %d\n ", button_readable, msb);



                        if(button_readable == 10) {     //delete presseed
                            if(place_value_arr_size>1) {
                                place_value_arr_size -=1;
                                Report("deelete, Ind: %d", place_value_arr_size);
                            } else {
                                Report("empty delete\n");
                            }

                        } else if(button_readable == 11) {      //enter
                            Report("place val  = %d \n", place_value_arr_size);
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
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************





//// lab 3
//
////*****************************************************************************
////
//// Application Name     - int_sw
//// Application Overview - The objective of this application is to demonstrate
////                          GPIO interrupts using SW2 and SW3.
////                          NOTE: the switches are not debounced!
////
////*****************************************************************************
//
////****************************************************************************
////
////! \addtogroup int_sw
////! @{
////
////****************************************************************************
//
//// Standard includes
//#include <stdint.h>
//#include <stdio.h>
//#include <string.h>
//
//// Driverlib includes
//#include "hw_types.h"
//#include "hw_ints.h"
//#include "hw_memmap.h"
//#include "hw_common_reg.h"
//#include "interrupt.h"
//#include "hw_apps_rcm.h"
//#include "prcm.h"
//#include "rom.h"
//#include "rom_map.h"
//#include "prcm.h"
//#include "gpio.h"
//#include "utils.h"
//
//#include "spi.h"
//#include "uart.h"
//
//
//#include "hw_nvic.h"
//#include "systick.h"
//
//
//// Common interface includes
//#include "uart_if.h"
//
//#include "pin_mux_config.h"
//
//#include "Adafruit_SSD1351.h"
//#include "oled_test.h"
//#include "Adafruit_GFX.h"
//#include "glcdfont.h"
//#include "i2c_if.h"
//
//
//
//
////////////////////////////////////////////////////////////////////
////SYSTIC
////////////////////////////////////////////////////////////////////
//
//// some helpful macros for systick
//
//// the cc3200's fixed clock frequency of 80 MHz
//// note the use of ULL to indicate an unsigned long long constant
//#define SYSCLKFREQ 80000000ULL
//
//// macro to convert ticks to microseconds
//#define TICKS_TO_US(ticks) \
//    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
//    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\
//
//// macro to convert microseconds to ticks
//#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))
//
//// systick reload value set to 40ms period
//// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
//#define SYSTICK_RELOAD_VAL 8000000UL
//
//#define BUTTON_ONE -1749723041
//#define BUTTON_TWO 66219167
//#define BUTTON_THREE -1474275809
//#define BUTTON_FOUR 1106806847
//#define BUTTON_FIVE -433688129
//#define BUTTON_SIX 1382254079
//#define BUTTON_SEVEN -158240897
//#define BUTTON_EIGHT -936618033
//#define BUTTON_NINE 1817854287
//#define BUTTON_ZERO -209228065
//#define BUTTON_LAST -661170801
//#define BUTTON_ENTER  891055645
//
////////////////////////////////////////////////////////////////////
//
//
//
//
////////////////////////////////////////////////////////////////////
////OLED
////////////////////////////////////////////////////////////////////
//
//#define APPLICATION_VERSION     "1.4.0"
////************************************************
////
//// Application Master/Slave mode selector macro
////
//// MASTER_MODE = 1 : Application in master mode
//// MASTER_MODE = 0 : Application in slave mode
////
////*************************************************
//#define MASTER_MODE      1
//
//#define SPI_IF_BIT_RATE  100000
//#define TR_BUFF_SIZE     100
//
//#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
//#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"
//#define I2C_BASE                I2CA0_BASE
//#define SYS_CLK                 80000000
//#define FAILURE                 -1
//#define SUCCESS                 0
//#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
//#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
//                                   if (SUCCESS != iRetVal) \
//                                     return  iRetVal;}
////font size 3
//    //Next letter +=20
//   //next row +=30, prev row -=30
//   //right edge of screen = row_size * letter_size - letter_spacing = 90 (font 3)
//
//#define LETTER_SPACING      10
//#define LETTER_DIM          20
//#define LETTER_ROWSIZE      5
//#define LETTER_COLSIZE      2
//
//
////////////////////////////////////////////////////////////////////
//
//
//
//
////*****************************************************************************
////                 GLOBAL VARIABLES -- Start
////*****************************************************************************
//
//
////////////////////////////////////////////////////////////////////
////SYSTIC
////////////////////////////////////////////////////////////////////
//
//// track systick counter periods elapsed
//// if it is not 0, we know the transmission ended
//volatile int systick_cnt = 0;
//
//extern void (* const g_pfnVectors[])(void);
//
//extern void (* const g_pfnVectors[])(void);
//
////-----------------------------------------------------------------------------
//
//volatile unsigned long SW2_intcount;
//volatile unsigned char SW2_intflag;
//volatile unsigned long SW2_example;
//
//
//volatile uint64_t delta_previous;
//volatile uint64_t delta_current;
//volatile uint64_t delta_us;
//
////////////////////////////////////////////////////////////////////
//
//
//
////////////////////////////////////////////////////////////////////
////OLED
////////////////////////////////////////////////////////////////////
//static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
//static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
//static unsigned char ucTxBuffNdx;
//static unsigned char ucRxBuffNdx;
//
//#if defined(ccs)
//extern void (* const g_pfnVectors[])(void);
//#endif
//#if defined(ewarm)
//extern uVectorEntry __vector_table;
//#endif
////////////////////////////////////////////////////////////////////
//
//
////*****************************************************************************
////                 GLOBAL VARIABLES -- End
////*****************************************************************************
//
//
////*****************************************************************************
////                      LOCAL FUNCTION PROTOTYPES
////*****************************************************************************
//static void BoardInit(void);
//
//
////*****************************************************************************
////                      LOCAL FUNCTION DEFINITIONS
////*****************************************************************************
//
///**
// * Reset SysTick Counter
// */
//static inline void SysTickReset(void) {
//    // any write to the ST_CURRENT register clears it
//    // after clearing it automatically gets reset without
//    // triggering exception logic
//    // see reference manual section 3.2.1
//    HWREG(NVIC_ST_CURRENT) = 1;
//
//    // clear the global count variable
//    systick_cnt = 0;
//}
//
///**
// * SysTick Interrupt Handler
// *
// * Keep track of whether the systick counter wrapped
// */
//static void SysTickHandler(void) {
//    // increment every time the systick handler fires
//    systick_cnt++;
//}
//
//
//// an example of how you can use structs to organize your pin settings for easier maintenance
//typedef struct PinSetting {
//    unsigned long port;
//    unsigned int pin;
//} PinSetting;
//
//static PinSetting our_input = { .port = GPIOA0_BASE, .pin = 0x40};
//
//
//
//static void GPIOA2IntHandler(void) {
//    unsigned long ulStatus;
//
//    ulStatus = MAP_GPIOIntStatus (our_input.port, true);
//    MAP_GPIOIntClear(our_input.port, ulStatus);     // clear interrupts on GPIOA2
//    SW2_intcount++;
//    SW2_intflag=1;
//
//
//    // read the countdown register and compute elapsed cycles
//    delta_current = SYSTICK_RELOAD_VAL - SysTickValueGet();
//
//    // convert elapsed cycles to microseconds
//    delta_us = TICKS_TO_US(delta_current);
//
//    SysTickReset();
//
//}
////*****************************************************************************
//// REPLACED WITH OLED'S BOARDINIT FUNCTION
////! Board Initialization & Configuration
////!
////! \param  None
////!
////! \return None
////
////*****************************************************************************
////static void
////BoardInit(void) {
////    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
////
////    // Enable Processor
////    //
////    MAP_IntMasterEnable();
////    MAP_IntEnable(FAULT_SYSTICK);
////
////    PRCMCC3200MCUInit();
////}
//static void
//BoardInit(void)
//{
///* In case of TI-RTOS vector table is initialize by OS itself */
//#ifndef USE_TIRTOS
//  //
//  // Set vector table base
//  //
//#if defined(ccs)
//    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
//#endif
//#if defined(ewarm)
//    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
//#endif
//#endif
//    //
//    // Enable Processor
//    //
//    MAP_IntMasterEnable();
//    MAP_IntEnable(FAULT_SYSTICK);
//
//    PRCMCC3200MCUInit();
//
///////////ORIGINAL  BOARD INIT
////    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
////
////        // Enable Processor
////        //
////        MAP_IntMasterEnable();
////        MAP_IntEnable(FAULT_SYSTICK);
////
////        PRCMCC3200MCUInit();
//}
//
///**
// * Initializes SysTick Module
// */
//static void SysTickInit(void) {
//
//    // configure the reset value for the systick countdown register
//    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
//
//    // register interrupts on the systick module
//    MAP_SysTickIntRegister(SysTickHandler);
//
//    // enable interrupts on systick
//    // (trigger SysTickHandler when countdown reaches 0)
//    MAP_SysTickIntEnable();
//
//    // enable the systick module itself
//    MAP_SysTickEnable();
//}
//
//
//
////////////////////////////////////////////////////////////////////
////OLED
////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////
////FUNCTIONS WE DEFINED
////////////////////////////////////////////////////////////////////
//
////PUT BUTTON INTO A  READABLE FORMAT
////          need to check that the return value is greateer than -1before using output
////          last = 10, enter = 11, none = -1
//static int ButtonPressed(int b_data) {
//    int button = -1;
//
//    switch(b_data)  {
//        case(BUTTON_ONE) :
//                button = 1;
//        break;
//        case(BUTTON_TWO):
//                button = 2;
//        break;
//        case(BUTTON_THREE):
//                button = 3;
//        break;
//        case(BUTTON_FOUR):
//                button = 4;
//        break;
//        case(BUTTON_FIVE):
//                button = 5;
//        break;
//        case(BUTTON_SIX):
//                button = 6;
//        break;
//        case(BUTTON_SEVEN):
//                button = 7;
//        break;
//        case(BUTTON_EIGHT):
//                button = 8;
//        break;
//        case(BUTTON_NINE):
//                button = 9;
//        break;
//        case(BUTTON_ZERO):
//                button = 0;
//        break;
//        case(BUTTON_LAST):
//                button = 10;
//        break;
//        case(BUTTON_ENTER):
//                button = 11;
//        break;
//
//    }
//    return button;
//}
//
//
////puts button_readable into a leetter form
////      delete = 'D', send = 'S'
////DOESNT INCLUDE B0, B1
//static char ButtonToLetter(int b_read, int times_pressed) {
////    char let = 'A';
//    switch(b_read) {
//        case 2:
//            //a,  b, c
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 'a';
//            } else if(times_pressed  ==  2) {
//                return 'b';
//            } else if(times_pressed == 0) {
//                return'c';
//            }
//        break;
//        case 3:
//            //d, e, f
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 'd';
//            } else if(times_pressed  ==  2) {
//                return 'e';
//            } else if(times_pressed == 0) {
//                return 'f';
//            }
//        break;
//        case 4:
//            //g  h i
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 'g';
//            } else if(times_pressed  ==  2) {
//                return 'h';
//            } else if(times_pressed == 0) {
//                return 'i';
//            }
//        break;
//        case 5:
//            // j k l
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 'j';
//            } else if(times_pressed  ==  2) {
//                return 'k';
//            } else if(times_pressed == 0) {
//                return 'l';
//            }
//        break;
//        case  6:
//            // m n o
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 'm';
//            } else if(times_pressed  ==  2) {
//                return 'n';
//            } else if(times_pressed == 0) {
//                return 'o';
//            }
//        break;
//        case 7:
//            // p q r
//            times_pressed = times_pressed%4;
//            if(times_pressed == 1) {
//                return 'p';
//            } else if(times_pressed  ==  2) {
//                return 'q';
//            } else if(times_pressed == 3) {
//                return 'r';
//            } else if(times_pressed == 0) {
//                return 's';
//            }
//
//            break;
//        case 8:
//            // s  t u
//            times_pressed = times_pressed%3;
//            if(times_pressed == 1) {
//                return 't';
//            } else if(times_pressed  ==  2) {
//                return 'u';
//            } else if(times_pressed == 0) {
//                return 'v';
//            }
//            break;
//        case 9:
//            // v w x
//            times_pressed = times_pressed%4;
//            if(times_pressed == 1) {
//                return 'w';
//            } else if(times_pressed  ==  2) {
//                return 'x';
//            } else if(times_pressed == 3) {
//                return 'y';
//            } else if(times_pressed == 0) {
//                return 'z';
//            }
//        break;
//
//        case  10:       //delete
//            return 'D';
//        break;
//        case 11:        //send
//            return 'S';
//        break;
//    }
//    return ' ';
//}
//
//
//
//// display  the correect letter in the correct space
//// DOESN'T INCLUDE SEND
//static void DisplayLetter(int* x, int* y, char let, int presses) {
//
//    int right_edge_screen = (LETTER_ROWSIZE*LETTER_DIM) - (LETTER_SPACING);
//
//    if(let == 'D') {
//        drawChar(*x, *y, let, WHITE, WHITE, 3);             //clear current letter
//
//        //find next coordinates
//        if (*x == LETTER_DIM) {                             //left edge of screen
//            if(*y != LETTER_SPACING) {                      //not the  first row, not the first letter ever
//                //right edge of screen = row_size * letter_size - letter_spacing = 90 (font 3)
//                *x = right_edge_screen;
//                *y = *y - (LETTER_DIM + LETTER_SPACING);
//            }
//        } else {                                            //letter not at left edge
//            *x -= LETTER_DIM;
//        }
//    } else if(presses == 1) {                               //new button = next space
//        if (*x == right_edge_screen) {
//            if(*y < LETTER_COLSIZE) {
//                *x = LETTER_DIM;
//                *y += (LETTER_DIM + LETTER_SPACING);
//            }
//        } else {                                            //letter not at RIGHT edge
//            if((*x != LETTER_DIM) && (*y != LETTER_SPACING)) {      // not first  letter ever
//                *x += LETTER_DIM;
//            }
//        }
//        drawChar(*x, *y, let, BLUE, WHITE, 3);
//    } else if(presses>1) {                                  //same button pressed
//        drawChar(*x, *y, let, BLUE, WHITE, 3);
//    }
//
//}
//
//
//
//
////*****************************************************************************
////
////! SPI Slave Interrupt handler
////!
////! This function is invoked when SPI slave has its receive register full or
////! transmit register empty.
////!
////! \return None.
////
////*****************************************************************************
//static void SlaveIntHandler()
//{
//    unsigned long ulRecvData;
//    unsigned long ulStatus;
//
//    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);
//
//    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
//
//    if(ulStatus & SPI_INT_TX_EMPTY)
//    {
//        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
//        ucTxBuffNdx++;
//    }
//
//    if(ulStatus & SPI_INT_RX_FULL)
//    {
//        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
//        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
//        Report("%c",ulRecvData);
//        ucRxBuffNdx++;
//    }
//}
//
////*****************************************************************************
////
////! SPI Master mode main loop
////!
////! This function configures SPI modelue as master and enables the channel for
////! communication
////!
////! \return None.
////
////*****************************************************************************
//void MasterMain()
//{
//    //
//    // Set Tx buffer index
//    //
//    ucTxBuffNdx = 0;
//    ucRxBuffNdx = 0;
//
//    //
//    // Reset SPI
//    //
//    MAP_SPIReset(GSPI_BASE);
//
//    //
//    // Configure SPI interface
//    //
//    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
//                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
//                     (SPI_SW_CTRL_CS |
//                     SPI_4PIN_MODE |
//                     SPI_TURBO_OFF |
//                     SPI_CS_ACTIVEHIGH |
//                     SPI_WL_8));
//
//    //
//    // Enable SPI for communication
//    //
//    MAP_SPIEnable(GSPI_BASE);
//
//    // MOSI = channel 0
//    // MISO = channel 1
//    // clock = channel 2
//    // enable (OC) = channel 3
//
//
//   Adafruit_Init();
//
//
//   int button_data = 0;
//   int button_readable  = -1;
//   int prev_b_readable = -1;
//   char letter = 'B';
//   int num_press = 1;
//
//   int curr_coord_x  = LETTER_SPACING;
//   int curr_coord_y =  LETTER_DIM;
//
//
//   while (1) {
//       fillScreen(WHITE);
//
//       if (SW2_intflag) {
//           SW2_intflag=0;
//           int msb = 0;
//
//           msb = delta_us/1000;        //should truncate
//
//           if(msb>0) {
//
//               if(msb == 1) {              //bit=1
//                   button_data = button_data*10;
//               }
//               else if (msb == 2) {        //bit =2
//                   button_data = (button_data*10) + 1;
//               }
//               else {                      //extra signal we don't need
//
//                   if(button_data!=0)  {           //if we got a readable button press
//
//                       button_readable = ButtonPressed(button_data);
//
//                       if(button_readable>=0) {    //if  the button pressed was one of the ones we are using
////                            Report("button read: %d\n", button_readable);
//                           if(prev_b_readable == button_readable) {    //find number  of consecutive presses
//                               num_press +=1;
//                           } else {
//                               num_press = 1;
//                           }
//
//                           letter = ButtonToLetter(button_readable, num_press);
//                           Report("%c", letter);
//
////                           DisplayLetter(&curr_coord_x, &curr_coord_y, letter, num_press);
////                           Report("x coord: %d\n", curr_coord_x);
////                           prev_b_readable = button_readable;
//                       }
//                   }
//
//
//                   button_data = 0;
//                   button_readable = -1;
//               }
//
//           }
//
//       }
//   }
//
//
////    fillScreen(WHITE);
////    int count = 0;
////    while(true){
////        while(count < 1270 ){
////               drawChar(50, 50, font[count],RED, WHITE, 2);
////               count++;
////           }
////
////           fillScreen(WHITE);
////
//
////           drawChar(10, 20, 'H',BLUE, WHITE, 3);
////           drawChar(30, 20, 'E',GREEN, WHITE, 3);
////           drawChar(50, 20, 'L',YELLOW, WHITE, 3);
////           drawChar(70, 20, 'L',MAGENTA, WHITE, 3);
////           drawChar(90, 20, 'O',RED, WHITE, 3);
////           drawChar(10, 50, 'W',RED, WHITE, 3);
////           drawChar(30, 50, 'O',MAGENTA, WHITE, 3);
////           drawChar(50, 50, 'R',YELLOW, WHITE, 3);
////           drawChar(70, 50, 'L',GREEN, WHITE, 3);
////           drawChar(90, 50, 'D',BLUE, WHITE, 3);
////
////           lcdTestPattern();
////           lcdTestPattern2();
////
////           testlines(WHITE);
////           testfastlines(RED, BLUE);
////           testdrawrects(MAGENTA);
////           testfillrects(WHITE, BLACK);
////           testfillcircles(5, YELLOW);
////           testdrawcircles(10, RED);
////           testroundrects();
////           testtriangles();
////    }
//
//
//
////
////    fillScreen(WHITE);
////    fillCircle(64, 64, 3, RED);
////
////    I2C_IF_Open(I2C_MASTER_MODE_FST);
////
////    unsigned char ucDevAddr = 0x18;
////    unsigned char ucWrLen = 1;
////    unsigned char ucRdLen = 1;
////    unsigned char x_axis = 0x03;
////    unsigned char y_axis = 0x05;
////    unsigned char pucRdDataBuf_X[256];
////    unsigned char pucRdDataBuf_Y[256];
////    unsigned char pucWrDataBuf_X[1] = {x_axis};
////    unsigned char pucWrDataBuf_Y[1] = {y_axis};
//
//
////  Read X-axis data
////    while(1){
//
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
////        Report("%d\n", int_x_acc);
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
//
//        // x cannot be smaller than 3
//        // x cannot be greater than 124
//
//        // y cannot be smaller than 3
//        // y cannot be greater than 124
////    }
//}
//
////*****************************************************************************
////
////! SPI Slave mode main loop
////!
////! This function configures SPI modelue as slave and enables the channel for
////! communication
////!
////! \return None.
////
////*****************************************************************************
//void SlaveMain()
//{
//
//    //
//    // Set Tx buffer index
//    //
//    ucTxBuffNdx = 0;
//    ucRxBuffNdx = 0;
//
//    //
//    // Reset SPI
//    //
//    MAP_SPIReset(GSPI_BASE);
//
//    //
//    // Configure SPI interface
//    //
//    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
//                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
//                     (SPI_HW_CTRL_CS |
//                     SPI_4PIN_MODE |
//                     SPI_TURBO_OFF |
//                     SPI_CS_ACTIVEHIGH |
//                     SPI_WL_8));
//
//    //
//    // Register Interrupt Handler
//    //
//    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);
//
//    //
//    // Enable Interrupts
//    //
//    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);
//
//    //
//    // Enable SPI for communication
//    //
//    MAP_SPIEnable(GSPI_BASE);
//
//    //
//    // Print mode on uart
//    //
//    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
//}
//
//
//
//
////****************************************************************************
////
////! Main function
////!
////! \param none
////!
////!
////! \return None.
////
////****************************************************************************
//int main() {
//    unsigned long ulStatus;
//
//    BoardInit();
//
//    PinMuxConfig();
//
//    //
//    // Enable the SPI module clock
//    //
//    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
//
//    // Enable SysTick
//    SysTickInit();
//
//    // Initialize UART terminal
//    InitTerm();
//
//    // Clear UART Terminal
//    ClearTerm();
//
//    //
//    // Register the interrupt handlers
//    //
//
//    MAP_GPIOIntRegister(our_input.port, GPIOA2IntHandler);
//
//    //
//    // Configure fallling edge interrupts on button presses
//    //
//    MAP_GPIOIntTypeSet(our_input.port, our_input.pin, GPIO_FALLING_EDGE);    // our input
//
//
//    ulStatus = MAP_GPIOIntStatus(our_input.port, false);
//    MAP_GPIOIntClear(our_input.port, ulStatus);         // clear interrupts on our input GPIO
//
//    // clear global variables
//    SW2_example=0;
//    SW2_intcount=0;
//    SW2_intflag=0;
//    delta_current = 0;
//    delta_us = 0;
//
//    MAP_GPIOIntEnable(our_input.port, our_input.pin);
//
////    SW2_example = MAP_GPIOPinRead(our_input.port, our_input.pin);
//
//    Message("\t\t****************************************************\n\r");
//    Message("\t\t\t LAB 3\n\r");
//    Message("\t\t ****************************************************\n\r");
//    Message("\n\n\n\r");
//
//    //
//    // Reset the peripheral
//    //
//    MAP_PRCMPeripheralReset(PRCM_GSPI);
//
//    #if MASTER_MODE
//
//        MasterMain();
//
//    #else
//
//        SlaveMain();
//
//    #endif
//
//        while(1)
//        {
//
//        }
//
//
//}
//
////*****************************************************************************
////
//// Close the Doxygen group.
////! @}
////
////*****************************************************************************
