/*******************************************************************************
 *
 * ECE 431: Lab 8 Shell
 *
 ******************************************************************************/
#include <driverlib.h>

// Change based on LCD Memory locations
#define pos1 9   /* Digit A1 begins at S18 */
#define pos2 5   /* Digit A2 begins at S10 */
#define pos3 3   /* Digit A3 begins at S6  */
#define pos4 18  /* Digit A4 begins at S36 */
#define pos5 14  /* Digit A5 begins at S28 */
#define pos6 7   /* Digit A6 begins at S14 */

// LCD memory map for numeric digits
const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

/*
 * TimerA0 UpMode Configuration Parameter
 * Use this struct to set up a 15ms timer
 */
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        50000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

/*
 * TimerB0 UpMode Configuration Parameter
 * Use this struct to set up a 15ms timer
 */
Timer_B_initUpModeParam initUpParam_B0 =
{
        TIMER_B_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_B_CLOCKSOURCE_DIVIDER_1,          // SMCLK/4 = 2MHz
        50000,                                  // 15ms debounce period
        TIMER_B_TBIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_B_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

Calendar currentTime;

// TODO: LAB 9 - Declare states using an enum
enum TIME_SET_STATES{STATE_IDLE,
    STATE_SET_HOURS,
    STATE_SET_MINUTES,
    STATE_SET_SECONDS,
    STATE_SET_TIME}TIME_SET_STATE;

// Function foreward declarations
void show_digit(int, int);
void set_time(int, int, int);
void display_time(void);

void setTheTime(void);

void init_GPIO(void);
void init_LCD(void);

// Button debounce variables
volatile unsigned char S1buttonDebounce = 0;
volatile unsigned char S2buttonDebounce = 0;

// TODO: LAB 9 - Add volatile unsigned char(s)/int(s) which are flags and counters for your state machine
volatile unsigned char setTimeRequest = 0;
volatile unsigned char nextStateFlag = 0;
volatile unsigned int buttonCounter = 0;
volatile unsigned int buttonHeldCounter = 0;

// TODO: LAB 9 - Declare some variables to hold hours, minutes, and seconds
int hours = 0;
int minutes = 0;
int seconds = 0;

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Initialize the input and output pins (GPIO)
    init_GPIO();

    // Initialize LCD
    init_LCD();

    // Setup counter, clock, and initial set time
    set_time(23,59,59);

    // TODO: LAB 9 - Initialize state machine
    TIME_SET_STATE = STATE_IDLE; // setup state machine

    //enable interrupts
    __enable_interrupt();


    // An infinite loop
    while(1)
    {
        // Update displayed time.
        display_time();

        // TODO: LAB 9 - Check flags, call function that sets the time
        if(setTimeRequest){
            setTheTime();
        }

    }
}

// TODO: LAB 9 - Function that sets the states and what happens in each
void setTheTime(){
    switch(TIME_SET_STATE){
        case STATE_IDLE:
                if(!setTimeRequest){
                    TIME_SET_STATE = STATE_IDLE;

                    buttonCounter = 0;
                }else{
                    TIME_SET_STATE = STATE_SET_HOURS;

                    nextStateFlag = 0;
                    buttonCounter = 0;
                }
            break;
        case STATE_SET_HOURS:
            if(nextStateFlag){
                set_time(hours, minutes, seconds);

                nextStateFlag = 0;
                buttonCounter = 0;

                TIME_SET_STATE = STATE_SET_MINUTES;
            }else{
                hours = buttonCounter;
                set_time(hours, minutes, seconds);
            }
            break;
        case STATE_SET_MINUTES:
            if(nextStateFlag){
                set_time(hours, minutes, seconds);

                nextStateFlag = 0;
                buttonCounter = 0;

                TIME_SET_STATE = STATE_SET_SECONDS;
            }else{
                minutes = buttonCounter;
                set_time(hours, minutes, seconds);
            }
            break;
        case STATE_SET_SECONDS:
            if(nextStateFlag){
                set_time(hours, minutes, seconds);

                buttonCounter = 0;
                nextStateFlag = 0;

                TIME_SET_STATE = STATE_SET_TIME;
            }else{
                seconds = buttonCounter;
                set_time(hours, minutes, seconds);
            }
            break;
        case STATE_SET_TIME:
            RTC_C_startClock(RTC_C_BASE);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

            buttonCounter = 0;
            nextStateFlag = 0;
            setTimeRequest = 0;

            set_time(hours, minutes, seconds);

            TIME_SET_STATE = STATE_IDLE;
            break;
        default:
            TIME_SET_STATE = STATE_IDLE;
            break;
    }
}

// TODO: LAB 9 - Make sure you only start the clock if we are not in the mode of setting the time manually,
//              as we only want to start the clock if we have finished setting the time.
void set_time(int hours, int minutes, int seconds)
{
    // "Pause" the clock
    RTC_C_holdClock(RTC_C_BASE);

    // Set the struct
    currentTime.Seconds = seconds;
    currentTime.Minutes = minutes;
    currentTime.Hours   = hours;
    currentTime.DayOfWeek = 0x04;
    currentTime.DayOfMonth = 0x30;
    currentTime.Month = 0x04;
    currentTime.Year = 0x2015;

    // Set the clock parameters
    RTC_C_initCalendar(RTC_C_BASE,
                       &currentTime, RTC_C_FORMAT_BINARY);
    RTC_C_initCounter(RTC_C_BASE,
                      RTC_C_CLOCKSELECT_32KHZ_OSC, RTC_C_COUNTERSIZE_16BIT);

    // Display initial time
    display_time();

    // Start clock
    if(!setTimeRequest){
        RTC_C_startClock(RTC_C_BASE);
    }

}

void show_digit(int d, int position)
{
    // Write digits to the LCD
    LCDMEM[position] = digit[d][0];
    LCDMEM[position+1] = digit[d][1];
}
/*
 * Retrieves the currentTime from the counter and
 * displays it on the LCD
 */
void display_time()
{
    // Retrieve time
    currentTime = RTC_C_getCalendarTime(RTC_C_BASE);

    // Display the time. %10 gets the 1s place, /10 gets the 10ths place
    show_digit((currentTime.Seconds) % 10,pos6);
    show_digit((currentTime.Seconds) / 10,pos5);
    show_digit((currentTime.Minutes) % 10,pos4);
    show_digit((currentTime.Minutes) / 10,pos3);
    show_digit((currentTime.Hours) % 10  ,pos2);
    show_digit((currentTime.Hours) / 10  ,pos1);

    // Display the 2 colons
    LCDM7 |= 0x04;
    LCDM20 |= 0x04;
}



/*
 * PORT1 Interrupt Service Routine
 * Handles S1 and S2 button press interrupts
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_P1IFG1 :    // Button S1 pressed
            /* TODO:
             * Handle button debouncing
             * Toggle LED and start/stop clock
             */

            if (S1buttonDebounce == 0)
            {
                // Set debounce flag on first high to low transition
                S1buttonDebounce = 1;

                if(setTimeRequest == 0){
                    setTimeRequest = 1;

                    RTC_C_holdClock(RTC_C_BASE);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

                }else if(setTimeRequest == 1){

                    // Toggle the state flag
                    if(nextStateFlag == 0){
                        nextStateFlag = 1;
                    }

                }

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
            break;
        case P1IV_P1IFG2 :    // Button S2 pressed

            if (S2buttonDebounce == 0) {

                S2buttonDebounce = 1;

                GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN7);
                Timer_B_initUpMode(TIMER_B0_BASE, &initUpParam_B0);
            }

            break;
        default: break;
    }
}

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{
    // TODO: Handle button debouncing
    // Turn off the timer interrupt once button is released

    // Button S1 released
    if (P1IN & BIT1)
    {
        S1buttonDebounce = 0; // Clear button 1 debounce
        Timer_A_stop(TIMER_A0_BASE);
    }

}

/*
 * Timer B0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR (void)
{
    // TODO: LAB 9 - Handle button debouncing
    // Turn off the timer interrupt once button is released
    // Increment count that is used in the state machine

    // Button S2 released
    if (P1IN & BIT2) {
        S2buttonDebounce = 0; // Clear button 2 debounce

        buttonHeldCounter = 0;

        GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN7);
        Timer_B_stop(TIMER_B0_BASE);

        if(TIME_SET_STATE == STATE_SET_HOURS){
            if(buttonCounter < 23){
                buttonCounter++;
            }else{
                buttonCounter = 0;
            }
        }else{
            if(buttonCounter < 59){
                buttonCounter++;
            }else{
                buttonCounter = 0;
            }
        }
    }else{
        buttonHeldCounter++;

        if(buttonHeldCounter > 10){

            if(TIME_SET_STATE == STATE_SET_HOURS){

                if(buttonCounter < 23){
                    buttonCounter++;
                }else{
                    buttonCounter = 0;
                }

            }else{

                if(buttonCounter < 59){
                    buttonCounter++;
                }else{
                    buttonCounter = 0;
                }

            }
        }
    }

}

/*
 * GPIO Initialization
 */
void init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating
    // input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, 0xFF);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, 0xFF);

    GPIO_setAsOutputPin(GPIO_PORT_P1, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P2, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P3, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P4, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P5, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P6, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P7, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P8, 0xFF);
    GPIO_setAsOutputPin(GPIO_PORT_P9, 0xFF);

    // Configure button S1 (P1.1) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S2 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}


void init_LCD()
{
    LCD_C_initParam initParams = {0};
    initParams.clockSource = LCD_C_CLOCKSOURCE_ACLK;
    initParams.clockDivider = LCD_C_CLOCKDIVIDER_1;
    initParams.clockPrescalar = LCD_C_CLOCKPRESCALAR_16;
    initParams.muxRate = LCD_C_4_MUX;
    initParams.waveforms = LCD_C_LOW_POWER_WAVEFORMS;
    initParams.segments = LCD_C_SEGMENTS_ENABLED;

    LCD_C_init(LCD_C_BASE, &initParams);
    // LCD Operation - VLCD generated internally, V2-V4 generated internally, v5 to ground

    /*  'FR6989 LaunchPad LCD1 uses Segments S4, S6-S21, S27-S31 and S35-S39 */
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_4,
                                LCD_C_SEGMENT_LINE_4);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_6,
                                LCD_C_SEGMENT_LINE_21);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_27,
                                LCD_C_SEGMENT_LINE_31);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_35,
                                LCD_C_SEGMENT_LINE_39);

    LCD_C_setVLCDSource(LCD_C_BASE, LCD_C_VLCD_GENERATED_INTERNALLY,
                        LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS,
                        LCD_C_V5_VSS);

    // Set VLCD voltage to 3.20v
    LCD_C_setVLCDVoltage(LCD_C_BASE,
                         LCD_C_CHARGEPUMP_VOLTAGE_3_02V_OR_2_52VREF);

    // Enable charge pump and select internal reference for it
    LCD_C_enableChargePump(LCD_C_BASE);
    LCD_C_selectChargePumpReference(LCD_C_BASE,
                                    LCD_C_INTERNAL_REFERENCE_VOLTAGE);

    LCD_C_configChargePump(LCD_C_BASE, LCD_C_SYNCHRONIZATION_ENABLED, 0);

    // Clear LCD memory
    LCD_C_clearMemory(LCD_C_BASE);

    //Turn LCD on
    LCD_C_on(LCD_C_BASE);

    LCD_C_selectDisplayMemory(LCD_C_BASE, LCD_C_DISPLAYSOURCE_MEMORY);
}
