#include <stdint.h>
#include <stm32l432xx.h>
#include <stdio.h>
#include <errno.h>
#include "stm32l4xx.h"
#include "eeng1030_lib.h"
#include "display.h"
#include "spi.h"
#include <sys/unistd.h>  // STD FILE OUT, STD FILE IN

// SUMMARY:
// NOTE: ALL OF THIS CIRCUIT USES 3.3V

// This code allows the user to program an 8-step sequencer using 2 buttons and a potentiometer as inputs
// in a circuit. The button connected to PB4 is the pause button, and should show a pause message on the
// ST7735 when it is pressed,enable a red LED connected to PB3, and stop the sequencer from progressing 
// or playing through notes. This button is a toggle, so it must be pressed again to unpause the sequencer.

// The second button is connected to PB5 and it is used in conjunction with the potentiometer to change
// the tone played through the buzzer. To use this, adjust the potentiometer (the screen should display
// the current value) and when ready, press the button. This will change the current step in the
// sequencer to play at the set duty cycles note. There is 8 steps in total and each time the button is
// pressed it will adjust the next steps duty cycle.

// The potentiometer must be wired to ground on its third pin to achieve a full range of values in the ADC.
// The middle pin of the potentiometer is wired to PA0 which is where the ADC will convert the analogue
// signal into a value in the microcontroller. Using the ADC, the ADC will output values between 0 and
// 4095. This is fed into the ARR of TIM2 which will adjust the PWM.

// The sequencer produces tones by oscillating the buzzer with Pulse Width Modulation through PA9.

// The ST7735 LCD recieves its SPI clock through PA1 and it receives MOSI data through PA7. The Reset pin
// should be wired to PA6, DC should be wired to PA5, and CS should be wired to PA4. BLK is the backlight
// and this can be wired to the +3.3V rail. It is important to note that the wires for the ST7735 are
// required to be very short as they are transferring high amounts of data. If the wires are too long, it
// may not work.

// CODE:
#define NUM_STEPS 8 // Number of steps in the Duty Cycle Array
#define SCREEN_WIDTH 160 // ST7735 Screen Width
#define SCREEN_HEIGHT 80 // ST7735 Screen Height

// Arrays
// The stepVal array is initalized with values to help understand the order of the sequencer
// They will be adjusted once the updateDutyCycle state is active.
volatile uint32_t stepVal[NUM_STEPS] = {4000,2000,0,2000,4000,2000,0,2000};  // Duty Cycle array

// Initial Variables
volatile uint8_t pauseState = 0;  // Pause Flag (0 is not pressed, 1 is pressed)
volatile uint8_t dutyAdjust_Idx = 0; // Counting variable used to cycle through the duty cycle array
volatile uint8_t dutyIdx = 0; // Counting Variable to play next duty cycle in sequence
volatile int newVal; // SysTick Handler variable for updating the PWM
volatile uint32_t tickcntr = 0; // SysTick Counter Variable
volatile int updateDutyCycle=0; // Updating Duty Cycle Flag (0 is not updating, 1 is updating)
volatile uint32_t milliseconds;

// Setup
void setup(void);

// USART (for debugging with printf)
void initSerial(uint32_t baudrate);
void eputc(char c);

// PWM
void setTimer1Duty(int duty); // Function to change the current Duty Cycle
void initTimer1(void); // Initialize TIM1 for PWM

// ADC
int readADC(int chan); // Reads the ADC value from a select channel
void initADC(); // Initialize the ADC

// Delays
void delay(volatile uint32_t dly); // Tick delay

// Scales ADC value so that it is shown as between 1 and 100
int scaleValue(int value);

// Interrupt handler for adjusting duty cycle array
void EXTI9_5_IRQHandler();

// changes:
//not using tim6 anymore, using systick instead
// changed value of arr to initially be much higher so as to reduce the frequency of the buzzer

int main() {
    setup();

    // Clear the display with a black rectangle
    fillRectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, RGBToWord(0, 0, 0));

    while (1) {
            // State 1 (Play)

            // Print milliseconds through serial for debugging SysTick (uncomment to use)
            // printf("%ld \n",milliseconds);

            // Split screen into two sides with line
            int lineXPos = 85; // X position for line
            drawLine(lineXPos,0,lineXPos,80,RGBToWord(255,255,255));

            // Set Values Side of LCD (from x = 0 to x = 74)
            // These will be the current set duty for each step in the array
            // NOTE: All displayed ADC values are converted to be between 0 and 100 percent so that
            // it is more intuitive to change the duty cycle for each step
            int d1Val = stepVal[0];
            int d2Val = stepVal[1];
            int d3Val = stepVal[2];
            int d4Val = stepVal[3];
            int d5Val = stepVal[4];
            int d6Val = stepVal[5];
            int d7Val = stepVal[6];
            int d8Val = stepVal[7];

            // There is not enough space on the ST7735 to display all the steps at once, so using
            // the SysTick Interrupt the microcontroller is cycling between displaying steps 1-4
            // and steps 5-8 on the right hand side of the screen. It currently switches every 500ms.
            if(tickcntr < 500){
                printText("Step1:", 5, 10, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step2:", 5, 20, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step3:", 5, 30, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step4:", 5, 40, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d1Val), 48, 10, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d2Val), 48, 20, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d3Val), 48, 30, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d4Val), 48, 40, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
            } else {
                printText("Step5:", 5, 10, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step6:", 5, 20, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step7:", 5, 30, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printText("Step8:", 5, 40, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d5Val), 48, 10, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d6Val), 48, 20, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d7Val), 48, 30, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
                printNumber(scaleValue(d8Val), 48, 40, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
            }

            // This is the display commands for the adjusting (RHS) side of the screen so that it is easy
            // to see which step is currently being adjusted by the potentiometer. It also displays the
            // current ADC value in percentage so that it can be used to set the duty more accurately.
            // Adjusting Side of LCD (From x =76 to x = 160)
            int adjustCurVal = readADC(5);
            int rhsXPos = 90; // Adjusts adjusting side of LCD X starting point
            printText("Adjusting:", rhsXPos, 10, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
            printText("Step #", rhsXPos, 20, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
            printNumber(dutyAdjust_Idx+1, rhsXPos, 30, RGBToWord(255,255,255),RGBToWord(0,0,0));
            printText("Duty:", rhsXPos, 40, RGBToWord(255,255,255),RGBToWord(0,0,0));
            printNumber(scaleValue(adjustCurVal), rhsXPos, 50, RGBToWord(255,255,255), RGBToWord(0,0,0));
            delay_ms(500);  // Small delay before updating again

            // States 2 and 3 are both if statements here. 
            // State 2 is updateDutyCycle and it is active whenever
            // a SysTick interrupt occurs, which is when it will update the current played step from the
            // sequencer.
            // State 3 is the pauseState which is active when the button connected to PB4 is pressed causing pauseState
            // become active. When it is active, it stops the tickcntr from incrementing and disabled TIM1.
            // This effectively pauses the sequencer. When this occurs the text "PAUSED" will appear in the
            // bottom left of the LCD. This state is exited by pressing the button again which will toggle
            // the state and clear the PAUSED text in the bottom left with a black rectangle.

            if(pauseState == 0) // If not paused
            {
                TIM1->CR1 |= (1<<0); // Enable TIM1
                fillRectangle(4, 50, 100, 40, RGBToWord(0, 0, 0));  // Clear pause text
            }
            else if(pauseState == 1) //if paused
            {
                TIM1->CR1 &= ~(1<<0); // Disable TIM1
                printTextX2("PAUSED", 4, 60, RGBToWord(255, 255, 255), RGBToWord(0, 0, 0));
            }

            if(updateDutyCycle==1) // Updating Duty Cycle
            {
                stepVal[dutyAdjust_Idx] = readADC(5); // Insert current ADC value into array
                dutyAdjust_Idx=(dutyAdjust_Idx+1)%NUM_STEPS; // Incremement dutyAdjust Idx by one in modulo 4
                updateDutyCycle = 0; // Reset update duty cycle flag
            }
        }        
    }

void setup()
{
    initClocks();  // Initialize system clocks
    SysTick->LOAD = 80000-1; //SysTick CLK = 80MHz. 80000000/80000=1kHz
    SysTick->CTRL = 7; // Enable SysTick counter and interrupts
    __asm(" cpsie i "); // Enable global interrupts

    RCC->AHB2ENR |= (1 << 0) | (1 << 1);  // Enable GPIOA and GPIOB

    initSerial(9600); // Initialize Serial Communications with a baud rate of 9600
    init_display();  // Initialize Peripheral LCD

    // Configure GPIO pins
    pinMode(GPIOB, 4, 0);  // PB4 is input for pause button
    pinMode(GPIOB, 3, 1);  // PB3 outputs to red LED (pause state)
    pinMode(GPIOA, 0, 3);  // Pot 1 Analog Input (ADC)
    pinMode(GPIOB, 5, 0); // PB5 is the input for the adjust duty cycle button

    enablePullUp(GPIOB, 4); // Pull-up enabled for PB4
    enablePullUp(GPIOB, 5); // Pull-up enabled for PB5

    initADC();  // Start ADC
    initTimer1();  // Initialize PWM for buzzer

    // Configure EXTI4 for pause button
    RCC->APB2ENR |= (1 << 0);  // Enable SYSCFG
    SYSCFG->EXTICR[1] &= ~(7 << 0); // Clear EXTI4 bits using 0b111
    SYSCFG->EXTICR[1] |= (1 << 0);  // Map EXTI4 interrupt to PB4
    EXTI->FTSR1 |= (1 << 4);  // Falling edge trigger
    EXTI->IMR1 |= (1 << 4);  // Enable EXTI4
    NVIC->ISER[0] |= (1 << 10);  // IRQ10 maps to EXTI4

    // Configure EXTI5 for adjust duty cycle button
    SYSCFG->EXTICR[1] &= ~(7<<4); //CLEAR EXTI5 bits using 0b111
    SYSCFG->EXTICR[1] |= (1<<4); // Set EXTI5 interrupt to PB5
    EXTI->FTSR1 |= (1<<5); // Make EXTI5 falling edge trigger
    EXTI->IMR1 |= (1<<5); // Enable EXTI5 by Masking it as an interrupt
    NVIC_EnableIRQ(EXTI9_5_IRQn); // Map IRQ to EXTI5
    __enable_irq(); // Enable external line interrupts
}

// EXTI5 Interrupt Handler (Adjust Duty Cycle Button)
void EXTI9_5_IRQHandler(void)
{
    if ((GPIOB->IDR & (1 << 5)) == 0) {  // Check if button is pressed
        delay(10000);  // Debounce delay
        if ((GPIOB->IDR & (1 << 5)) == 0) {  // Confirm stable press
            updateDutyCycle=1; // Set update duty cycle flag so interrupt is quick
        }
    }
    EXTI->PR1 |= (1 << 5);  // Clear EXTI5 interrupt flag
}

// EXTI4 Interrupt Handler (Pause Button)
void EXTI4_IRQHandler() 
{
    if ((GPIOB->IDR & (1 << 4)) == 0) {  // Check if button is pressed
        delay(10000);  // Debounce delay
        if ((GPIOB->IDR & (1 << 4)) == 0) {  // Confirm stable press
            pauseState = !pauseState;  // Toggle pause state
            GPIOB->ODR ^= (1 << 3);  // Toggle Red Pause LED (PB3)
        }
    }
    EXTI->PR1 |= (1 << 4);  // Clear EXTI4 interrupt flag
}

// Initialize UART for Debug Output
void initSerial(uint32_t baudrate) 
{
    RCC->AHB2ENR |= (1 << 0);  // Enable GPIOA clock
    pinMode(GPIOA, 2, 2);  // Set PA2 as alternate function mode
    selectAlternateFunction(GPIOA, 2, 7);  // AF7 = USART2 TX

    RCC->APB1ENR1 |= (1 << 17);  // Enable USART2

    const uint32_t CLOCK_SPEED = 80000000;
    uint32_t BaudRateDivisor = CLOCK_SPEED / baudrate;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = (1 << 12);  // Disable overrun errors
    USART2->BRR = BaudRateDivisor;
    USART2->CR1 = (1 << 3);  // Enable transmitter
    USART2->CR1 |= (1 << 0);
}

// Redirect `printf()` to UART
int _write(int file, char *data, int len) 
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    while (len--) {
        eputc(*data);
        data++;
    }
    return 0;
}

void eputc(char c)
{
    while ((USART2->ISR & (1 << 6)) == 0);  // Wait for transmission to finish
    USART2->TDR = c;
}

void initTimer1(void){
    // Enable TIM1 and GPIO clocks
    pinMode(GPIOA,9,2);
    GPIOA->AFR[1] &= ~(0xF << (4 * (9-8))); // Clear 
    GPIOA->AFR[1] |= (1 << (4 * (9-8))); // Set AF1 for PA9 TIM1_CH3
    RCC->APB2ENR |= (1<<11); // TIM1 Enable

    // Config timer
    TIM1->CR1 = 0; // Turn off tim1 before config

    TIM1->CCMR1 = (0b110 << 12) + (1<<11) + (1<<10); // (CH2) Set output compare mode, preload, and fast enable
    TIM1->CCER |= (1<<12); // Set capture/compare 4 output enable

    TIM1->CCMR1 &= ~(0x7 << TIM_CCMR1_OC2M_Pos);  // Clear mode bits
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);     // Set PWM Mode 1 (110)
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;  // Enable preload register

    TIM1->PSC = 80 - 1; // Prescale
    TIM1->ARR = 12000-1; // Use 12000 to ensure that frequency is audible
    int hARR = TIM1->ARR; // Store ARR value in variable
    TIM1->CCR2 = hARR/2; // Start with CCR4 at 50% duty cycle

    TIM1->CCER |= (1<<4); // Enable pwm pin
    TIM1->BDTR |= (1<<15); // Enable advanced timers
    TIM1->CR1 |= (1<<0); // Enable timer 1
}
void setTimer1Duty(int duty)
{
    int arrvalue=(duty*TIM1->ARR)/4095;
    TIM1->CCR2=arrvalue; // Compare with ARR to change its value
}

void initADC()
{
    // Initialize the ADC
    RCC->AHB2ENR |= (1 << 13); // Enable the ADC
    RCC->CCIPR |= (1 << 29) | (1 << 28); // Select system clock for ADC
    ADC1_COMMON->CCR = ((0b01) << 16) + (1 << 22) ; // Set ADC clock = HCLK and turn on the voltage reference
    // Start ADC calibration    
    ADC1->CR=(1 << 28); // Turn on the ADC voltage regulator and disable the ADC
    delay_ms(100); // Wait for voltage regulator to stabilize (20 microseconds according to the datasheet).  This gives about 180microseconds
    ADC1->CR |= (1<< 31);
    while(ADC1->CR & (1 << 31)); // Wait for calibration to finish.
    ADC1->CFGR = (1 << 31); // Disable injection
    ADC1_COMMON->CCR |= (0x0f << 18);
}

int readADC(int chan)
{
    ADC1->SQR1 |= (chan << 6); // Set channel
    ADC1->ISR = (1 << 3); // Clear EOS flag
    ADC1->CR |= (1 << 0); // Enable the ADC
    while ( (ADC1->ISR & (1 <<0))==0); // Wait for ADC to be ready
    ADC1->CR |= (1 << 2); // Start conversion
    while ( (ADC1->ISR & (1 <<3))==0); // Wait for conversion to finish
    return ADC1->DR; // Return the result
    ADC1->CR = 0;
}

void delay(volatile uint32_t dly) // Tick delay
{
    while (dly--); // Decrement from value using System clock
}

// Function to scale values between 0 and 100
int scaleValue(int value) {
    int minValue = 0;
    int maxValue = 4095;
    int minScale = 1;
    int maxScale = 100;

    // Ensure value is within range
    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;

    // Scale the value
    int scaledValue = ((value - minValue) * (maxScale - minScale) / (maxValue - minValue)) + minScale;

    return scaledValue;
}

void SysTick_Handler(void) // SysTick handler goes through steps
{    
    milliseconds++;
    if(!pauseState)
    {
    tickcntr++; // Increment the counter for each system core clock tick
        if((tickcntr >= 1000)) // If it reaches 1000
        {
            tickcntr=0; // Reset tick counter
            dutyIdx = (dutyIdx+1)%NUM_STEPS; // Step through duty cycle sequence in array
            newVal = stepVal[dutyIdx]; // Take new value from duty cycle array
            setTimer1Duty(newVal); // Update TIM2 to use the new value for PWM
        }
    }
}

