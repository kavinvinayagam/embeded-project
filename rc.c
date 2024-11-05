#include "stm32f4xx.h"
void GPIO_Init(void);
void motor1_forward(void);
void motor1_backward(void);
void motor1_stop(void);
void motor2_forward(void);
void motor2_backward(void);
void motor2_stop(void);
void motor3_forward(void);
void motor3_backward(void);
void motor3_stop(void);
void motor4_forward(void);
void motor4_backward(void);
void motor4_stop(void);
void USART1_Init(void);
void USART1_Write(char ch);
char USART1_Read(void);
void run_all_motors_forward(void);
void run_all_motors_backward(void);
void stop_all_motors(void);
void steer_left(void);
void steer_right(void);
void halt(void);
void EXTI0_IRQHandler(void); // Interrupt handler for EXTI line 0
int stop=1;

int main(void) {
    // Enable GPIOC clock for LED control
    RCC->AHB1ENR |= (1U << 2);  // Enable clock for GPIOC
    RCC->AHB1ENR |= (1U << 1);
    RCC->AHB1ENR |= (1U << 0);
    
    // Enable USART1 clock
    RCC->APB2ENR |= (1U << 4);  // Enable clock for USART1
    
    // Configure GPIO and USART
		
    GPIO_Init();
    USART1_Init();

    // Configure EXTI for obstacle detection (assuming PB0 is used for the sensor)
    RCC->APB2ENR |= (1U << 14);  // Enable SYSCFG clock
    SYSCFG->EXTICR[0] |= (1U << 0); // Map EXTI line 0 to PB0

    EXTI->IMR |= (1U << 0);   // Unmask EXTI line 0
    EXTI->RTSR |= (1U << 0);  // Trigger on rising edge (adjust as needed)

    NVIC_EnableIRQ(EXTI0_IRQn); // Enable EXTI0 interrupt in NVIC

    // Update main loop to check 'stop' state
while (1) {
    

    char command = USART1_Read(); // Read a command from Bluetooth
    if (command == 'w') {
        run_all_motors_forward();
        USART1_Write('c'); // Send back character 'c'
    } else if (command == 's') {
        if (stop) { // Only move if stop flag allows
            run_all_motors_backward();
            USART1_Write('b');
        }
    } else if (command == 'a') {
        steer_left();
        USART1_Write('t');
    } else if (command == 'd') {
        steer_right();
        USART1_Write('r');
    } else if (command == 'b') {
        halt();
    } else {
        USART1_Write('n');
    }
}

}

// Interrupt handler for EXTI line 0

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1U << 0)) {
        EXTI->PR |= (1U << 0); // Clear the pending bit

        if (!((GPIOB->IDR &(1U<<0))==(1U<<0))) {
            halt();             // Stop all motors when triggered
            USART1_Write('o');  // Notify obstacle detected
                     // Set stop state to indicate stopped
        } 
    }
}



void GPIO_Init(void) {
    // Enable clock for GPIOA and GPIOB
		GPIOB->MODER &= ~(3U << (0 * 2)); // Set PB0 to input mode (00)

    // Set PA0, PA1 as output for Motor 1
    GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2))); // Clear mode bits for PA0 and PA1
    GPIOA->MODER |= (1 << (0 * 2)) | (1 << (1 * 2)); // Set PA0 and PA1 to output mode

    // Set PA3, PA4 as output for Motor 2
    GPIOA->MODER &= ~((3 << (3 * 2)) | (3 << (4 * 2))); // Clear mode bits for PA3 and PA4
    GPIOA->MODER |= (1 << (3 * 2)) | (1 << (4 * 2)); // Set PA3 and PA4 to output mode

    // Set PB3, PB4 as output for Motor 3
    GPIOB->MODER &= ~((3 << (3 * 2)) | (3 << (4 * 2))); // Clear mode bits for PB3 and PB4
    GPIOB->MODER |= (1 << (3 * 2)) | (1 << (4 * 2)); // Set PB3 and PB4 to output mode

    // Set PB5, PB6 as output for Motor 4
    GPIOB->MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2))); // Clear mode bits for PB5 and PB6
    GPIOB->MODER |= (1 << (5 * 2)) | (1 << (6 * 2)); // Set PB5 and PB6 to output mode
}

int is_obstacle_front(void) {
    return !(GPIOB->IDR & (1U << 0)); // Assuming B0 is used for the front sensor
}

// Motor 1 Control Functions
void motor1_forward() {
    GPIOA->ODR |= (1 << 0);  // Set PA0 high (IN1 for Motor 1)
    GPIOA->ODR &= ~(1 << 1);  // Set PA1 low (IN2 for Motor 1)
}

void motor1_backward() {
    GPIOA->ODR |= (1 << 1);  // Set PA1 high (IN2 for Motor 1)
    GPIOA->ODR &= ~(1 << 0);  // Set PA0 low (IN1 for Motor 1)
}

void motor1_stop() {
    GPIOA->ODR &= ~((1 << 0) | (1 << 1)); // Stop Motor 1
}

// Motor 2 Control Functions
void motor2_forward(){
    GPIOA->ODR |= (1 << 3);  // Set PA3 high (IN3 for Motor 2)
    GPIOA->ODR &= ~(1 << 4);  // Set PA4 low (IN4 for Motor 2)
}

void motor2_backward() {
    GPIOA->ODR |= (1 << 4);  // Set PA4 high (IN4 for Motor 2)
    GPIOA->ODR &= ~(1 << 3);  // Set PA3 low (IN3 for Motor 2)
}

void motor2_stop() {
    GPIOA->ODR &= ~((1 << 3) | (1 << 4)); // Stop Motor 2
}

// Motor 3 Control Functions
void motor3_forward() {
    GPIOB->ODR |= (1 << 3);  // Set PB3 high (IN5 for Motor 3)
    GPIOB->ODR &= ~(1 << 4);  // Set PB4 low (IN6 for Motor 3)
}

void motor3_backward() {
    GPIOB->ODR |= (1 << 4);  // Set PB4 high (IN6 for Motor 3)
    GPIOB->ODR &= ~(1 << 3);  // Set PB3 low (IN5 for Motor 3)
}

void motor3_stop() {
    GPIOB->ODR &= ~((1 << 3) | (1 << 4)); // Stop Motor 3
}

// Motor 4 Control Functions
void motor4_forward() {
    GPIOB->ODR |= (1 << 5);  // Set PB5 high (IN7 for Motor 4)
    GPIOB->ODR &= ~(1 << 6);  // Set PB6 low (IN8 for Motor 4)
}

void motor4_backward() {
    GPIOB->ODR |= (1 << 6);  // Set PB6 high (IN8 for Motor 4)
    GPIOB->ODR &= ~(1 << 5);  // Set PB5 low (IN7 for Motor 4)
}

void motor4_stop() {
    GPIOB->ODR &= ~((1 << 5) | (1 << 6)); // Stop Motor 4
}
void steer_left(){
	motor3_forward();
	motor4_forward();
	motor1_stop();
	motor2_stop();
}
void steer_right(){
	motor1_forward();
	motor2_forward();
	motor3_stop();
	motor4_stop();
}
void halt(){
	motor1_stop();
	motor2_stop();
	motor3_stop();
	motor4_stop();
	
}

// Run all motors forward

void run_all_motors_forward(void) {
    motor1_forward();
    motor2_forward();
    motor3_forward();
    motor4_forward();
}

// Run all motors backward
void run_all_motors_backward(void) {
    motor1_backward();
    motor2_backward();
    motor3_backward();
    motor4_backward();
}

// Stop all motors
void stop_all_motors(void) {
    motor1_stop();
    motor2_stop();
    motor3_stop();
    motor4_stop();
}

// USART1 Initialization
void USART1_Init(void) {
    // Enable GPIOA clock for USART1 pins
    RCC->AHB1ENR |= (1U << 0); // Enable GPIOA clock

    // Configure PA9 (TX) and PA10 (RX) for USART1
    GPIOA->MODER &= ~(3U << (9 * 2) | 3U << (10 * 2));
    GPIOA->MODER |= (2U << (9 * 2)) | (2U << (10 * 2)); // Set PA9 and PA10 to alternate function
    GPIOA->AFR[1] |= (7U << (1 * 4)) | (7U << (2 * 4)); // Set AF7 (USART1) for PA9 and PA10

    // Configure USART1 for 9600 baud rate
	
    USART1->BRR = 0x683; // Assuming 16 MHz clock, BRR for 9600 baud
    USART1->CR1 = (1U << 3) | (1U << 2) | (1U << 13); // Enable USART1, TX, RX
}

void USART1_Write(char ch) {
    while (!(USART1->SR & (1U << 7))); // Wait until TX buffer is empty
    USART1->DR = ch;
}

char USART1_Read(void) {
    while (!(USART1->SR & (1U << 5))); // Wait until RX buffer has data
    return USART1->DR;
}