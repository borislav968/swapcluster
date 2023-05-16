
#define F_CPU 16000000UL    // Тактовая частота 16МГц

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>          // для sprintf()

#define UART_BAUD_RATE 57600UL  // Скорость передачи UART 57600 бит/с
#define SET_UBRR ((F_CPU/(16UL*UART_BAUD_RATE))-1UL) // Значения регистров для настройки скорости UART

uint16_t period = UINT16_MAX;   // измеренный период сигнала в 1/62500с
uint8_t time = 0;               // используется в главном цикле

// Настроить UART
void init_uart () {
    UBRR0H = (uint8_t)(SET_UBRR>>8);    // настройка скорости
    UBRR0L = (uint8_t)SET_UBRR;
    UCSR0B = (1<<TXEN0);                // разрешение передачи
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);  // 8 бит
}

// Отправить символ в UART
void uart_putc (uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0))); // ожидание готовности
    UDR0 = data;                    // отправка
}

// Отправить строку в UART
void uart_puts (char *s) {
    while (*s) {        // пока строка не закончилась
        uart_putc(*s);  // отправить очередной символ
        s++;            // передвинуть указатель на следующий символ
    }
}

// Настроить таймер 1 (измерение частоты)
void init_meter () {
    TCCR1B |= (1<<ICES1);   // захват положительного фронта сигнала
    TIMSK1 |= (1<<ICIE1);   // включить прерывание по захвату
    TIMSK1 |= (1<<TOIE1);   // включить прерывание по переполнению
    TCCR1B |= (1<<CS12);    // делитель 256, частота 62500Гц
}

// Настроить таймер 4 (главный цикл)
void init_timer_main () {
    TCCR4B |= (1<<WGM42);   // режим сравнения
    OCR4A = 6250;           // отсчёт до 6250, 1/10с
    TIMSK4 |= (1<<OCIE4A);  // включить прерывание по сравнению
    TCCR4B |= (1<<CS42);    // делитель 256, частота 62500Гц
}

// Прерывание по захвату сигнала таймером 1
ISR (TIMER1_CAPT_vect) {
    TCNT1 = 0;      // обнулить счётный регистр
    period = ICR1;  // результат измерения берётся из регистра захвата
}

// Прерывание по переполнению таймера 1
ISR (TIMER1_OVF_vect) {
    period = UINT16_MAX;
}

// Прерываение по сравнению таймера 4
ISR (TIMER4_COMPA_vect) {
    time++;
}

int main () {

    uint16_t frequency = 0; // частота входного сигнала
    char buffer [30];       // буфер для вывода строк
    
    init_uart();        // настроить UART
    init_meter();       // настроить таймер 1 (измерение частоты)
    init_timer_main();  // настроить таймер 4 (главный цикл)
    
    uart_puts("\x1b[2J\x1b[?25l\n\n");
    uart_puts("Преобразователь частоты тахометра\n\r");
    
    sei();  // разрешить все прерывания
    
    while (1) {
        if (time) {
            frequency = 62500 / period;
            sprintf(buffer, "Частота %uГц  \r", frequency);
            uart_puts(buffer);
            time = 0;
        }
        asm(" ");
    }
    
}
