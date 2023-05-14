
#define F_CPU 16000000UL    // Тактовая частота 16МГц

#include <avr/io.h>

#define UART_BAUD_RATE 57600UL  // Скорость передачи UART 57600 бит/с
#define SET_UBRR ((F_CPU/(16UL*UART_BAUD_RATE))-1UL) // Значения регистров для настройки скорости UART


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

int main () {
    
    init_uart();    // настроить UART
    
    uart_puts("\x1b[2J\x1b[?25l\n\n");
    uart_puts("Преобразователь частоты тахометра\n\r");
    
}
