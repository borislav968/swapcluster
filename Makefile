PROJECT = swapcluster
DEVICE = atmega328pb
PROGRAMMER = -c usbasp -P usb
COMPILE = avr-gcc -Wall -O1 -mmcu=$(DEVICE)
OBJCOPY = avr-objcopy

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -V

SRC = $(wildcard *.c)
HDR = $(wildcard *.h)
OBJ = $(patsubst %.c, %.o, $(wildcard *.c))

.PHONY: all

all: $(PROJECT).bin

$(PROJECT).bin: $(OBJ)
	$(COMPILE) -o $(PROJECT).elf $(OBJ)
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin
	avr-size -C --mcu=$(DEVICE)  $(PROJECT).elf

%.o: %.c $(HDR)
	$(COMPILE) -c $< -o $@

flash:	$(PROJECT).bin
	$(AVRDUDE) -U flash:w:$(PROJECT).bin:r

verify:	$(PROJECT).bin
	$(AVRDUDE) -U flash:v:$(PROJECT).bin:r

erase:
	$(AVRDUDE) -e
	
clean:
	rm -f *.elf *.o *.bin
	
