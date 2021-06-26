CC=arm-none-eabi-gcc
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0

LDFLAGS= -nostdlib -T stm32_ls.ld -W
source= main.c stm32f4xx_gpio_driver.c stm32_startup.c multiled.c pushbuttonled.c


toggleled:clean toggle.o stm32f4xx_gpio_driver.o stm32_startup.o toggle.elf simtoggle

pushbutton:clean pushbuttonled.o stm32f4xx_gpio_driver.o stm32_startup.o pushbuttonled.elf simpush

multipleled:clean multiled.o stm32f4xx_gpio_driver.o stm32_startup.o multiled.elf simmulti

intblink:clean int_blink.o stm32f4xx_gpio_driver.o stm32_startup.o int_blink.elf simintled

seat:clean seatbelt.o stm32f4xx_gpio_driver.o stm32_startup.o seatbelt.elf simseatbelt
	


simtoggle:
	qemu-system-gnuarmeclipse.exe -M STM32F4-Discovery -mcu STM32F407VG -kernel toggle.elf

simmulti:
	qemu-system-gnuarmeclipse.exe -M STM32F4-Discovery -mcu STM32F407VG -kernel multiled.elf

simpush:
	qemu-system-gnuarmeclipse.exe -M STM32F4-Discovery -mcu STM32F407VG -kernel pushbuttonled.elf

simintled:
	qemu-system-gnuarmeclipse.exe -M STM32F4-Discovery -mcu STM32F407VG -kernel int_blink.elf

simseatbelt:
	qemu-system-gnuarmeclipse.exe -M STM32F4-Discovery -mcu STM32F407VG -kernel seatbelt.elf		



toggle.o:toggle.c
	$(CC) $(CFLAGS) -o $@ $^

pushbuttonled.o:pushbuttonled.c
	$(CC) $(CFLAGS) -o $@ $^

multiled.o:multiled.c
	$(CC) $(CFLAGS) -o $@ $^

stm32f4xx_gpio_driver.o:stm32f4xx_gpio_driver.c
	$(CC) $(CFLAGS) -o $@ $^

stm32_startup.o:stm32_startup.c
	$(CC) $(CFLAGS) -o $@ $^

int_blink.o:int_blink.c
	$(CC) $(CFLAGS) -o $@ $^

seatbelt.o:seatbelt.c
	$(CC) $(CFLAGS) -o $@ $^


multiled.elf:multiled.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

pushbuttonled.elf:pushbuttonled.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

toggle.elf:toggle.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

int_blink.elf:int_blink.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

seatbelt.elf:seatbelt.o stm32f4xx_gpio_driver.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^



clean:
	rm -rf *.o *.elf *.map 



analysis:
	cppcheck --enable=all --inconclusive $(source)



init:
	git init
	git remote add origin https://github.com/99004475sourav/STM32_First_Project.git
	
commit:
	git status
	git add .
	git status
	git commit -m 'Commit'

push:
	git branch -M main
	git push -f origin main

pull:
	git pull origin main
