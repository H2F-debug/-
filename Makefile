allin_demo: main.o gpio.o pwm.o uart.o
	aarch64-linux-gcc -Wall main.o gpio.o pwm.o uart.o -o allin_demo

# 统一用 aarch64-linux-gcc 编译所有 .o 文件
main.o: main.c
	aarch64-linux-gcc -c -Wall main.c -o main.o

gpio.o: gpio.c
	aarch64-linux-gcc -c -Wall gpio.c -o gpio.o  # 关键修正：将 cc 改为 aarch64-linux-gcc

pwm.o: pwm.c
	aarch64-linux-gcc -c -Wall pwm.c -o pwm.o

uart.o: uart.c
	aarch64-linux-gcc -c -Wall uart.c -o uart.o
clean:
	rm -f *.o allin_demo