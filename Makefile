all:
	@gcc -g vehicle.c module_core.c modbus.c log.c demo_03.c demo_02.c uart.c misc.c nmea.c gps_car.c -o gps_car -lm

clean:
	@rm gps_car

