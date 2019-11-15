################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ADandEst.c \
../CommonRoutines.c \
../Constants.c \
../Global.c \
../HAL_ADC.c \
../HAL_Antenna.c \
../HAL_EPS.c \
../HAL_GPS.c \
../HAL_Global.c \
../HAL_Heater.c \
../HAL_IMU.c \
../HAL_MTR.c \
../HAL_Payload.c \
../HAL_RW.c \
../Mode_Preprocessing.c \
../OBC_GPS.c \
../OBC_IMU.c \
../OBC_Ref_Comp.c \
../Orbit_Computation.c \
../SunSensors.c \
../Telecommand.c \
../Telemetry.c \
../main.c 

O_SRCS += \
../28012019.o \
../dak.o \
../dak1.o \
../obc.o 

S_UPPER_SRCS += \
../inthandler.S 

OBJS += \
./ADandEst.o \
./CommonRoutines.o \
./Constants.o \
./Global.o \
./HAL_ADC.o \
./HAL_Antenna.o \
./HAL_EPS.o \
./HAL_GPS.o \
./HAL_Global.o \
./HAL_Heater.o \
./HAL_IMU.o \
./HAL_MTR.o \
./HAL_Payload.o \
./HAL_RW.o \
./Mode_Preprocessing.o \
./OBC_GPS.o \
./OBC_IMU.o \
./OBC_Ref_Comp.o \
./Orbit_Computation.o \
./SunSensors.o \
./Telecommand.o \
./Telemetry.o \
./inthandler.o \
./main.o 

C_DEPS += \
./ADandEst.d \
./CommonRoutines.d \
./Constants.d \
./Global.d \
./HAL_ADC.d \
./HAL_Antenna.d \
./HAL_EPS.d \
./HAL_GPS.d \
./HAL_Global.d \
./HAL_Heater.d \
./HAL_IMU.d \
./HAL_MTR.d \
./HAL_Payload.d \
./HAL_RW.d \
./Mode_Preprocessing.d \
./OBC_GPS.d \
./OBC_IMU.d \
./OBC_Ref_Comp.d \
./Orbit_Computation.d \
./SunSensors.d \
./Telecommand.d \
./Telemetry.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


