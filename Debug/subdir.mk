################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../xdump.s 

C_SRCS += \
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
../Telecommand.c \
../Telemetry.c \
../adcs_ADandEst.c \
../adcs_CommonRoutines.c \
../adcs_Constants.c \
../adcs_GPS_OD.c \
../adcs_LinearController.c \
../adcs_ModePreProcs.c \
../adcs_RefComp.c \
../adcs_SensorDataProcs.c \
../adcs_VarDeclarations.c \
../adcs_pinit.c \
../main.c 

S_UPPER_SRCS += \
../inthandler.S 

OBJS += \
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
./Telecommand.o \
./Telemetry.o \
./adcs_ADandEst.o \
./adcs_CommonRoutines.o \
./adcs_Constants.o \
./adcs_GPS_OD.o \
./adcs_LinearController.o \
./adcs_ModePreProcs.o \
./adcs_RefComp.o \
./adcs_SensorDataProcs.o \
./adcs_VarDeclarations.o \
./adcs_pinit.o \
./inthandler.o \
./main.o \
./xdump.o 

S_DEPS += \
./xdump.d 

S_UPPER_DEPS += \
./inthandler.d 

C_DEPS += \
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
./Telecommand.d \
./Telemetry.d \
./adcs_ADandEst.d \
./adcs_CommonRoutines.d \
./adcs_Constants.d \
./adcs_GPS_OD.d \
./adcs_LinearController.d \
./adcs_ModePreProcs.d \
./adcs_RefComp.d \
./adcs_SensorDataProcs.d \
./adcs_VarDeclarations.d \
./adcs_pinit.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: SPARC RTEMS C Compiler'
	sparc-rtems-gcc -O0 -g3 -Wall -msoft-float -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<" "../inthandler.S" "../xdump.s"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.S
	@echo 'Building file: $<'
	@echo 'Invoking: SPARC RTEMS C Compiler'
	sparc-rtems-gcc -O0 -g3 -Wall -msoft-float -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<" "../inthandler.S" "../xdump.s"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.s
	@echo 'Building file: $<'
	@echo 'Invoking: SPARC RTEMS C Compiler'
	sparc-rtems-gcc -O0 -g3 -Wall -msoft-float -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<" "../inthandler.S" "../xdump.s"
	@echo 'Finished building: $<'
	@echo ' '


