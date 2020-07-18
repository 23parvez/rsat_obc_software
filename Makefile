PREFIX0=sparc-gaisler-elf
PREFIX=$(PREFIX0)-
CC = $(PREFIX)gcc
OBJDUMP = $(PREFIX)objdump
MKPROM = mkprom2
MKPROMOPT ?=
MKPROMFLAGS = -v -ccprefix $(PREFIX0) $(MKPROMOPT) -freq 25 -memcfg1 0x10f80a77 -memcfg2 0x00000E6A -memcfg3 0xC8000300 -rmw -ramsize 2048

CCOPT = -Os -qnano
BSP = leon2
CFLAGS = -qbsp=$(BSP) -mcpu=leon -mfix-at697f -lm -O2 -g

PROJ= rsat-obc
SOURCES = main.c HAL_Global.c Telemetry.c Telecommand.c inthandler.S Global.c HAL_Payload.c HAL_EPS.c HAL_IMU.c HAL_ADC.c HAL_GPS.c HAL_Heater.c HAL_MTR.c HAL_RW.c HAL_Antenna.c adcs_ADandEst.c adcs_RefComp.c adcs_Constants.c adcs_LinearController.c adcs_ModePreProcs.c adcs_pinit.c adcs_SensorDataProcs.c adcs_VarDeclarations.c adcs_GPS_OD.c adcs_CommonRoutines.c remote_patch.c
all: $(PROJ).elf $(PROJ).dis $(PROJ).prom

$(PROJ).elf: $(SOURCES)
	$(CC) -Wl,-Map=$(PROJ).map $^ -o $@ $(CFLAGS)

$(PROJ).dis: $(PROJ).elf
	$(OBJDUMP) -d $^ > $@

$(PROJ).prom: $(PROJ).elf
	$(MKPROM) $(MKPROMFLAGS) -o $@ $^ -$(BSP)

clean:
	del /f $(PROJ).elf $(PROJ).map $(PROJ).dis $(PROJ).prom xdump.s

