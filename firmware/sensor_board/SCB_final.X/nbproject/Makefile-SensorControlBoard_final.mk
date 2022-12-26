#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-SensorControlBoard_final.mk)" "nbproject/Makefile-local-SensorControlBoard_final.mk"
include nbproject/Makefile-local-SensorControlBoard_final.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=SensorControlBoard_final
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/readsensor.c ../src/adconversion.c ../src/binarySensor.c ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c ../src/main.c ../src/config/SensorControlBoard_final/initialization.c ../src/config/SensorControlBoard_final/interrupts.c ../src/config/SensorControlBoard_final/exceptions.c ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/readsensor.o ${OBJECTDIR}/_ext/1360937237/adconversion.o ${OBJECTDIR}/_ext/1360937237/binarySensor.o ${OBJECTDIR}/_ext/977999967/plib_uart3.o ${OBJECTDIR}/_ext/234832186/plib_adchs.o ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o ${OBJECTDIR}/_ext/1353942241/plib_can4.o ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/502049811/initialization.o ${OBJECTDIR}/_ext/502049811/interrupts.o ${OBJECTDIR}/_ext/502049811/exceptions.o ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o ${OBJECTDIR}/_ext/1353941903/plib_clk.o ${OBJECTDIR}/_ext/977597024/plib_gpio.o ${OBJECTDIR}/_ext/977543196/plib_evic.o ${OBJECTDIR}/_ext/977981641/plib_tmr1.o ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/readsensor.o.d ${OBJECTDIR}/_ext/1360937237/adconversion.o.d ${OBJECTDIR}/_ext/1360937237/binarySensor.o.d ${OBJECTDIR}/_ext/977999967/plib_uart3.o.d ${OBJECTDIR}/_ext/234832186/plib_adchs.o.d ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d ${OBJECTDIR}/_ext/1353942241/plib_can4.o.d ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/502049811/initialization.o.d ${OBJECTDIR}/_ext/502049811/interrupts.o.d ${OBJECTDIR}/_ext/502049811/exceptions.o.d ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d ${OBJECTDIR}/_ext/1353941903/plib_clk.o.d ${OBJECTDIR}/_ext/977597024/plib_gpio.o.d ${OBJECTDIR}/_ext/977543196/plib_evic.o.d ${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/readsensor.o ${OBJECTDIR}/_ext/1360937237/adconversion.o ${OBJECTDIR}/_ext/1360937237/binarySensor.o ${OBJECTDIR}/_ext/977999967/plib_uart3.o ${OBJECTDIR}/_ext/234832186/plib_adchs.o ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o ${OBJECTDIR}/_ext/1353942241/plib_can4.o ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/502049811/initialization.o ${OBJECTDIR}/_ext/502049811/interrupts.o ${OBJECTDIR}/_ext/502049811/exceptions.o ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o ${OBJECTDIR}/_ext/1353941903/plib_clk.o ${OBJECTDIR}/_ext/977597024/plib_gpio.o ${OBJECTDIR}/_ext/977543196/plib_evic.o ${OBJECTDIR}/_ext/977981641/plib_tmr1.o ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o

# Source Files
SOURCEFILES=../src/readsensor.c ../src/adconversion.c ../src/binarySensor.c ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c ../src/main.c ../src/config/SensorControlBoard_final/initialization.c ../src/config/SensorControlBoard_final/interrupts.c ../src/config/SensorControlBoard_final/exceptions.c ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-SensorControlBoard_final.mk dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MK1024MCF064
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/readsensor.o: ../src/readsensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/readsensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/readsensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/readsensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/readsensor.o.d" -o ${OBJECTDIR}/_ext/1360937237/readsensor.o ../src/readsensor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/adconversion.o: ../src/adconversion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/adconversion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/adconversion.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/adconversion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/adconversion.o.d" -o ${OBJECTDIR}/_ext/1360937237/adconversion.o ../src/adconversion.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/binarySensor.o: ../src/binarySensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/binarySensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/binarySensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/binarySensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/binarySensor.o.d" -o ${OBJECTDIR}/_ext/1360937237/binarySensor.o ../src/binarySensor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977999967/plib_uart3.o: ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977999967" 
	@${RM} ${OBJECTDIR}/_ext/977999967/plib_uart3.o.d 
	@${RM} ${OBJECTDIR}/_ext/977999967/plib_uart3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977999967/plib_uart3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977999967/plib_uart3.o.d" -o ${OBJECTDIR}/_ext/977999967/plib_uart3.o ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/234832186/plib_adchs.o: ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/234832186" 
	@${RM} ${OBJECTDIR}/_ext/234832186/plib_adchs.o.d 
	@${RM} ${OBJECTDIR}/_ext/234832186/plib_adchs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/234832186/plib_adchs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/234832186/plib_adchs.o.d" -o ${OBJECTDIR}/_ext/234832186/plib_adchs.o ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr7.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353942241/plib_can4.o: ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353942241" 
	@${RM} ${OBJECTDIR}/_ext/1353942241/plib_can4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353942241/plib_can4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353942241/plib_can4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353942241/plib_can4.o.d" -o ${OBJECTDIR}/_ext/1353942241/plib_can4.o ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr5.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr2.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/initialization.o: ../src/config/SensorControlBoard_final/initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/initialization.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/initialization.o.d" -o ${OBJECTDIR}/_ext/502049811/initialization.o ../src/config/SensorControlBoard_final/initialization.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/interrupts.o: ../src/config/SensorControlBoard_final/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/interrupts.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/interrupts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/interrupts.o.d" -o ${OBJECTDIR}/_ext/502049811/interrupts.o ../src/config/SensorControlBoard_final/interrupts.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/exceptions.o: ../src/config/SensorControlBoard_final/exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/exceptions.o.d" -o ${OBJECTDIR}/_ext/502049811/exceptions.o ../src/config/SensorControlBoard_final/exceptions.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1125350937/xc32_monitor.o: ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1125350937" 
	@${RM} ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353941903/plib_clk.o: ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353941903" 
	@${RM} ${OBJECTDIR}/_ext/1353941903/plib_clk.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353941903/plib_clk.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353941903/plib_clk.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353941903/plib_clk.o.d" -o ${OBJECTDIR}/_ext/1353941903/plib_clk.o ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977597024/plib_gpio.o: ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977597024" 
	@${RM} ${OBJECTDIR}/_ext/977597024/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/977597024/plib_gpio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977597024/plib_gpio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977597024/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/977597024/plib_gpio.o ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977543196/plib_evic.o: ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977543196" 
	@${RM} ${OBJECTDIR}/_ext/977543196/plib_evic.o.d 
	@${RM} ${OBJECTDIR}/_ext/977543196/plib_evic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977543196/plib_evic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977543196/plib_evic.o.d" -o ${OBJECTDIR}/_ext/977543196/plib_evic.o ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977981641/plib_tmr1.o: ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977981641" 
	@${RM} ${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d 
	@${RM} ${OBJECTDIR}/_ext/977981641/plib_tmr1.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d" -o ${OBJECTDIR}/_ext/977981641/plib_tmr1.o ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr9.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
else
${OBJECTDIR}/_ext/1360937237/readsensor.o: ../src/readsensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/readsensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/readsensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/readsensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/readsensor.o.d" -o ${OBJECTDIR}/_ext/1360937237/readsensor.o ../src/readsensor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/adconversion.o: ../src/adconversion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/adconversion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/adconversion.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/adconversion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/adconversion.o.d" -o ${OBJECTDIR}/_ext/1360937237/adconversion.o ../src/adconversion.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/binarySensor.o: ../src/binarySensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/binarySensor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/binarySensor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/binarySensor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/binarySensor.o.d" -o ${OBJECTDIR}/_ext/1360937237/binarySensor.o ../src/binarySensor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977999967/plib_uart3.o: ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977999967" 
	@${RM} ${OBJECTDIR}/_ext/977999967/plib_uart3.o.d 
	@${RM} ${OBJECTDIR}/_ext/977999967/plib_uart3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977999967/plib_uart3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977999967/plib_uart3.o.d" -o ${OBJECTDIR}/_ext/977999967/plib_uart3.o ../src/config/SensorControlBoard_final/peripheral/uart/plib_uart3.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/234832186/plib_adchs.o: ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/234832186" 
	@${RM} ${OBJECTDIR}/_ext/234832186/plib_adchs.o.d 
	@${RM} ${OBJECTDIR}/_ext/234832186/plib_adchs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/234832186/plib_adchs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/234832186/plib_adchs.o.d" -o ${OBJECTDIR}/_ext/234832186/plib_adchs.o ../src/config/SensorControlBoard_final/peripheral/adchs/plib_adchs.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr7.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr7.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr7.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr7.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353942241/plib_can4.o: ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353942241" 
	@${RM} ${OBJECTDIR}/_ext/1353942241/plib_can4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353942241/plib_can4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353942241/plib_can4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353942241/plib_can4.o.d" -o ${OBJECTDIR}/_ext/1353942241/plib_can4.o ../src/config/SensorControlBoard_final/peripheral/can/plib_can4.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr5.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr5.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr5.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr5.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr2.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr2.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr2.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr2.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/initialization.o: ../src/config/SensorControlBoard_final/initialization.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/initialization.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/initialization.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/initialization.o.d" -o ${OBJECTDIR}/_ext/502049811/initialization.o ../src/config/SensorControlBoard_final/initialization.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/interrupts.o: ../src/config/SensorControlBoard_final/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/interrupts.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/interrupts.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/interrupts.o.d" -o ${OBJECTDIR}/_ext/502049811/interrupts.o ../src/config/SensorControlBoard_final/interrupts.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/502049811/exceptions.o: ../src/config/SensorControlBoard_final/exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/502049811" 
	@${RM} ${OBJECTDIR}/_ext/502049811/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/502049811/exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/502049811/exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/502049811/exceptions.o.d" -o ${OBJECTDIR}/_ext/502049811/exceptions.o ../src/config/SensorControlBoard_final/exceptions.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1125350937/xc32_monitor.o: ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1125350937" 
	@${RM} ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1125350937/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/1125350937/xc32_monitor.o ../src/config/SensorControlBoard_final/stdio/xc32_monitor.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353941903/plib_clk.o: ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353941903" 
	@${RM} ${OBJECTDIR}/_ext/1353941903/plib_clk.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353941903/plib_clk.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353941903/plib_clk.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353941903/plib_clk.o.d" -o ${OBJECTDIR}/_ext/1353941903/plib_clk.o ../src/config/SensorControlBoard_final/peripheral/clk/plib_clk.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977597024/plib_gpio.o: ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977597024" 
	@${RM} ${OBJECTDIR}/_ext/977597024/plib_gpio.o.d 
	@${RM} ${OBJECTDIR}/_ext/977597024/plib_gpio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977597024/plib_gpio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977597024/plib_gpio.o.d" -o ${OBJECTDIR}/_ext/977597024/plib_gpio.o ../src/config/SensorControlBoard_final/peripheral/gpio/plib_gpio.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977543196/plib_evic.o: ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977543196" 
	@${RM} ${OBJECTDIR}/_ext/977543196/plib_evic.o.d 
	@${RM} ${OBJECTDIR}/_ext/977543196/plib_evic.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977543196/plib_evic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977543196/plib_evic.o.d" -o ${OBJECTDIR}/_ext/977543196/plib_evic.o ../src/config/SensorControlBoard_final/peripheral/evic/plib_evic.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/977981641/plib_tmr1.o: ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/977981641" 
	@${RM} ${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d 
	@${RM} ${OBJECTDIR}/_ext/977981641/plib_tmr1.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/977981641/plib_tmr1.o.d" -o ${OBJECTDIR}/_ext/977981641/plib_tmr1.o ../src/config/SensorControlBoard_final/peripheral/tmr1/plib_tmr1.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
${OBJECTDIR}/_ext/1353925528/plib_tmr9.o: ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353925528" 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/config/SensorControlBoard_final" -I"../src/packs/PIC32MK1024MCF064_DFP" -I"../src/mips" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1353925528/plib_tmr9.o.d" -o ${OBJECTDIR}/_ext/1353925528/plib_tmr9.o ../src/config/SensorControlBoard_final/peripheral/tmr/plib_tmr9.c    -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp=${DFP_DIR}
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x36F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=512,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp=${DFP_DIR}
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_SensorControlBoard_final=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=512,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp=${DFP_DIR}
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/SCB_final.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/SensorControlBoard_final
	${RM} -r dist/SensorControlBoard_final

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
