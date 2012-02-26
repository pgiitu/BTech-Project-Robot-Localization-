################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Faces.cpp \
../src/HypothesisGenerator.cpp \
../src/HypothesisGenerator_test.cpp \
../src/Majoritymap.cpp \
../src/MapIOHandler.cpp \
../src/MapIOHandler_test.cpp \
../src/PolygonUtil.cpp \
../src/PolygonUtil_test.cpp \
../src/RobotLocalization.cpp \
../src/UIutil.cpp 

OBJS += \
./src/Faces.o \
./src/HypothesisGenerator.o \
./src/HypothesisGenerator_test.o \
./src/Majoritymap.o \
./src/MapIOHandler.o \
./src/MapIOHandler_test.o \
./src/PolygonUtil.o \
./src/PolygonUtil_test.o \
./src/RobotLocalization.o \
./src/UIutil.o 

CPP_DEPS += \
./src/Faces.d \
./src/HypothesisGenerator.d \
./src/HypothesisGenerator_test.d \
./src/Majoritymap.d \
./src/MapIOHandler.d \
./src/MapIOHandler_test.d \
./src/PolygonUtil.d \
./src/PolygonUtil_test.d \
./src/RobotLocalization.d \
./src/UIutil.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


