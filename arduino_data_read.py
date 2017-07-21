# -*- coding: utf-8 -*-
"""
Author: Praful Vasireddy
Date: 6/29/17
Description: Master file to control motor using an arduino, calculate 
             the motor potentiometer resistance using a voltage ratio, display
             the photodiode voltage and calibrated power, display the lock-in
             voltage and phase, interactively plot the voltage and power from 
             the photodiode as a function of resistance of the potentiometer
             (equivalent to position).
"""
from serial import Serial
import time
import matplotlib.pyplot as plt
import numpy as np
import sys
import csv
import re


R  = '\033[31m' # red
W  = '\033[0m'  # white (normal)

filename = "/home/praful/Dropbox/SURF17/arduino_code/arduino_motor_controller/data_files/motor_data_" + str(int(time.time())) + ".csv"

with open(filename, 'w') as f:
	writer = csv.writer(f, delimiter='\t')
     	writer.writerow(["Potentiometer Resistance (kOhms)", "Photodiode Voltage (V)", "Photodiode Power (mW)", "Lock-In Magnitude (V)", "Lock-In Phase (deg)"])

# Fitting code to convert voltages into power:

fit_degree = 2 # degree of polynomial fit function

# Calibration values for power and voltage
# When adding to this list, the power and voltage values have to line up, e.g.,
# power_vals_library[i] corresponds to voltage_vals_library[i].
voltage_vals_library = np.array([4.37, 4.296, 4.316, 4.306, 3.402, 2.41, 1.857, 1.496, 1.017, 
                               0.855, 0.733, 0.626, 0.557, 0.347, 0.225])

power_vals_library = np.array([13.75, 13.88, 13.94, 13.96, 11.12, 8.05, 6.36, 5.20, 3.76,
                                 3.17, 2.80, 2.49, 2.28, 1.65, 1.30])

fit_vals = np.flip(np.polyfit(voltage_vals_library, power_vals_library, fit_degree), 0)
# The output of np.polyfit is flipped because it is given in inconvenient 
# highest to lowest degree order.

# Standard polynomial function using the parameters given by np.polyfit
def fit_func(voltages):
    fit_func_value = 0
    for i in range(fit_degree + 1):
        fit_func_value += fit_vals[i]*voltages**i
    return fit_func_value

# Serial connections:    

# The arduino serial port number often changes between /dev/ttyACM0 and
# /dev/ttyACM1. The best way to check is to go to Arduino IDE and see which
# port is available to be connected to. If this could be somehow automated,
# that would be great.


# Miscellaneous functions:

# readcr() iteratively reads bytes from the serial interface until it hits a 
# carriage return. It then converts the bytes into a string and then returns
# a float for the number read from the lock-in detector.

arduino = Serial('/dev/ttyACM0', 9600)
time.sleep(2)

lock_in = Serial('/dev/ttyUSB0', 9600) # 9600 baud rate to match lock-in
                                       # setting
time.sleep(2)
lock_in.write("OUTX 0\n")

def closeSerial():
    arduino.close()
    lock_in.close()

def readcr():
    carriage_return_available = False
    string_read_buffer = []
    while (not(carriage_return_available)):
        char_read = lock_in.read()
        if char_read != '\r':
            string_read_buffer.append(char_read)
        else:
            carriage_return_available = True
    string_read = ''.join(string_read_buffer)
    return float(string_read)
    
def skipToPosition(targetResistance): #works
    arduino.write("SKIP")
    arduino.flush()
    time.sleep(2)
    targetGain = round((targetResistance/10.2), 5)
    arduino.write(str(targetGain))
    arduino.flush()
    stopTurning = False
    while(not(stopTurning)):
        currentGain = float(arduino.readline())
        if(currentGain != 777.777):
            currentResistance = round(10.2*currentGain, 3)
            if(abs(currentResistance - targetResistance) < 0.025):
                stopTurning = True
            print "Target Resistance: " + str(targetResistance) + "kOhms"
            print "Current Resistance: " + str(currentResistance) + "kOhms\n"       
        else:
            print "Resistance not found."
            stopTurning = True
    time.sleep(2)
    finalResistance = getResistance()
    print "Final resistance: " + str(finalResistance) + "kOhms"

def setResolution(resolution): #works
    arduino.write("RES")
    arduino.flush()
    time.sleep(2)
    arduino.write(str(resolution))
    arduino.flush()
    resolutionSetting = int(re.sub(r'[^\d-]+', '', arduino.readline()))
    print "Resolution set to: " + str(resolutionSetting) + "ms"
    
def setPD(targetPDVoltage): #works
    arduino.write("SETPD")
    arduino.flush()
    time.sleep(2)
    arduino.write(str(targetPDVoltage))
    arduino.flush()
    stopPD = False
    while(not(stopPD)):
        currentPDVoltage = float(arduino.readline())
        if(currentPDVoltage != 777.777):
            if(abs(currentPDVoltage - targetPDVoltage) < 0.1):
                stopPD = True
            print "Target Voltage: " + str(targetPDVoltage) + "V"
            print "Current Voltage: " + str(currentPDVoltage) + "V\n"
        else:
            print "Voltage not found in sweep."
            stopPD = True
    time.sleep(2)
    finalVoltage = getVoltage()
    print "Final voltage: " + str(finalVoltage) + "V"
            
def getVoltage(): #works
    arduino.write("GETVOLT")
    arduino.flush()
    time.sleep(2)
    currentVoltage = float(arduino.readline())
    return currentVoltage
 
def getResistance(): #works
    arduino.write("GETPOS")
    arduino.flush()
    currentResistance = round(10.2*float(arduino.readline()), 3)
    return currentResistance
    
def sweep(sweepSteps, sweepTimes, startingDirection): #works
    switchTimes = sweepTimes * 2
    if (startingDirection == "CCW"):
        measuredRunCCW(sweepSteps)
        time.sleep(2)
        switchDirection()
        time.sleep(2)
        for i in range(switchTimes - 1):
            measuredRun(sweepSteps)
            time.sleep(2)
            switchDirection()
            time.sleep(2)
    elif(startingDirection == "CW"):
        measuredRunCW(sweepSteps)
        time.sleep(2)
        switchDirection()
        time.sleep(2)
        for i in range(switchTimes - 1):
            measuredRun(sweepSteps)
            time.sleep(2)
            switchDirection()
            time.sleep(2)
        
def switchDirection(): #works
    arduino.write("SWITCH")
    arduino.flush()
    
def timeSweep(seconds):
    currentResistance = getResistance()
    time.sleep(2)
    lock_in_magnitude_buffer = []
    lock_in_phase_buffer = []
    pd_voltage_buffer = []
    pd_power_buffer = []
    pot_resistance_buffer = []
    
    plt.ion()
    
    plt.figure(3)
    plt.xlabel("PD Voltage (V)")
    plt.ylabel("Lock-In Magnitude (V)")
    plt.grid(True, which='both', ls='-')
    
    plt.figure(4)
    plt.xlabel("PD Voltage (V)")
    plt.ylabel("Lock-In Phase (deg)")
    plt.grid(True, which='both', ls='-')
    
    arduino.write("TIMESWEEP")
    arduino.flush()
    t_end = time.time() + seconds
    if (seconds < 2):
        time.sleep(2)
    while (time.time() < t_end):
        pot_resistance_buffer.append(currentResistance)
        pd_voltage_value = float(arduino.readline())
        pd_voltage_buffer.append(pd_voltage_value)
        pd_power_value = fit_func(pd_voltage_value)
        pd_power_buffer.append(round(pd_power_value, 3))
        
        print "**********************************"
        print "PD Voltages: "
        print pd_voltage_buffer
        print "\nPD Powers: "
        print pd_power_buffer
        
        lock_in.write("OUTP? 1\n")
        lock_in_x_value = readcr()
        lock_in.write("OUTP? 2\n")
        lock_in_y_value = readcr()
        lock_in_magnitude_value = np.absolute(lock_in_x_value + 1j * lock_in_y_value)
        lock_in_magnitude_buffer.append(lock_in_magnitude_value)    # store and print
        print "\nLock-In Voltages: "
        print lock_in_magnitude_buffer
        
        lock_in.write("OUTP? 4\n")
        lock_in_phase_value = readcr()
        lock_in_phase_buffer.append(lock_in_phase_value)
        print "\nLock-In Phases: "
        print lock_in_phase_buffer
        
        plt.figure(3)
        plt.scatter(pd_voltage_value, lock_in_magnitude_value, c = 'g')
        plt.pause(0.05)
        
        plt.figure(4)
        plt.scatter(pd_voltage_value, lock_in_phase_value, c = 'r')
        plt.pause(0.05)
        
    arduino.write("STOP")
    arduino.flush();
    
    zip(pot_resistance_buffer, pd_voltage_buffer, pd_power_buffer, lock_in_magnitude_buffer, lock_in_phase_buffer)
    with open(filename, 'a') as g:
        awriter = csv.writer(g, delimiter='\t')
        awriter.writerows(zip(pot_resistance_buffer, pd_voltage_buffer, pd_power_buffer, lock_in_magnitude_buffer, lock_in_phase_buffer))
    
    #plt.figure(5)
    #plt.grid(True, which='both', ls='-')
    #plt.xlabel("log PD Voltage")
    #plt.ylabel("log Lock-In Magnitude")
    #plt.xscale('log')
    #plt.yscale('log')
    #plt.plot(pd_voltage_buffer, lock_in_magnitude_buffer, c = 'g')

def getResolution(): #works
    arduino.write("GETRES")
    arduino.flush()
    currentResolution = int(re.sub(r'[^\d-]+', '', arduino.readline()))
    print "Current Resolution: " + str(currentResolution) + "ms"
    
def getDirection(): #works
    arduino.write("GETDIR")
    arduino.flush()
    currentDirection = int(re.sub(r'[^\d-]+', '', arduino.readline()))
    return currentDirection

def measuredRunCCW(numSteps): #works
    if (not(getDirection())):
        time.sleep(2)
        switchDirection()
    time.sleep(2)
    measuredRun(numSteps)
        
def measuredRunCW(numSteps): #works
    if (getDirection()):
        time.sleep(2)
        switchDirection()
    time.sleep(2)
    measuredRun(numSteps)
        
def measuredRun(numSteps): #works
    # Reading the data from the arduino and lock-in detector:

    # Initializing empty buffers for use in the loop which stores data
    pd_voltage_buffer = []
    pd_power_buffer = []

    lock_in_magnitude_buffer = []
    lock_in_phase_buffer = []
    
    pot_resistance_buffer = []

    plt.ion()   # turn on interactive plotting

    # Create two separate figures to observe how voltage and power react to 
    # the turning of the motor
    plt.figure(1)
    plt.grid(True, which='both', ls='-')
    plt.xlabel("CCW Resistance (kOhms)")
    plt.ylabel("PD Voltage (V)")

    plt.figure(2)
    plt.grid(True, which='both', ls='-')
    plt.xlabel("CCW Resistance (kOhms)")
    plt.ylabel("Output Power (mW)")
    
    plt.figure(3)
    plt.xlabel("PD Voltage (V)")
    plt.ylabel("Lock-In Magnitude (V)")
    plt.grid(True, which='both', ls='-')
    
    plt.figure(4)
    plt.xlabel("PD Voltage (V)")
    plt.ylabel("Lock-In Phase (deg)")
    plt.grid(True, which='both', ls='-')

    arduino.write("START")  # interpreted on the arduino side to begin
    arduino.flush()

    # Big loop to store the data read from the serial interfaces, write the 
    # data in a readable format to the console, and interactively plot the data.
    for i in range(numSteps):     # figuring out exactly how long to run this loop or
                            # even making it a user setting is a definite area
                            # this code could be improved

        # Read the voltage and power from the arduino, store into a buffer.
        dataReady = False
        while(not(dataReady)):
            if (arduino.readline() == "MEASURE\r\n"):
                dataReady = True
                
        pd_voltage_value = float(arduino.readline())
        pd_voltage_buffer.append(pd_voltage_value)
        pd_power_value = fit_func(pd_voltage_value)
        pd_power_buffer.append(round(pd_power_value, 3))
    
        # Print out the data
        print "MOTOR TURN #" + str(i+1)
        print "\nPD Voltages: "
        print pd_voltage_buffer
        print "\nPD Powers: "
        print pd_power_buffer
        
        # Convert the voltage ratio sent from the arudino back into a resistance
        # by multiplying by the total resistance, 10.2 kOhms.
        helipot_ccw_resistance = 10.2*float(arduino.readline())
        pot_resistance_buffer.append(round(helipot_ccw_resistance, 3))
        print "\nHelipot CCW (Orange) Side Resistance: " + str(round(helipot_ccw_resistance, 2)) + "k"
        
        # Send over the command to read from the X channel on the Lock-In
        # OUTP? is a general command to get the output of a channel, specified by
        # the following parameter:
        # 1 <=> X, 2 <=> Y, 3 <=> R, 4 <=> phi
        lock_in.write("OUTP? 1\n")
        lock_in_x_value = readcr()    # read the voltage as a float
        lock_in.write("OUTP? 2\n")
        lock_in_y_value = readcr()
        lock_in_magnitude_value = np.absolute(lock_in_x_value + 1j * lock_in_y_value)
        lock_in_magnitude_buffer.append(lock_in_magnitude_value)    # store and print
        print "\nLock-In Voltages: "
        print lock_in_magnitude_buffer
        
        lock_in.write("OUTP? 4\n")
        lock_in_phase_value = readcr()
        lock_in_phase_buffer.append(lock_in_phase_value)
        print "\nLock-In Phases: "
        print lock_in_phase_buffer
        
        print R+"\nPercentage of way to counterclockwise end: "
        pot_percentage = int(helipot_ccw_resistance/10.2 * 100)
        
        sys.stdout.write("\n" + str(pot_percentage) + "%")
        sys.stdout.write("  [")
        for j in range(pot_percentage):
            sys.stdout.write("#")
        for k in range(100-pot_percentage):
            sys.stdout.write(" ")
        sys.stdout.write("]\n"+W)
        print

        plt.figure(1)       # choose which figure to plot on
        plt.scatter(helipot_ccw_resistance, pd_voltage_value, c = 'b')
        plt.pause(0.05)     # allow matplotlib enough time to draw the points
    
        plt.figure(2)
        plt.scatter(helipot_ccw_resistance, pd_power_value, c = 'r')
        plt.pause(0.05)
    
        plt.figure(3)
        plt.scatter(pd_voltage_value, lock_in_magnitude_value, c = 'g')
        plt.pause(0.05)
        
        plt.figure(4)
        plt.scatter(pd_voltage_value, lock_in_phase_value, c = 'r')
        plt.pause(0.05)
        
        if (i < numSteps - 1):
            arduino.write("NEXT")
            arduino.flush()
        else:
            arduino.write("STOP")
            arduino.flush()
        
    zip(pot_resistance_buffer, pd_voltage_buffer, pd_power_buffer, lock_in_magnitude_buffer, lock_in_phase_buffer)
    with open(filename, 'a') as g:
        awriter = csv.writer(g, delimiter='\t')
        awriter.writerows(zip(pot_resistance_buffer, pd_voltage_buffer, pd_power_buffer, lock_in_magnitude_buffer, lock_in_phase_buffer))

    #plt.figure(5)
    #plt.grid(True, which='both', ls='-')
    #plt.xlabel("log PD Voltage")
    #plt.ylabel("log Lock-In Magnitude")
    #plt.xscale('log')
    #plt.yscale('log')
    #plt.plot(pd_voltage_buffer, lock_in_magnitude_buffer, c = 'g')
    
print R+"\nMOTOR CONTROL INTERFACE\n\n"
print "Functions available:\n"
print "closeSerial(): close lock-in and arduino serial ports\n"
print "skipToPosition(target resistance in kOhms): set motor position to user setting"
print "setPD(target voltage in V): sets the motor to a target voltage"
print "setResolution(motor on time in ms): set resolution to user setting"
print "switchDirection(): reverse current direction\n"
print "getVoltage(): returns current voltage in V"
print "getResistance(): returns current resistance in kOhms"
print "getResolution(): returns the current resolution in ms"
print "getDirection(): returns 1 for CCW and 0 for CW\n"
print "sweep(# of steps per sweep, # of sweeeps, \"CCW\" or \"CW\"): sweeps across a given range"
print "timeSweep(seconds): collects data without moving the motor for specified time\n"
print "measuredRunCCW(# of steps per run): store and plot data for specified number of steps in CCW direction"
print "measuredRunCCW(# of steps per run): store and plot data for specified number of steps in CW direction"
print "measuredRun(# of steps per run): store and plot data for specified number of steps in current direction\n"
print "REMEMBER TO RUN closeSerial() AT END OF EACH SESSION"+W