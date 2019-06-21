##########################################################################################

#

# SUITCEYES Python Module v1.0

# by Dr Raymond Holt, University of Leeds

#

# This code was developed as part of the SUITCEYES project (http://suitceyes.eu), and is owned by the SUITCEYES consortium. 

# It may not be modified or redsitributed without the consortium's express permission.

#

# modified 23rd April 2019

# by Raymond Holt

#

# Version History

# v0.1 - Raymond Holt (UNIVLEEDS) - 23rd April 2019 - Initial version created

# v1.0 - Raymond Holt (UNIVLEEDS) - 3rd May 2019 - First release.

# Overview

# This module contains the functions used to communicate with the SUITCEYES controllers developed in WP5.

#

#############################################################################################

def Handshake(Port, channels, sensors, Verbose = False, Feedback = False, Timing = False, ClosedLoop = False, Kp=1, Ki = 0, Kd = 0, Offset = 510, VariableOffset = False, CutOff = False, LowerCutOff = 0, UpperCutOff = 1023, ADCThreshold = 10, ADCReset = False):
    print("Beginning Handshake")
    print("Obtaining Packet Size from Arduino")
    packet_size_received = False
    while packet_size_received == False:
        if Port.inWaiting() > 0: #if data is waiting. First thing to identify is the Packet Size, since this is needed to send information. 
           status = Port.readline() #read the next line.
           print(status)
           character = status.split(":")
           if character[0] == "X":
               print("Signal values will be multiplied by " + str(character[1]))
           if character[0] == "P": #collect Packet Size from Arduino
               Packet_Size = int(character[1])
               packet_size_received = True
    print("Packet Size Received - Packet Size:" + str(Packet_Size))
    print("Setting Verbose:" + str(Verbose))
    if Verbose:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Setting Channels: " + str(channels))
    TransmitValue(Port, Packet_Size, channels, Verbose)
    print("Setting Sensors: " + str(sensors))
    TransmitValue(Port, Packet_Size, sensors, Verbose)
    print("Setting Feedback: " + str(Feedback))
    if Feedback:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Setting Timing: " + str(Timing))
    if Timing:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print ("Setting Closed Loop:" + str(ClosedLoop))
    if ClosedLoop:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Setting Proportional Gain")
    TransmitValue(Port, Packet_Size, Kp, Verbose)
    print("Setting Integral Gain")
    TransmitValue(Port, Packet_Size, Ki, Verbose)
    print("Setting Derivative Gain")
    TransmitValue(Port, Packet_Size, Kd, Verbose)
    print("Setting default ADC Offset")
    TransmitValue(Port, Packet_Size, Offset, Verbose)
    print("Setting Variable ADC Offset")
    if VariableOffset:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Setting Cutoff")
    if CutOff:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Setting LowerCutOff")
    TransmitValue(Port, Packet_Size, LowerCutOff, Verbose)
    print("Setting UpperCutoff")
    TransmitValue(Port, Packet_Size, UpperCutOff, Verbose)    
    print("Setting ADC Threshold")
    TransmitValue(Port, Packet_Size, ADCThreshold, Verbose)
    print("Setting ADC Reset")
    if ADCReset:
        TransmitValue(Port, Packet_Size, 1, Verbose)
    else:
        TransmitValue(Port, Packet_Size, 0, Verbose)
    print("Confirming settings from Arduino")
    ListenToController(Port, True)#Let Arduino Offload Summary Information.
    print("Handshake Complete")
    Port.flushInput()
    Port.flushOutput() #Clear Serial Buffer
    return Packet_Size

def MultiHandshake(Port, channels, sensors, Verbose = False, Feedback = False, Timing = False, ClosedLoop = False):
    Packet_Size = Handshake(Port[0], channels[0], sensors[0], Verbose, Feedback, Timing, ClosedLoop) #handshake with Arduino - send channels, sensors, etc and receive Packet_Size for future data transmissions. 
    CurrentController = 1
    while CurrentController<len(Port):
        Handshake(Port[CurrentController], channels[CurrentController], sensors[CurrentController], Verbose, Feedback, Timing, ClosedLoop)
        CurrentController+=1
    return Packet_Size

def ListenToController(Port, Verbose = False):
    #Listen for the controller to confirm receipt of a signal (by writing a line beginning "C")
    #Any feedback from the controller (denoted by a line beginning "F") will be stored as an Array of timestamps, and an Array of Measurements.

    Feedback_Array = [] #Create a blank array for returning timestamp and feedback data
    confirmed = False
    while confirmed == False: #Until the frame is confirmed, keep checking the serial buffer.
        if Port.inWaiting() > 0: # if data is waiting.
            status = Port.readline()
            if Verbose:
                print(status)
            character = status.split(",")
            if character[0] == "C":          #If the Arduino sends a line beginning C, the Arduino has completed the task.
                confirmed = True
            if character[0] == "F": #if an 'F' is received, the line is feedback, so append it to the relevant arrays.
                Sensor_Data = [] #Create blank array for storing the current feedback run
                for i in range(1,len(character),1):
                    Sensor_Data.append(int(character[i])) #Create array of feedback for current timestamp and readings
                Feedback_Array.append(Sensor_Data)#append feedback array
            if character[0] == "T": #if an 'T' is received, the line is Timing Data, so append it to the relevant arrays.
                Timing_Data = [] # create blank array for storing current timing
                for i in range(1,len(character),1):
                    Timing_Data.append(int(character[i])) #Create array of feedback for current timestamp and readings
                Feedback_Array.append(Timing_Data)#append feedback array
            if character[0] == "E1":
                print("Channels requested exceeds maximum for this Arduino. Setting channels to maximum available.")
            if character[0] == "E2":
                print("Too many channels for bidirectional display - no direction control available, as direction pins needed for intensity display")
            if character[0] == "E3":
                print("ADC reading on Channel 0 outside cutoff. Terminating signal.") 
            if character[0] == "V":
                print("Arduino set to Verbose Mode")
            if character[0] == "NV":
                print("Arduino set to Silent Mode")
            if character[0] == "M":
                print("This Arduino is Master - it will provide a trigger signal to any Slave Arduinos")
            if character[0] == "S":
                print("This Arduino is a Slave - it will wait for a trigger signal before displaying the signal")
            if character[0] == "FB":
                print("Feedback enabled")
            if character[0] == "Ti":
                print("Timing Enabled")
            if character[0] == "CL":
                print("Closed Loop Control enabled on Channel 0")
            if character[0] == "GADC":
                print("Variable ADC offset enabled - getting offset")
            if character[0] == "ADCO":
                print("ADC Offset identified as: "+str(character[1]));
            if Verbose:
                if character[0] == "RT":
                    print("Resetting Temperature")
                if character[0] == "RS":
                    print("Resetting Signal")

    return Feedback_Array; #return the timestamp and feedback arrays

def TransmitValue(Port, Packet_Size, value, Verbose):
    value_to_send = str(value)#convert value to string
    string_to_send = "" #set the string for communication to be blank                      
    if value_to_send[0] == "-":
        value_to_send = value_to_send[1:] #remove the first character, which is the negative size.
        while len(value_to_send) <(Packet_Size-1): #if there are less than Packet Size - 1 characters in the current signal.
            value_to_send = "0"+value_to_send
        value_to_send = "-" + value_to_send #re-append the negative sign.
    else:
        while len(value_to_send) <Packet_Size: #if there are less than three characters in the current signal.
            value_to_send = "0"+value_to_send                                  
    string_to_send = string_to_send + value_to_send +"," #append latest figure
    Port.write(string_to_send)
    if Verbose:
        print(string_to_send)
    ListenToController(Port, Verbose) # wait for Arduino to respond.

def MultiDirectSignal(Port, Packet_Size, SignalToSend, Verbose = False):   
    TransmitValue(Port, Packet_Size, -2, Verbose) #transmits "-2" to the controller to tell it that this is a multichannel direct signal.
    Sensor, TimingData = SendSignal(Port, 2, Packet_Size, SignalToSend, Verbose)
    return Sensor, TimingData

def DirectSignal(Port, channel, Packet_Size, SignalToSend, Verbose = False):   
    TransmitValue(Port, Packet_Size, channel, Verbose) #transmits channel number to the controller to tell it that this is a direct signal.
    Sensor, TimingData = SendSignal(Port, 1, Packet_Size, SignalToSend, Verbose)
    return Sensor, TimingData

def BlankDirectSignal(Port, channel, Packet_Size, SignalToSend, Verbose = False):   
    TransmitValue(Port, Packet_Size, -4, Verbose) #transmits channel number to the controller to tell it that this is a direct signal.
    Sensor, TimingData = SendSignal(Port, 1, Packet_Size, SignalToSend, Verbose)
    return Sensor, TimingData

def BlankMultiDirectSignal(Port, Packet_Size, SignalToSend, Verbose = False):   
    TransmitValue(Port, Packet_Size, -3, Verbose) #transmits "-2" to the controller to tell it that this is a multichannel direct signal.
    Sensor, TimingData = SendSignal(Port, 2, Packet_Size, SignalToSend, Verbose)
    return Sensor, TimingData

def TransmitSignal(Port, channels, Packet_Size, SignalToSend, Verbose = False):   
    TransmitValue(Port, Packet_Size, -1, Verbose) #transmits "-1" to the controller to tell it that this is a standard signal.
    Sensor, TimingData = SendSignal(Port, channels, Packet_Size, SignalToSend, Verbose)
    return Sensor, TimingData

def SendSignal(Port, channels, Packet_Size, SignalToSend, Verbose = False, Direct = False):
     current_frame = 0 #integer to keep track of length of signal
     interval = channels+1 #Set interval for reading from signal library: each frame has one entry per channel, plus one for duration.
     signal_length = len(SignalToSend)/interval #Calculated number of frames in signal.
     TransmitValue(Port, Packet_Size, signal_length, Verbose)
     while current_frame < signal_length:#for each frame in the signal.
         frame_signal_start = current_frame * interval #calculate the start point of the current frame
         frame_signal_end = frame_signal_start + interval#calculate end of frame - note that python will return up to, but not including, this.
         current_signal = SignalToSend[frame_signal_start:frame_signal_end] #get the next interval of readings
         if Verbose:
             print("Signal Length"+str(signal_length))
             print("Current Frame"+str(current_frame))
             print("Time to send the frame")
         string_to_send = "" #set the string for communication to be a blank.
         for channel in range (0, interval, 1):                         
             value_to_send = str(current_signal[channel])
             if value_to_send[0] == "-":
                 value_to_send = value_to_send[1:] #remove the first character, which is the negative size.
                 while len(value_to_send) <(Packet_Size-1): #if there are less than Packet Size - 1 characters in the current signal.
                     value_to_send = "0"+value_to_send
                 value_to_send = "-" + value_to_send #re-append the negative sign.
             else:
                 while len(value_to_send) <Packet_Size: #if there are less than three characters in the current signal.
                     value_to_send = "0"+value_to_send                                
             string_to_send = string_to_send + value_to_send +"," #append latest figure
         Port.write(string_to_send)
         if Verbose:
             print(SignalToSend)                   
             print(signal_length)
             print("I sent: " + string_to_send)
             print("Frame Transmitted: Waiting for response")

         ListenToController(Port, Verbose) #Wait until Arduino confirms signal.                                              
         current_frame += 1
     Sensor = ListenToController(Port, Verbose)
     TimingData = ListenToController(Port,Verbose)
     return Sensor, TimingData;

def SendDirectSignal(Port, channels, Packet_Size, SignalToSend, Verbose = False):
     current_frame = 0 #integer to keep track of length of signal
     interval = channels+1 #Set interval for reading from signal library: each frame has one entry per channel, plus one for duration.
     signal_length = len(SignalToSend)/interval #Calculated number of frames in signal.
     TransmitValue(Port, Packet_Size, signal_length, Verbose)
     while current_frame < signal_length:#for each frame in the signal.
         frame_signal_start = current_frame * interval #calculate the start point of the current frame
         frame_signal_end = frame_signal_start + interval#calculate end of frame - note that python will return up to, but not including, this.
         current_signal = SignalToSend[frame_signal_start:frame_signal_end] #get the next interval of readings
         if Verbose:
             print("Signal Length"+str(signal_length))
             print("Current Frame"+str(current_frame))
             print("Time to send the frame")
         string_to_send = "" #set the string for communication to be a blank.
         for channel in range (0, interval, 1):                         
             value_to_send = str(current_signal[channel])
             if value_to_send[0] == "-":
                 value_to_send = value_to_send[1:] #remove the first character, which is the negative size.
                 while len(value_to_send) <(Packet_Size-1): #if there are less than Packet Size - 1 characters in the current signal.
                     value_to_send = "0"+value_to_send
                 value_to_send = "-" + value_to_send #re-append the negative sign.
             else:
                 while len(value_to_send) <Packet_Size: #if there are less than three characters in the current signal.
                     value_to_send = "0"+value_to_send                                
             string_to_send = string_to_send + value_to_send +"," #append latest figure
         Port.write(string_to_send)
         if Verbose:
             print(SignalToSend)                   
             print(signal_length)
             print("I sent: " + string_to_send)
             print("Frame Transmitted: Waiting for response")

         ListenToController(Port) #Wait until Arduino confirms signal.                                              
         current_frame += 1
     Sensor = ListenToController(Port, Verbose)
     TimingData = ListenToController(Port,Verbose)
     return Sensor, TimingData;



def WriteFeedback(FeedbackFile, FeedbackArray, Verbose = False):
    #Takes an array of feedback or timing data provided by the Arduino and writes it to the specified File with in CSV format 
    with open(FeedbackFile, "wb") as feedback:
        columnTitleRow = "Time(s),"
        for i in range(1,(len(FeedbackArray[0])),1):
            columnTitleRow = columnTitleRow + "ADC" + str(i) +"," #use this to set column headings for the output CSV
        columnTitleRow = columnTitleRow + "\n"
        feedback.write(columnTitleRow) #write column headings to CSV
        working_row = 0 #set row counter to co-ordinate writing to CSV
        while working_row < len(FeedbackArray):
            current_row = ""
            working_array = FeedbackArray[working_row]
            for i in range(0,len(working_array),1):                               
                current_row = current_row + str(working_array[i]) +"," #Write data from Feedback Array
            current_row = current_row + "\n"
            if Verbose:
                print(current_row)
            feedback.write(current_row) #write to CSV
            working_row += 1 #increment working row.

def WriteDetailedTimingData(FeedbackFile, FeedbackArray, SignalArray, Verbose = False, Graphable = False):
    #Takes an array of timing data provided by the Arduino and writes the first column to a CSV file.
    
    with open(FeedbackFile, "wb") as feedback:
        columnTitleRow = "Time(s),"
        columnTitleRow = columnTitleRow + "\n"
        feedback.write(columnTitleRow) #write column headings to CSV
        working_row = 0 #set row counter to co-ordinate writing to CSV

        while working_row < (len(FeedbackArray)-1):         
            working_array = FeedbackArray[working_row]
            next_array = FeedbackArray[(working_row+1)]
            current_row = str(working_array[0])+"," # start the current row with the timestamp.
            next_row= str(next_array[0])+"," #start the next row with the timestamp.
            for i in range(1,len(working_array),1):                               
                current_row = current_row + str(working_array[i]) +"," #Write data from Feedback Array              
                next_row = next_row + str(working_array[i]) + ","
            current_row = current_row + "\n"
            if Verbose:
                print(current_row)
            next_row = next_row + "\n"
            feedback.write(current_row) #write to CSV
            if Graphable:
                feedback.write(next_row)
            working_row += 1 #increment working row.
            if Verbose:
                print(working_array)
                print(working_row)
        working_array = FeedbackArray[working_row]
        current_row = str(working_array[0])+"," # start the current row with the timestamp.
        for i in range(1,len(working_array),1):                               
            current_row = current_row + str(working_array[i]) +"," #Write data from Feedback Array               
        current_row = current_row + "\n"
        if Verbose:
            print(current_row)
        feedback.write(current_row) #write to CSV

def WriteTimingData(FeedbackFile, FeedbackArray, SignalArray, Verbose = False):
    #Takes an array of timing data provided by the Arduino and writes the first column to a CSV file.
    
    with open(FeedbackFile, "wb") as feedback:
        columnTitleRow = "Time(s),"
        columnTitleRow = columnTitleRow + "\n"
        feedback.write(columnTitleRow) #write column headings to CSV
        working_row = 0 #set row counter to co-ordinate writing to CSV
        while working_row < (len(FeedbackArray)-1):         
            working_array = FeedbackArray[working_row]
            current_row = str(working_array[0])+"," # start the current row with the timestamp.
            for i in range(1,len(working_array),1):                               
                current_row = current_row + str(working_array[i]) +"," #Write data from Feedback Array              
            current_row = current_row + "\n"
            if Verbose:
                print(current_row)
            feedback.write(current_row) #write to CSV
            working_row += 1 #increment working row.
            if Verbose:
                print(working_array)
                print(working_row)
        working_array = FeedbackArray[working_row]
        current_row = str(working_array[0])+"," # start the current row with the timestamp.
        for i in range(1,len(working_array),1):                               
            current_row = current_row + str(working_array[i]) +"," #Write data from Feedback Array               
        current_row = current_row + "\n"
        if Verbose:
            print(current_row)
        feedback.write(current_row) #write to CSV
