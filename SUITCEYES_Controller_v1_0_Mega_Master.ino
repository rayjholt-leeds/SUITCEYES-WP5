#include <PID_v1.h> //Include the PID Library

/* 
*SUITCEYES Haptic Display Controller 
*by Dr Raymond Holt, University of Leeds
*
*This code was developed as part of the SUITCEYES project (http://suitceyes.eu), and is owned by the SUITCEYES consortium. 
*It may not be modified or redsitributed without the consortium's express permission.
*
* This code makes use of the Arduino PID Library by Brett Beauregard, which is used under an MIT License as follows:

Arduino PID Library v1.2

Copyright (c) 2017 Brett Beauregard

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to 
deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is furnished to do so,subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

The above Licence applies solely to the Arduino PID Library v1.2.1.

*OVERVIEW
*This sketch controls an Arudino Mega to drive a tactile display as part of the HIPI (Haptic Intelligent Personalised Interface)from the SUITCEYES project.
* It can provide up to 12 channels with PWM and bidirectional control; if only an on/off signal is required, and no direction, then the direction pins can be repurposed as a further 18 channels.
* The first channel (Channel 0) can also provide Closed Loop control.
* It uses a serial connection to receive data from the controller, which will send it a signal comprising a series of frames.
* Please note that in this version, the 64-byte limit of the Arduino serial mean that 30 channels can only be used with a Packet_Size of 1. 
* The default setup uses a multiplier of 100, so that intensities can be specified as 0 or 1 to give 0% or 100%, and durations specified in multiples of 100ms (e.g. 1 = 100ms, 5 = 500ms, 9 = 900ms)
* 
*/

//These are Variables are hard coded, and can only be set when running up to 
bool Master = true;//records whether this is the master arduino that triggers the others.
byte PWMPins[] = {2,3,4,5,6,7,8,9,10,11,12,13}; //An array to store output pins for channels - you can change the number of channels by adding to these, just make sure that you select PWM-enabled pins!
byte TriggerPin = 11; //Define pin to trigger data logging. Make sure this is not one of the pins defined in PWMPins, so that the trigger doesn't inadvertantly operate an actuator.
byte dirPins[] = {14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}; //An array to store direction pins for channels. Make sure that these are NOT among the pins defined in PWMPins. This must be the same length as PWMPins.
byte SensorPins[] = {A0,A1,A2,A3,A4,A5};//Array to define feedback pins.
const byte Packet_Size = 1; //Sets the number of digits available for each intensity and duration value. This can only be set on the Arduino, to avoid conflicts with the controller.
const byte Max_Signal_Length = 18; //This tells you the most frames that the Arduino can hold in memory.
byte multiplier = 100; //This detrermines the multiples of ten used for signals. It allows more 
bool DetailedTiming = true;
int bitrate = 27600; //Use this to set the bitrate for serial communication. 
/*
 * NOTE: DetailedTiming is used for returning the behaviours of the output pins at the start of each frame.
 * The corresponding arrays take up large amounts of memory, whcih restricts the maximum number of channels and frames you can have.
 * This is intended for diagnostic purposes, and takes some effort to set up. 
 * In order to use it, "Detailed Timing" must be set to True and all instances of DirectionArray and OutputArray must be uncommented.
 *
 */


//The variables below can be set during the Handshake
int Signal_Length = Max_Signal_Length; //This tells you how many frames are available for each signal. All signals must have the same number of frames. Defaults to maximum available
bool Verbose = true; //A variable to set whether the Arduino should return feedback to the controller.
bool feedback = true;//Sets whether feedback should be gathered. This automatically enables the cutoff limits set by LowerCutOff and UpperCutOff. Must be enabled for ClosedLoop to work. 
bool timing = true;//set whether timing data should be reported.
bool ClosedLoop = false;//Sets whether the controller should run in closed loop mode, using PID control. This is set for each channel.
bool Variable_Offset = false;//If true, this takes the initial reading on ADC0 as the ADC_Offset, and the value to return to.
bool Temperature_Reset = false;//If true, the system will try to get the signal to ADC_Offset between tests.
bool bidirectional = true;//If true, the system will use direction pins to set direction of the display. Defaults to on, unless more channels are requested than are available in PWM Pins.
bool CutOffActive = false;//
int LowerCutOff = 0;// Sets the ADC reading below which power will cut out and a warning light will be activated. Set to zero if no cutoff required. Not currently used.
int UpperCutOff = 1023;// Sets the ADC reading above which power will cut out and a warning light will be activated. Set to 1023 if no cutoff required. Not currently used.
int ADC_Offset = 550; //Set ADC reading equivalent to zero.
int Temp_threshold = 10; //Set how near to ADC_offset a reading needs to be before Reset_Temperature() is considered complete.
double Kp = 16;
double Ki = 0;
double Kd = 0;
const int PWMCount = (sizeof(PWMPins)/sizeof(byte)); //Identify number of PWMPins.
const int SensorCount = (sizeof(SensorPins)/sizeof(byte)); //Identify number of pins from SensorPins Array
const int dirPinCount = (sizeof(dirPins)/sizeof(byte));//Identify number of direction pins.
const int PinCount = PWMCount + dirPinCount; //The maximum number of channels is equal to the sum of PWM and direction pins - use this to reserve memory for the intensity signal.

int LEDPin =13;// defines the pin to which the LED is attached - 13 is the built-in LED on an Arduino Nano.
int8_t Intensity_Signal[PinCount][Max_Signal_Length]={};// 2D array of intensities for each channel for each frame - passed by controller.
int Duration_Signal[Max_Signal_Length]={}; //1D array of durations for each frame in ms - passed by controller from Haptic Library CSV. Note that all channels must have the same duration.
byte Current_Frame; //Variable to store current frame index
char Separator[] = ","; //variable to set separator character for feedback from Arduino.
String SignalEnd = "C,,,";
int Packet_Interval = ((PinCount+1)*(Packet_Size+1)); //Calcultes number of bytes required by each frame, based on Packet_Size.
long int TimeArray[Max_Signal_Length+1] = {}; //Array to store timing data from each signal.
//int OutputArray[][Max_Signal_Length+1] = {}; //Array to store displayed data from each signal. Must be uncommented for DetailedTiming.
//int DirectionArray[][Max_Signal_Length+1] = {};//Array to store displayed direction from each signal, or in Boolean mode, to store values displayed on non-PWM channels. Must be uncommented for DetailedTiming.
long int SignalStart; //Variable to store the start of signals for timing purposes.
int LastDirection[PinCount]; //Array to store the previous direction from each frame.
double Setpoint, Input, Output; //Initialise variables required by PID controller.
bool CutOff = false; //variable to store when cutoff has been activated.
int channels = PWMCount;//stores the number of Channels - defaults to number of PWM pins available.
int sensors = SensorCount;//Stores the number of Sensors.
PID PID1(&Input, &Output, &Setpoint,Kp, Ki, Kd, DIRECT);//Initialise PID. 

/*
 * FUNCTIONS 
 */

void ResetSignal(){//Fills the Intensity and Duration Signal Arrays with zeros, ready to receive next signal 
  Serial.println("Resetting the old signal...");
  for (int Frame = 0; Frame<Max_Signal_Length; Frame++){
    Duration_Signal[Frame] = 0;
    for (int Pin = 0; Pin<PinCount; Pin++){
      Intensity_Signal[Pin][Frame] = 0;
    }
  }
}

int GetData(){
  while (Serial.available()<Packet_Size){//Do nothing while there is less serial data available than the Packet Size.
}
String Value = Serial.readStringUntil(',');
int number = Value.toInt();
if (Verbose){
Serial.print("Got: ");
Serial.println(number);
}
Serial.println(SignalEnd);
return number;
}

int GetPin(int PinSpecified){//Identify which Pin to use, based on Pin Specified, and the PWMPin and dirPin Arrays.
   int PinIdentified;
   if (PinSpecified < PWMCount){//If PinSpecified is less than the number of pins identified in the PWMPins array, look up the relevant value in PWMPins.
    PinIdentified = PWMPins[PinSpecified];
   }
   else {//If PinSpecified is larger than the number of pins identified, then we need to look the value up in the dirPins Array.
    PinIdentified = dirPins[(PinSpecified - PWMCount)];
   }
    return PinIdentified;
}

void Handshake(){
//Handshake with controller - send Packet Size required for serial communication, then read data in packets. 
Serial.print("X:");
Serial.println(multiplier);
Serial.print("P:");
Serial.println(Packet_Size);
//controller now knows Packet_Size, so data should be sent in packet size chunks.
int VerboseWanted = GetData();
if (VerboseWanted == 1){Verbose = true;}
    else {Verbose = false;}
channels = GetData();//Get number of channels from Controller.
sensors = GetData();//Get number of sensors from Controller.
Packet_Interval = ((channels+1)*(Packet_Size+1));
int FeedbackWanted = GetData();
if (FeedbackWanted == 1){feedback = true;}
    else {feedback = false;}
int TimingWanted = GetData();
if (TimingWanted == 1){timing = true;}
    else {timing = false;}
int ClosedLoopWanted = GetData();
if (ClosedLoopWanted == 1){ClosedLoop = true;}
    else {ClosedLoop = false;}
Kp = GetData();
Ki = GetData();
Kd = GetData();
PID1.SetTunings(Kp, Ki, Kd); 
ADC_Offset = GetData();
int VariableOffsetWanted = GetData();
if(VariableOffsetWanted == 1){Variable_Offset = true;}
    else{Variable_Offset = false;}
int CutOffWanted = GetData();
if(CutOffWanted == 1){CutOffActive = true;}
    else{CutOffActive = false;}
int LowerCutOff = GetData();// Sets the ADC reading below which power will cut out and a warning light will be activated. Set to zero if no cutoff required.
int UpperCutOff = GetData();// Sets the ADC reading above which power will cut out and a warning light will be activated. Set to 1023 if no cutoff required. 
int Temp_threshold = GetData(); //Set how near to ADC_offset a reading needs to be before Reset_Temperature() is considered complete.
int TempResetWanted = GetData();
if (TempResetWanted ==1){Temperature_Reset = true;}
   else{Temperature_Reset = false;}  
if (ClosedLoop){
        feedback = true;//forces feedback to be on if Closed Loop control is being used on at least one pin, even if it was declared "off" above.
        PID1.SetOutputLimits(-255,255);//Scale the output so it goes from -255 to 255.
        PID1.SetMode(AUTOMATIC);}//Start the PID controller, as closed loop feedback is being used.
if (channels > PinCount){
  Serial.print("E1");
  Serial.println(Separator);
  channels = PinCount;
}
Serial.print("Ch");
Serial.print(Separator);
Serial.println(channels); //tell Central Unit how many channels are available.  
if (channels>PWMCount){//If more channels are requested than are available in PWMPins, switch Bidirectional mode off, so that direction pins can be used for intensity signals.
  bidirectional = false;
  Serial.print("E2");
  Serial.println(Separator);
}
Serial.print("OP");
Serial.print(Separator);
for (int Pin = 0; Pin < channels; Pin++) {
    int PinToUse = GetPin(Pin);
    Serial.print(PinToUse); //Iterate through each declared pin, and print its number.
    Serial.print(Separator);
    }
Serial.println();
if (bidirectional){
    Serial.print("DP");
    Serial.print(Separator);
    for (int Pin = 0; Pin < channels; Pin++) {
        Serial.print(dirPins[Pin]); //Iterate through each declared pin, and print its number.
        Serial.print(", ");
        }
    Serial.println();

    }
Serial.print("Se");
Serial.print(Separator);
Serial.println(sensors);
Serial.print("SP");
Serial.print(Separator);
for (int Sensor = 0; Sensor< sensors; Sensor++) {
    Serial.print(SensorPins[Sensor]); //Iterate through each declared pin, and print its number.
    Serial.print(Separator);;
    }
Serial.println();
Serial.print("PI");
Serial.println(Packet_Interval);
Serial.print("MaxSL");
Serial.println(Max_Signal_Length);
if (Verbose){Serial.print("V");}
    else {Serial.print("NV");}
Serial.println(Separator);
if (Master){Serial.print("M");}
    else {Serial.print("S");}
Serial.println(Separator);
if (feedback){Serial.print("FB");}
    else {Serial.print("NFB");}
Serial.println(Separator);
if (timing){Serial.print("Ti");}
    else {Serial.print("NT");}
Serial.println(Separator);
if (ClosedLoop){Serial.print("CL");}
Serial.println(Separator);
if (Variable_Offset){
   Serial.print("GADC");
   Serial.println(Separator);
   int CurrentValue = analogRead(0);
   for (int i=0; i<9; i++){ //Take 10 readings at 100ms intervals, and average to get ADC Offset
       CurrentValue = CurrentValue+analogRead(0);
   }
   float AverageValue = CurrentValue/10;
   ADC_Offset = int(AverageValue);
   Serial.print("ADCO");
   Serial.print(Separator);
   Serial.println(ADC_Offset);
}

if (ClosedLoop){//If closed loop control is active on Channel 1.
if(Temperature_Reset){
Reset_Temperature();}
}

Serial.println(SignalEnd);//Sent signal to end handshake.

}

int GetDirectChannel(){
  int ChannelToUse = GetData();
  SignalStart = millis();//Start Timer.
  return ChannelToUse;
}

//OK - I think that we need to reshuffle the whole way we do this.

void GetSignal(int ChannelToUse){
    bool Direct = true;
    bool Multichannel = false;
    bool DisplayNothing = false;
    if (ChannelToUse <=0){Direct = false;}//If the specified Channel is 0 or more, this is a direct signal.
    if (ChannelToUse == -2){//If the specified Channel is -2, this is a multichannel signal.
       Multichannel = true; 
       //Serial.println("Multi");//If the specified Channel is zero, get the signal in the same way as direct, but get two
    }
    if (ChannelToUse == -3) {//If the specified channel is -3, the signal should be displayed on another Arduino, so this should display zero throughout the signal.
      Multichannel = true; 
      DisplayNothing =true; 
      //Serial.println("Multiblank");
    }
    if (ChannelToUse == -4) {//If the specified channel is -3, the signal should be displayed on another Arduino, so this should display zero throughout the signal.
      DisplayNothing =true; 
      //Serial.println("DirectBlank");
      Direct = true;
    }
    Signal_Length = GetData();//Get Signal length from Controller.
    int ChannelsToUse = channels;
    if (Direct){ChannelsToUse = 1; Serial.println("Setting Channels to Use to 1");}//In direct mode, set channels to use to 1 - each frame has only an intensity, plus the duration.
    if (Multichannel){ChannelsToUse = 1;}//In multichannel direct mode, set channels to use to 2 - each frame has a channel and an intensity and a
    Packet_Interval = ((ChannelsToUse+1)*(Packet_Size+1));
    if (Multichannel){((ChannelsToUse+2)*(Packet_Size+1));}
    //if (Verbose){
    //  Serial.print("Channel to Use:");
    //  Serial.println(ChannelToUse);
    //  Serial.println(Direct);
    //  Serial.print("Channels To Use:");
    //  Serial.println(ChannelsToUse);}
    while (Current_Frame < Signal_Length){//keep repeating this until all the frames have been read. 
      if (Serial.available()<Packet_Interval){//keep coercing temperature while waiting for signal.
        if (ClosedLoop){ 
        if (Temperature_Reset){
        Reset_Temperature();}}
         }

         if (Serial.available()>= Packet_Interval){ //If there are more than "Packet Interval" characters waiting, we know that at least one whole number is in the buffer. 
         String Value;
         for (int Pin = 0; Pin < ChannelsToUse; Pin++) {
         int PinToUse = Pin;
         if (Direct){PinToUse = ChannelToUse;}
         if (Multichannel){//override the channel by reading the next integer 
         Value = Serial.readStringUntil(',');           
         PinToUse = Value.toInt();
         Serial.print("Channel: ");
         Serial.println(PinToUse);
         }
         Value = Serial.readStringUntil(','); // Read the next string until the next separator.
         if (DisplayNothing){
         Intensity_Signal[PinToUse][Current_Frame] = 0; 
         }
         else{
         Intensity_Signal[PinToUse][Current_Frame] = (Value.toInt())*multiplier; // Get the next integer from the buffer.
         }
         Serial.print("I");
         Serial.println(Intensity_Signal[PinToUse][Current_Frame]);
         if (Verbose){
          Serial.print("Sending");
          Serial.print(Pin+1);
          Serial.print("/");
          Serial.println(ChannelsToUse);
          }
         }
         Value = Serial.readStringUntil(',');// Get next integer from buffer
         Duration_Signal[Current_Frame] = (Value.toInt())*(multiplier);
         Serial.print("DS");
         Serial.println(Duration_Signal[Current_Frame]) ;

         if (Verbose) {        
            Serial.print("CF");
            Serial.print(Current_Frame);         
            Serial.print(", read:");
            for (int Pin = 0; Pin < ChannelsToUse; Pin++) {
               int PinToUse = Pin;
               if(Direct){PinToUse = ChannelToUse;}
               Serial.print(Intensity_Signal[PinToUse][Current_Frame]);
               Serial.print(Separator);
            }
            Serial.println(Duration_Signal[Current_Frame]);
         }
         Current_Frame++;
         
         if (ClosedLoop){ 
         if (Temperature_Reset){
         Reset_Temperature();}}//   coerce temperature towards startpoint
         
         Serial.println(F("C,,,"));
         }
                }  
}

void SendFeedback(){
  long int Timer = millis() - SignalStart;
          Serial.print("F"); //use an F to indicate that the line is feedback.
          Serial.print(Separator);
          Serial.print(Timer);
          for (int Sensor = 0; Sensor < sensors; Sensor++){//get sensor data.
              int currentReading = analogRead(SensorPins[Sensor]);
              if (Sensor == 0){
                  if (currentReading < LowerCutOff){ActivateCutOff(Sensor);}
                  if (currentReading > UpperCutOff){ActivateCutOff(Sensor);}
              }
              Serial.print(Separator);
              Serial.print(currentReading);           
              }
          Serial.println();

}

void PrintIntensityArray() {
  Serial.print("Printing IS");
   for (int i=0; i<Max_Signal_Length; i++){
        for (int j=0; j<PinCount; j++){
       Serial.print(Intensity_Signal[j][i]);
       Serial.print(Separator);}
       Serial.println(" ");
}
}

void PrintDurationArray() {
  Serial.print("Printing DS");
   for (int i=0; i<Max_Signal_Length; i++){      
       Serial.print(Duration_Signal[i]);
       Serial.print(Separator);}
       Serial.println(" ");
}

void DisplaySignal(int ChannelToUse) {
  if (Master){digitalWrite(TriggerPin, HIGH);} //If in Master mode, send signal.
  else {
    while(digitalRead(TriggerPin)==LOW){SendFeedback();};//If in Slave mode, wait until the trigger signal is received.
  }
  for (int Frame = 0; Frame<Signal_Length; Frame++){//iterate through each frame in turn, presenting data.
         if (Verbose){
         Serial.print("start frame");
         Serial.println(Frame);
         }
         int ChannelsToUse = channels;
         for (int Pin = 0; Pin < ChannelsToUse; Pin++) {
            int PinSelected = Pin;
            int PinToUse = GetPin(PinSelected);
            int DisplayBytes; //A variable to store the bytes for display
            int DisplayDirection;//Variable to store Direction.
            /*
            if (DetailedTiming){//Please note - OutputArray and DirectionArray consume large amounts of memory, so are best left commented out.
               if (Pin < PWMCount){          
            OutputArray[Pin][Frame] = Intensity_Signal[Pin][Frame]; //Before doing anything, store the requested intensity for timing data.
            }
            else {
              DirectionArray[Pin-PWMCount][Frame] = Intensity_Signal[Pin][Frame];//If this channel is a dirPin, use Direction Array to store the data.
            }
            }
            */
            if (ClosedLoop && PinSelected==0){//if ClosedLoop control is active and this is the pin for Channel 1, then we need to do something a bit different. 
               double TargetGiven = ((Intensity_Signal[Pin][Frame])/2)+ADC_Offset; //calculate the target setpoint  as a double: doing it as an integer seems to confuse the PID library.   
               Setpoint = TargetGiven;//adjust setpoint. It *should* now stay at this level until the next frame, so there is no need to recalculate TargetGiven.         
               CalculatePID(PinSelected);//Reads the current ADC value, and updates Output based on the PID.       
               DisplayBytes = abs(Output); //It is convenient to record this so that PID Output can be fed back.
               //Serial.println("Time to check direction...");
               DisplayDirection = CalculateDirection(PinSelected, Output); //Set direction based on output, not intensity request.
               /*
               if (DetailedTiming){
               DirectionArray[Pin][Frame] = Setpoint; //If Closed Loop Control is being used, then Direction is meaningless, so SetPoint is returned instead.
               }
               */
         }        
         else {//if closed loop isn't being used on this pin, then just scale the Intensity Signal directly and calcualte .
          DisplayBytes = CalculatePWM(Intensity_Signal[Pin][Frame]); //Calculate PWM value from intensity.
          if (bidirectional){//if the bidirectional mode is available, set the directional signal and store the direction data in DirectionArray.
              DisplayDirection = CalculateDirection(Pin,Intensity_Signal[Pin][Frame]);//Calculate Direction from intensity.
              /*
              if (DetailedTiming){
              DirectionArray[Pin][Frame] = DisplayDirection; //If Open Loop control is used, just direction will be returned on direction pins.
              }
              */
              LastDirection[Pin] = DisplayDirection; //Store LastDirection as an Array
              }                   
          }
         analogWrite(PinToUse,DisplayBytes); //write requested intensity to relevant output pin.
                  }
         long int Timer = millis()-SignalStart;
         TimeArray[Frame] = Timer;
         int FrameStart = millis();
         int Frame_Timer = millis()-FrameStart;
           while (Frame_Timer < Duration_Signal[Frame]){
          Frame_Timer = millis()-FrameStart;//Update Frame Timer
          //Offload feedback to PC.
          if (feedback){
          SendFeedback();
             if (ClosedLoop){//If Closed Loop Control is active, carry out the PID calculation.
                  CalculatePID(0);//determines the new Setpoint, and defines the Bytes as the initial error, so that direction can be determined.       
                  CalculateDirection(0, Output); //Set direction based on output.
                  analogWrite(PWMPins[0],abs(Output));//Set PWM based on Output.
                };

          if (CutOff){break;}//if CutOff has been activated, terminate frame.
          }
          if (CutOff){break;}//if CutOff has been activated, terminate signal.
          }
       }
         for (int Pin = 0; Pin < channels; Pin++) {
         int PinToUse = GetPin(Pin);
         analogWrite(PinToUse,0); //End Signal on each Pin - all signals should now turn off.
         }
         TimeArray[Signal_Length] = millis()-SignalStart;
         /*
         if (DetailedTiming){
              for (int Pin = 0; Pin < PWMCount; Pin++) {
                  OutputArray[Pin][Signal_Length] = 0; //Write values of zero for end of Timing Data Array.
                  DirectionArray[Pin][Signal_Length] = 0;
                  }
         }
         */
       if (Master){digitalWrite(TriggerPin, LOW);}//Turn off Trigger Pin.
       if (Verbose){
       PrintIntensityArray();
       PrintDurationArray();
       }
       Serial.println("C, Signal Ends"); // - Send "C" to notify controller that signal has ended, and next signal can be displayed.
       ResetSignal();
       if (Verbose){
       PrintIntensityArray();
       PrintDurationArray();
       }
}

int CalculatePWM(int BytetoProcess) { 
  int ByteConverted = BytetoProcess*2.55;
  int BytetoReturn = abs(ByteConverted);
  return BytetoReturn;
}

void Reset_Temperature() {//attempts to settle the temperature towards the ADC_Offset value. Only works on first channel at present/
   if (Temperature_Reset){
   Serial.println("Reset CL");
   int Diff; //Variable to store ADC difference.
   Setpoint = ADC_Offset;
   do {//Currently only set to work on the first SensorPin.
    int currentReading(analogRead(SensorPins[0]));
    Diff = abs(currentReading - ADC_Offset);
    CalculatePID(0);
    CalculateDirection(0, Output);//Set Direction.
    if (Verbose){
       Serial.print("Read ");
       Serial.println(currentReading); //Iterate through each declared pin, and print its number.
       Serial.print(", Diff ");
       Serial.println(Diff);
    }
    analogWrite(PWMPins[0],abs(Output));                
    if (currentReading < LowerCutOff){ActivateCutOff(0);}//If the ADC reading is outside the acceptable limits, 
    if (currentReading > UpperCutOff){ActivateCutOff(0);}
    delay(10);
    if (CutOff){break;}
    } while(Diff > Temp_threshold); 
      }
}

void CalculatePID(int PinNumber){ 
           Input = analogRead(PinNumber);
           PID1.Compute();
           if (Verbose){//if in Verbose Mode, clarify output used.
           Serial.print(", SP: ");
           Serial.print(Setpoint);
           Serial.print(", OP:");
           Serial.println(Output);
           }
}

int CalculateDirection(int PinNumber, int BytetoProcess){

   int BytetoReturn;//declare local variable.

   //Establish whether signal is positive, negative or zero. If it's zero, then we can leave it as before, and BytetoReturn equals zero. Otherwise, we need to select an appropriate value to return (1 if positive, -1 if negative).
   if (BytetoProcess == 0){//if the Byte to process is zero, BytetoReturn is zero, and you don't need to do anything else.
      BytetoReturn = 0; 
   }
   else if (BytetoProcess > 0){//If the byte to process is +ve, then set BytetoReturn to one.
      BytetoReturn = 1;  
   }
   else {
        BytetoReturn = -1;//If the byte to process is neither +ve nor zero, it is negative, so set BytetoReturn to minus one.
   }

   if (BytetoReturn > 0){//If Byte to Process is the same as the last one, 
      analogWrite(PWMPins[PinNumber], 0);//Write 0 to the current Pin, while things change
      //if (Verbose){Serial.println("Direction High");}
      delay(1);//1ms delay - to avoid shoot through. Not sure if this is necessary.
      digitalWrite(dirPins[PinNumber], HIGH);
   }
   else if (BytetoReturn < 0)  {
      analogWrite(PWMPins[PinNumber], 0);//Write 0 to the current Pin, while things change.
      //if (Verbose){Serial.println("Direction Low");}
      delay(1);//1ms delay - is this necessary? Ensures that PWM has set to zero before switching H-bridge, to avoid shoot-through.
      digitalWrite(dirPins[PinNumber], LOW);
  }

  return BytetoReturn;
}

void ActivateCutOff(int SensorID){//turns off all channels and exits signal if ADC readings are outside acceptable bounds. 
     if (CutOffActive){
     for (int Pin = 0; Pin < channels; Pin++) {
        analogWrite(PWMPins[Pin], 0); //Iterate through each declared pin, and set its output to zero.
     }
     Serial.print("ADC");
     Serial.print(SensorPins[SensorID]);
     Serial.print(" Outside Cutoff.");
     CutOff = true;//Set CutOff to true.
     digitalWrite(LEDPin, HIGH); //turn on the LED
     }   
}

void SendTimingData(){
        if (timing){ 
        for (int Report = -1; Report<Signal_Length+1; Report++){
        Serial.print("T");
        Serial.print(Separator);
        Serial.print(TimeArray[Report]);
        /*
        if (DetailedTiming){
        for (int Pin = 0; Pin < PWMCount; Pin++) {//Report Data Stored for Each Pin in Turn
        Serial.print(Separator);
        Serial.print(OutputArray[Pin][Report]);
        Serial.print(Separator);
        Serial.print(DirectionArray[Pin][Report]);
        }
        }
        */
        Serial.println(" "); //End the Line       
       }
        }
       Serial.println("C, Signal Ends");
}
/*
 * MAIN PROGRAM
 */

void setup () {

if (Master){pinMode(TriggerPin,OUTPUT);} else {pinMode(TriggerPin,INPUT);}
pinMode(LEDPin, OUTPUT);
for (int Pin = 0; Pin < PinCount; Pin++) {
    pinMode(PWMPins[Pin], OUTPUT); //Iterate through each declared pin, and set it as an Output.
    pinMode(dirPins[Pin], OUTPUT);
    digitalWrite(PWMPins[Pin], LOW);//start with a signal of zero.
    digitalWrite(dirPins[Pin], LOW);
}

//Begin Serial Communication
Serial.begin(bitrate); 
while (Serial.available()>0){
  Serial.read(); //Empty the serial buffer before starting.
}
Handshake();
}

void loop () {
//main loop
    digitalWrite(TriggerPin, LOW); //Turn TriggerPin off so that it doesn't trigger data logging until signal begins. 
    Current_Frame = 0;
    Setpoint = ADC_Offset;//Set setpoint to ADC offset, so that PID if active drives towards desired starting temperature.
    CutOff = false;//reset cutoff signal.   
    int DirectChannel = GetDirectChannel();
    GetSignal(DirectChannel);
    digitalWrite(TriggerPin, HIGH);//Send trigger signal to notify any data logger that recording should start - the trouble is that this disconnects the two timers. They need to begin at the same time.
    //SignalStart = millis(); //Record start time of signal, to co-ordinate timer - comment this line out for Timing Data to be based on signal request time, but be aware that this will put any other Arduinos timers out of Sync.
    DisplaySignal(DirectChannel);
    SendTimingData();   
    if (ClosedLoop){ 
         Reset_Temperature();}//   coerce temperature towards startpoint
    }
