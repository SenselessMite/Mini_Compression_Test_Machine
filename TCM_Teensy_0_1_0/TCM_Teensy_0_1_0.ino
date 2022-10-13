/*
 * T-C Machine:
 *    This is a OPEN HARDWARE and OPEN SOFTWARE prototype for a Traction compression machine
 *    This driver is designed to work on a Teensy 3.1/3.2 board
 */

/*
 * Library included:
 *    - parameters.h contains all the constant parameters for the machine
 *    - pinout.h contains the definition of the pin used
 *    - ADC.h it is used for analog-digital conversion of the potentiometer value
 *    - TeensyStep.h is used to drive the stepper motor
 *    - IntervalTimer.h is the librar
 *    - HX711.h is the library for the SparkFun HX711 Load Cell Amplifier
 */
#include "parameters.h"
#include "pinout.h"

#include <ADC.h>
#include <TeensyStep.h>
#include <IntervalTimer.h>
#include <HX711.h>

ADC *adc = new ADC(); //ADC obbject 

IntervalTimer timer0; //Timer Object

HX711 scale;//Load Cell Object

/*
 * The variable mod is used to mantain the operational status of the machine:
 *    - if mod = false the machine is used in serial monitor mode and selSerial is set to Serial
 *    - if mod = true the machine is used in web mode and selSerial is set to Serial1
 */
boolean mod = false;

Stream *selSerial;//serial communication currently selected
/*
 * LMIN and LMAX are used to save the min and max position for the potentiometer
 *    - LMIN is position 0;
 *    - LMAX is the position when the column touch the load cell plate
 *    
 * The two variables are set in the autoCalibration function
 */
float LMIN=0; //minimum position potentiometer
float LMAX=0; //maximum position potentiometer

/*
 * Status variable for the machine:
 *    - ES is the displacement set for the measurement
 *    - SPEED is the speed of the motor expressed in mm/min
 *    - TS is the sampling time for the measure
 *    - NC is the number of measuring cycles 
 */
float ES= DEF_ES;   // displacement [mm]
float SPEED=DEF_SPEED;// spedd [mm/min] 
int TS=DEF_TS;   // sampling time [ms]
int NC=DEF_NC;     // number of cycles

float ES0=ES;  //displacement zero


/*
 * The following variables are used to set the machine 
 * parameters and are automatically obtained from the variables: ES, SPEED, TS, NC
 *    - VSTP indicates the speed of the motor in stp/s
 *    - POS indicates the number of steps for the current displacement
 *    - POSI is an integer approximation  of POS
 *    - TTOT indicates the time required to carry out the measurement process
 *    - NP indicates the number of measurements to be made
 *    - MAX_NC indicates the max number of cycles
 */
//int RPMSP=(int)(SPEED/PAS/MOTOR_STEPS*F1); // [rotation/min] motor speed in rpm 
int VSTP=(int)(SPEED/PAS/60*F1*MCRSTP); // [stp/s] motor speed in step
float POS=ES/PAS*MCRSTP; // total steps for the current displacement
int POSI=(int)POS; // approximation of POS
float TTOT=(2*NC*ES/SPEED*60); // [s] Total measurement time
//const int TP=(int)(1000*TTOT/POS); // [ms] single step time 
int NP=(int)(TTOT*1000/TS); // total number of measurement point
int MAX_NC = 0; // max cycle number for the current setting



/*
 * Array used to save the results of the measurement process
 *    - vTime contains the value of the time for the current measure
 *    - vPosition contains the value of the displacement for the current measure
 *    - vLoad contains the value read from the load cell for the current measure
 */
long vTime[NN];
int  vPosition[NN];
float vLoad[NN];

/*
 * variables used to measure time 
 */
unsigned long time1, time0, wait_time;

/*
 * Global variables used for generic operation
 */
int x=0; // loop index
long P0=0; // initial position for the potentiometer
float limo=1; // variable used to set the distance that the motor must travel
char r0; // command received

/*
 * This variable is used to stop the execution of the machine if the use mode is changed
 */
boolean endOperation;
boolean abortop = false;

/*
 * Indicate if there is a measure already saved
 */
boolean savedMeasure = false;

/*
 * Object used to define the motor and to control it:
 *    - motor is a basic 2-wires configuratio for the motor
 *    - controller is the object that operate the driver for the motor
 */
Stepper motor(STEP, DIR);   
StepControl controller;    // Use default settings 

/*
 * SETUP
 */
void setup() {
  
     //Set default motor speed and acceleration
     motor.setMaxSpeed(VSTP);         // stp/s
     motor.setAcceleration(VSTP*3);    // stp/s^2
     
     //Set DRV8825 pin
     pinMode(ENBL, INPUT);
     pinMode(RES, OUTPUT);
     pinMode(SLP, OUTPUT);

     disable();// disable the motor
     
     pinMode(MODE0, OUTPUT);
     pinMode(MODE1, OUTPUT);
     pinMode(MODE2, OUTPUT);
     
     //Set microstep level
     digitalWrite(MODE0, MOD0V);
     digitalWrite(MODE1, MOD1V);
     digitalWrite(MODE2, MOD2V);
     
     pinMode(LED0, OUTPUT);
     pinMode(LED1, OUTPUT);
     pinMode(ABRT, INPUT);
     
     pinMode(MODS, INPUT);
     pinMode(SESP, OUTPUT);
     
     attachInterrupt(digitalPinToInterrupt(ABRT), abortOP, CHANGE);
     //attachInterrupt(digitalPinToInterrupt(ESPABORT), abortOP, CHANGE);
     
     
     
     scale.begin(P_DOUT, P_SCK);
     scale.set_scale(HX_SC);     // this value is obtained by calibrating the scale with known weights; see the README for details
     scale.tare();      // reset the tare to 0
    
     // ADC resolution - 12bit: 0-4095, 13bit: 0-8191
     adc->adc0->setAveraging(16); // set number of averages
     adc->adc0->setResolution(12); // set bits of resolution
     adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

     // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
     // see the documentation for more information
     // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
     // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
     adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
     // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
     adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
    
     //Serial1 is used for the communication with the ESP32
     Serial1.begin(115200);
     digitalWrite(SESP, LOW);
     Serial1.print("INIT");
     autoCalibration();
     delay(2000);
     Serial.begin(115200);

     //Set the use mode for the machine
     if(digitalRead(MODS) == HIGH){
        mod = true;
        selSerial = &Serial1;
        digitalWrite(SESP, HIGH);
        clearSerial();
     }else{
        mod = false;
        selSerial = &Serial;
        digitalWrite(SESP, LOW);
        clearSerial();
     }
     MAX_NC = maxNC(NC, SPEED, ES, TS);
     Serial.println(MAX_NC);
     //pinMode(ESINT, OUTPUT);
     //digitalWrite(ESINT, LOW);
     // Set the DATE and TIME 
     //flash_data("$TIME 09:00:00"); 
     //flash_data("$DATE 2018-01-01"); 
     //pinMode(ESPABORT, INPUT);
}

//------------------------------MAIN------------------------------
void loop() {
   switchMode();//switch use mode
   Serial.println("Traction Compression machine\n");
   endOperation=false;
   while(!endOperation){
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, LOW);

      //Print the Serial menu, only for serial monitor mode
      printMenu();

      //read the commad and and check if there has been a change of use mode
      r0 = serialSwitchOnCommand();

      switch (r0){
        /*
         * b command set the default parameter for SPEED, ES, TS, NC.
         * The default value are setted in the parameters.h
         */
        case 'b':
        case 'B':
          setDefaultParameters();
          break;
        /*
         * Command used to debugging purpose
         */
        case 'l':
        case 'L':
          sendMeasure();
          break;
        /*
         * u command move the column towards the load cell. 
         * When the load read from the cell is greater then 0.2 the column stops 
         */
        case 'u':
        case 'U':
          moveToSample();
          break;
        /*
         * j command start the autocalibration procedure
         */
        case 'j':
        case 'J':
          autoCalibration();
          break;
        /*
         * g command write the status of the machine on the selSerial
         */
        case 'g':
        case 'G':
          sendStatus();
          break;
        /*
         * h command return the column to the 0 position of the potentiometer
         */
        case 'h':
        case 'H':
          returnHome();
          break;
        /*
         * d command read the value for the displacement from the 
         */
        case 'D':
        case 'd':
          Serial.println("d command, insert displacement [mm]: ");
          Serial.println("Please, be sure that plates do not touch! ");
          setDisplacement();
          //TODO aggiungere invio errore ne caso di problema con i valori inseriti per il displacement
          break;
        case 'S':
        case 's':
          Serial.println("s command, insert speed [mm/min]: ");
          setMSpeed();
          break;
        case 'T':
        case 't':
          Serial.println("t command, insert sampling time [ms]: ");
          Serial.println(" 150ms<sampling time<1000ms ");
          setSamplingTime();
          break;
        case 'N':
        case 'n':
          Serial.println("n command, insert number of cycles: ");
          setNumberOfCycles();
          break;
        case 'C':
        case 'c':
          Serial.print("c command, move the column, insert (mm) (<0 go down): ");
          moveColumn();
          break;
        case 'A':
        case 'a':
          Serial.println("a command, reset actual tare to 0 ");
          scale.tare();      // reset the tare to 0
          break;
        case 'M':
        case 'm':
          Serial.println("Hai digitato il comando m. Effettua la misura ");
          startMeasure();
          break;
      default:
        Serial.println("Invalid command");
        delay(100);
     }
     abortop = false;      
   }
   delay(1000);
   Serial.println("Changing the use mode");    
}// END main loop-----------------------------

//--------------------FUNCTIONS----------------


/*
 * Print the menu for the serial monitor mode
 */
void printMenu(){
    Serial.println("________________________________________________");
    Serial.print("Initial load: ");
    Serial.print(scale.get_units(5));
    pos0(); // misura posizione iniziale P0
    Serial.print("N; pos: ");
    Serial.print(P0*SCE);
    Serial.print("mm; Stp/s: ");
    Serial.print(VSTP);
    Serial.print("; Tot time: ");
    Serial.print(TTOT);
    Serial.print("s; Tot steps: ");
    Serial.print(POSI, 1);
    Serial.print("; Meas. points: ");
    Serial.println(NP, 1);
    Serial.println("________________________________________________");
    Serial.println(" Select a command: ");
    Serial.print(" d - relative displacement [mm], (");
    Serial.print(ES);Serial.println("): ");
    Serial.print(" s - speed [mm/min], (");
    Serial.print(SPEED);Serial.println("): ");
    Serial.print(" t - sampling time [ms], (");
    Serial.print(TS);Serial.println("): ");
    Serial.print(" n - number of cycles, (");
    Serial.print(NC);Serial.println("): ");
    Serial.println(' ');
    Serial.println(" c - MOVE THE COLUMN [mm]");
    Serial.println(" h - MOVE THE COLUMN TO HOME POSITION");
    Serial.println(" u - MOVE THE COLOUMN TO SAMPLE FOR MEASURE");
    Serial.println(" a - RESET ACTUAL TARE TO ZERO");
    Serial.println(" b - RESET PARAMETERS TO DEFAULT VALUE");
    Serial.println(" m - MEASURE");
    Serial.println("________________________________________________");
    Serial.println(' ');
}//end printMenu-------------------------------


/*
 * Move the column to set the max position and the minimum position of the potentiometer
 * TODO
 */
void autoCalibration(){
    unsigned long currentMillis = millis();
    float curSpeed = SPEED;
    SPEED = 10;
    updateParameters();
    scale.tare();
    motor.setMaxSpeed(VSTP*10);
    motor.setAcceleration(VSTP*30);
    enable();
    motor.setTargetRel((int)(-50/PAS*MCRSTP));
    controller.moveAsync(motor);
    boolean stpm = false;
    while(controller.isRunning() && !stpm){
      if(millis() - currentMillis > 200){
        currentMillis = millis();
        if(digitalRead(LED1) == HIGH){
          digitalWrite(LED1, LOW);
        }else{
          digitalWrite(LED1, HIGH);
        }
      }
      if(scale.get_units(5) > 0.2){
        stpm = true;
        disable();
      }
    }
    disable();
    pos0();
    LMAX = P0*SCE;
    enable();
    motor.setTargetRel((int)(50/PAS*MCRSTP));
    controller.moveAsync(motor);
    stpm = false;
    currentMillis = millis();
    while(controller.isRunning() && !stpm){
      if(millis() - currentMillis > 200){
        currentMillis = millis();
        if(digitalRead(LED1) == HIGH){
          digitalWrite(LED1, LOW);
        }else{
          digitalWrite(LED1, HIGH);
        }
      }
      pos0();
      if(P0 <= 1){
        stpm = true;
        disable();
      }
    }
    disable();
    pos0();
    
    LMIN = float(P0)*SCE;
    SPEED = curSpeed;
    updateParameters();
    Serial.println("pos min: " + String(LMIN) + "\t pos max: " + String(LMAX));
    char c [10];
    String ll = String(P0*SCE);
    for(int i = 0; i< ll.length(); i++){
      c[i] = ll[i];
    }
    Serial.println(atof(c));
    
}//end autoCalibration()-----------------------

/*
 * Move the coloumn to the home position
 */
void returnHome(){
  pos0();
  if(P0 > 0){
     float curSpeed = SPEED;
     SPEED = 40;
     updateParameters();
     moveMotorUpWithControl(100);
     SPEED = curSpeed;
     pos0();
     updateParameters();
     selSerial->print("OK");
  }
}//end returnHome------------------------------

/*
 * Move the column towards the sample
 */
void moveToSample(){
   float currSpeed = SPEED;
   SPEED = 10;
   updateParameters();
   moveMotorDownWithControl(-100);
   SPEED = currSpeed;
   pos0();
   updateParameters();
   selSerial->print("OK");
}//end moveToSample----------------------------

/*
 * move the column by a given position
 */
void moveColumn(){
   while(!selSerial->available()){
    //Do Nothing unless there is a reply!
   }
   limo=selSerial->parseFloat();
   Serial.println(limo);
   if(limo > 0){
     moveMotorUpWithControl(limo);
   }else{
     moveMotorDownWithControl(limo);
   }// Do the move
   pos0(); // misura posizione estensimetro 
   selSerial->print("OK");
   //sendInterrupt();
}//end moveColumn------------------------------

/*
 * write the machine status on the selected serial
 */
void sendStatus(){
   String mst = "";
   mst = mst + SPEED + "#";
   mst = mst + ES + "#";
   mst = mst + TS + "#";
   mst = mst + NC + "#";
   mst = mst + P0*SCE + "#";
   mst = mst + VSTP + "#";
   mst = mst + TTOT + "#";
   mst = mst + NP + "#";
   mst = mst + scale.get_units(5) + "#";
   mst = mst + POSI + "#";
   mst = mst + MAX_NC + "#";
   mst = mst + LMIN + "#" + LMAX + "#" + MINTS + "#" + MAXTS + "#" + MINSPEED + "#" + MAXSPEED +"##";
   selSerial->print(mst);
   selSerial->print("OK");
   Serial.print("sendState");
}//end sendStatus()----------------------------

void sendFirstConfig(){
  String mst = "";
  mst = mst + LMIN + "#" + LMAX + "#" + MINTS + "#" + MAXTS + "#" + MINSPEED + "#" + MAXSPEED +"##";
  selSerial->print(mst);
  selSerial->print("OK");
}
/*
 * Set the displacement
 */
void setDisplacement(){
  
  while(!selSerial->available()){
           //Do Nothing unless there is a reply!
  }
  float curES=selSerial->parseFloat();
  if(curES >= LMIN-0.001 && curES <= LMAX+0.001){
    int curMAXNC = maxNC(NC, SPEED, curES, TS);
    if(curMAXNC < 0){
      selSerial->print("NOKMP");
    }else{
      ES0=ES;
      ES = curES;
      MAX_NC = curMAXNC;
      updateParameters();
      selSerial->print("OK");
    }
  }else{
    selSerial->print("NOK");
  }
  
  
}//end setDisplacement-------------------------

/*
 * Set Motor speed
 */
void setMSpeed(){
   while(!selSerial->available()){
      //Do Nothing unless there is a reply!
   }
   float curSPEED=selSerial->parseFloat();
   if(curSPEED < MINSPEED || curSPEED > MAXSPEED){
     selSerial->print("NOK");
   }else{
     int curMAXNC = maxNC(NC, curSPEED, ES, TS);
     if(curMAXNC < 0){
        selSerial->print("NOKMP");
     }else{
        SPEED = curSPEED;
        MAX_NC = curMAXNC;
        updateParameters();
        selSerial->print("OK");
     }
   }
   
}//end setMSpeed();

/*
 * Set sampling time
 */
void setSamplingTime(){
   while(!selSerial->available()){
     //Do Nothing unless there is a reply!
   }
   int curTS=selSerial->parseInt();
   if(curTS < MINTS || curTS > MAXTS){
      selSerial->print("NOK");
   }else{
     int curMAXNC = maxNC(NC, SPEED, ES, curTS);
     if(curMAXNC < 0){
        selSerial->print("NOKMP");
     }else{
        TS = curTS;
        MAX_NC = curMAXNC;
        updateParameters();
        selSerial->print("OK");
     }
   }
   
}//end setSamplingTime()-----------------------

/*
 * Set number of cycles
 */
void setNumberOfCycles(){
   while(!selSerial->available()){
           //Do Nothing unless there is a reply!
   }
   int curNC=selSerial->parseInt();
   if(curNC >= MAX_NC){
      selSerial->print("NOKMP");
   }else{
      NC = curNC;
      updateParameters();
      selSerial->print("OK"); 
   }
   
}//end setNumberOfCycles-----------------------


/*
 * Start measuring process
 */
void startMeasure(){
    savedMeasure = false;
    digitalWrite(LED1, HIGH);
    pos0(); // read initial position
    motor.setMaxSpeed(VSTP);         // stp/s
    motor.setAcceleration(VSTP*3);    // stp/s^2
    delay(500);
    enable();
    time0 = millis();
    x=0;
    // measurement timer
    timer0.begin(getdata, TS*1000); // every TS*1000 microseconds, getdata
    for (int i=1; (i <= NC) && (!abortop); i++){ //repeat NC time
       motor.setTargetRel(-POSI);  // Set target position to 1000 steps from current position
       controller.moveAsync(motor);    // Do the move
       while(controller.isRunning() && !abortop){              // wait until the movement is finished
          delay(10);                     
       }
       motor.setTargetRel(POSI);  // Set target position to 1000 steps from current position
       controller.moveAsync(motor);    // Do the move
       while(controller.isRunning() && !abortop){              // wait until the movement is finished
          delay(10);                     
       }
       Serial.println("DONE Cycle Number: " + String(x));
    }
    disable(); //disabilita motore
    timer0.end(); // disabilita acquisizione
    if(abortop){
      selSerial->print("NOK");
    }else{
      digitalWrite(LED1, LOW); 
      ledBlink(LED0, 10, 100);
      savedMeasure = true;
      if(!mod){
         tempotot(); // stampa tempo totale
         printest(); // stampa seriale risultati
      }else{
         sendMeasure();
      }
    }       
}//end startMeasure----------------------------

//-----------------UTILITY FUNCTION---------------
/*
 * Send measure write the mesure to the externa serial device
 */
void sendMeasure(){
  int maxP = 40;
  clearSerial();
  selSerial->print("MSBEGIN");
  while(!(selSerial->available()>0) && !abortop){
  }
  if(selSerial->readString() == "OK"){
    if(!savedMeasure){
        selSerial->print("NOK");
    }else{
        String msm = "";
        int cNP = NP;
        bool sendM = false;
        if(cNP > 0){
          for(int i = 1; i < NP+1; i++){
            if(cNP >= maxP && !sendM){
              cNP = cNP - maxP;
              msm = String(maxP) + "#";
            }else if(cNP < maxP && !sendM){
              msm = String(cNP) + "#";
            }
             msm = msm + vTime[i-1] + "#";
             msm = msm + vPosition[i-1] + "#";
             msm = msm + vLoad[i-1] + "#";
             sendM = true;
             if((i%maxP == 0) || i == NP){
                sendM = false;
                selSerial->print(msm);
                while(!(selSerial->available()>0 && !abortop)){}
                if(selSerial->readString("CONTINUE")){
                  if(i == NP){
                    selSerial->print("MSEND");
                    Serial.println("END");
                  }else{
                    selSerial->print("MSCONTINUE");
                    Serial.println("CONTINUE");
                  }
                }
                while(!selSerial->available()&& !abortop){}
                if(selSerial->readString() == "OK"){}
             }
          }
        }
    }
    clearSerial();
  } 
}
/*
 * Set the machine parameter to the default value
 * Default value are stored in parameters.h
 */
void setDefaultParameters(){
  SPEED = DEF_SPEED;
  ES = DEF_ES;
  TS = DEF_TS;
  NC = DEF_NC;
  MAX_NC = maxNC (NC, SPEED, ES, TS);
  updateParameters();
  selSerial->print("OK");
}
/*
 * Calculate the max Number of cycles for the current setting
 * the function return :
 *  -  -1 if the number of measurement point esceeded the dimension of the arrays
 *  -  the max Number of cycles for the current setting
 */
int maxNC (int curNC, float curSPEED, float curES, int curTS){
    float tt = 2*curNC*curES/curSPEED*60;
    int cnp = (int)(tt*1000/curTS);
    if(cnp >=NN){
      return -1;
    }
    while(cnp < NN-1){
      tt = 2*curNC*curES/curSPEED*60;
      cnp = (int)(tt*1000/curTS);
      curNC++;
    }
    
    curNC--;
    Serial.print("max NC");
    Serial.println(curNC);
    return curNC;
  }
/*
 * Read the status of the sensors and fill the arrays for the measure
 */
void getdata(){ 
    digitalWrite(LED1, (x % 2) == 0);       //led link every TS (sample time)
                                            // TS sampleTime, N number of average
                                            // 13*N+1*N+1*N<=TS -->15*n<=TS N=TS/20
    //int N=(int)TS/20;
    time1 = millis();
    vTime[x]=(time1-time0);   // long 4 byte
    
    int N=2;
    vPosition[x] = getAveragePosition(50);
    vPosition[x] = (vPosition[x] - P0)*SCE * 100;
    vLoad[x]=(scale.get_units(N)); // float 4 byte
    x++; // increment index
}//end getdata---------------------------------

int getAveragePosition(int n){
    int aver = 0;
    int maxM = 0;
    int minM = 0;
    int curr = 0;
    for (int i = 0; i <= n; i++){
      curr = adc->analogRead(AP0);
      aver = aver + curr;
      if(curr < minM){
        minM = curr;
      }
      if(curr > maxM){
        maxM = curr;
      }
    }
    return (aver - maxM - minM)/(n-2);
}
/*
 * Print the results
 */
void printest(){
    int s=0;
    Serial.println("\n\rResults:");  
    Serial.println("# [ms] pos [N]");  
    while (s<=NP) {
      Serial.print(s);
      Serial.print("; ");
      Serial.print(vTime[s]);
      Serial.print("; ");
      Serial.print(vPosition[s]);
      Serial.print("; ");
      Serial.println(vLoad[s]);
      s++;
    }
    Serial.println("End results");  
}//end printest--------------------------------

/*
 * Update the status variables
 */
void updateParameters(){
    VSTP=(int)(SPEED/PAS/60*F1*MCRSTP); //[stp/s]
    POS=ES/PAS*MCRSTP;                  //step totali singola escursione
    POSI=(int)POS;                      //approssimazione agli interi
    TTOT=(2*NC*ES/SPEED*60);            //[s] stima tempo totale
    NP=(int)(TTOT*1000/TS);           //n.ro totale di punti da registrare
}//end aggiornaparametri------------------------

/*
 * Measure the positon of the potentiometer
 */
void pos0(){
    /*
    /*for(int xp0 = 0; xp0 < 100; xp0 ++){
      P0=P0+adc->analogRead(AP0);
    }*/
    P0 = getAveragePosition(50); 
}//end pos0-------------------------------------

/*
 * Verify possible error condition
 */
void verify(){ 
    time1 = millis();
    Serial.print("Measurement Time [s]: ");
    Serial.println((float)(time1-time0)/1000,2);   
}//end verify-----------------------------------

/*
 * Calculate the total measurement time
 */
void tempotot(){ 
    time1 = millis();
    Serial.print("Measurement time [s]: ");
    Serial.println((float)(time1-time0)/1000,2);   
}//end tempotot

/*
 * Enable steppermotor
 */
void enable(){ 
    digitalWrite(RES, HIGH);
    digitalWrite(SLP, HIGH);
}//end enable----------------------------------

/*
 * Disable steppermotor
 */
void disable(){ 
    digitalWrite(RES, LOW);
    digitalWrite(SLP, LOW);
}//end disable---------------------------------

/*
 * Move the motor down of a given value and check if the column touch the load cell
 */
boolean moveMotorDownWithControl(float mv){
    boolean stpm = false;
    if(mv < 0){
       scale.tare();
       motor.setMaxSpeed(VSTP*10);         // stp/s
       motor.setAcceleration(VSTP*30);
       enable();
       motor.setTargetRel((int)(mv/PAS*MCRSTP));  // Set target position to 1000 steps from current position
       controller.moveAsync(motor);// Do the move
       
       while(controller.isRunning()&& !stpm && !abortop){// wait until the movement is finished
          if(scale.get_units(5)>0.2){
              disable();
              stpm = true;                    
          }
       }
       disable();
    }
    return stpm;
}//end moveMotorDownWithControl----------------

/*
 * Move the motor up of a given value and check if the value exceeds the max HIGH
 */
boolean moveMotorUpWithControl(float mv){
    boolean stpm = false;
    if(mv > 0){
       motor.setMaxSpeed(VSTP*10);         // stp/s
       motor.setAcceleration(VSTP*30);    // stp/s^2
       enable(); //abilita motore
       motor.setTargetRel((int)(mv/PAS*MCRSTP));  // Set target position to 1000 steps from current position
       controller.moveAsync(motor);// Do the move
       
       while(controller.isRunning() && !stpm && !abortop){
          delay(10);
          pos0();
          if((P0 <= 1)){
              disable(); 
              stpm = true;                   
          }
       }
       disable();
    }
    return stpm;
}//end moveMotorDownWithControl-----------------

/*
 * Print function with built in delay
 */
void flash_data(char *pstring){
    Serial1.println(pstring); 
    delay(50); 
}//end flash_data------------------------------

/*
 * Cancel the movement of the motor via interrupt
 */
void abortOP(){
  //this function simply disable the motor
  disable();
  abortop = true;
  selSerial->write("NOK");
  Serial.println("ABORT CURRENT OPERATION");
}//end abortOP--------------------------------- 

/*
 * Blink a led for n time with a with a given delay
 */
void ledBlink(int led, int n, int dly){
  for(int i  = 0; i<= n; i++){
    digitalWrite(led, LOW);
    delay(dly);
    digitalWrite(led, HIGH);
    delay(dly);
  }
}//end ledBlink--------------------------------

/*
 * Change the use mode of the machine
 */
void switchMode(){
  if(mod){
    selSerial = &Serial1;
    Serial.println("Traction Compression machine started in web mode");
    digitalWrite(SESP, HIGH);
  }else{
    selSerial = &Serial;
    Serial.println("Traction Compression machine started in monitor mode");
    digitalWrite(SESP, LOW);
  }
}

void clearSerial(){
  while(selSerial->available()>0){
    selSerial->read();
  }
}
/*
 * Read a command and check if there has been a change of use mode 
 */
char serialSwitchOnCommand(){
   while(!selSerial->available() && !endOperation && !abortop){
      if((digitalRead(MODS) == HIGH) && !mod){
        mod = true;
        endOperation = true;
      }else if((digitalRead(MODS) == LOW) && mod){
        mod = false;
        endOperation = true;
      }
   }
   return selSerial->read();
}
