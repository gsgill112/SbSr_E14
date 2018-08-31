/*
 ******************************************************************************
 * @file    main.cpp
 * @author  GS Gill
 * @version V1.0.0
 * @date    14-SEP-2017
 * @brief   Smart Bike Smart Rider Source for NUCLEO-L476RG using
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board and
 *          BLE Expantion Board.
 ******************************************************************************
 *
 * Smart Bike Smart Rider Source for NUCLEO-L476RG
 * Please refer <> for further information and build instructions.
 *
 * Requires [SHIELD]
 *    +-- Nucleo IDB0XA1 BLE Shield
 *    +-- Nucleo IKS01A2 Motion Sensing Shield
 *
 * Requires [External]
 *    +-- Nextion 3.2" Enhanced Diaplsy
 *    +-- Hall Effect Sensors
 *    +-- Battery Module with Solar Charging
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "ble/BLE.h"
#include "LEDService.h"
#include "BatteryService.h"
#include "DeviceInformationService.h"
#include "XNucleoIKS01A2.h"

#define M_PI   3.14
#define cycleTyreRadius 0.2
//#define oneRevolutionDist 2*M_PI*cycleTyreRadius

//#define DEBUG 1                   //Enable Debug Statements 


// PIN Maps 
DigitalOut actuatedLED(LED1, 0);
DigitalOut errorLed(LED1, 0);            // error indicator 

InterruptIn mybutton(USER_BUTTON);           // TESTING STATEMENT    test
InterruptIn hallSensor(PA_12);               // Using st Morpho Connectors as all arduino pins are used by shields.

DigitalIn  dispBoot(PC_4);            // Connected to D4 :: Display Boot Sucessful  
DigitalIn  mPnP(PB_1);            // Connected to D10 :: Music PLay n Pause
DigitalIn  mRt(PB_2);            // Connected to D11 :: Music Rt
DigitalIn  rideS(PB_11);            // Connected to D5 :: Ride Screen
DigitalIn  bumpP(PB_12);            // Connected to D6 :: Bump Pressed
DigitalIn  scS(PB_13);            // Connected to D7 :: Self Check Screen
DigitalIn  musicS(PB_14);            // Connected to D8 :: Music Screen 
DigitalIn  mLt(PB_15);            // Connected to D9 :: Music Left



// Objects 
//Serial nextion(PC_4, PC_5);        // Sending Data to Nextion Display
Serial nextion(SERIAL_TX, SERIAL_RX);

Timer tick;

//Static Variables
static const char* ver="SbSr_V00";
static const char terminator = 0xFF;       // Nextion termination sequence
char* dispSpeed = "n0.val=";                              // Nextion RIDE screen commands
char* dispAvg = "n2.val=";
char* dispDist = "n1.val=";
char* accl_check = "t1.txt=\"Acc ";                                    // Nextion Self Check screen
char* hx_check = "t2.txt=\"Hx ";
char* temp_check = "t3.txt=\"Temp ";
char* gyro_check = "t4.txt=\"Gyro ";
char* bx_check = "t5.txt=\"Bk ";
char* ble_check = "t6.txt=\"BLE ";
char* wifi_check = "t7.txt=\"WiFi ";
char* pass = "PASS\"";
char* fail = "FAIL\"";

const static float seaLevelPressure = 1013.25;
const static float declinationAngle = 0.003839724;
float alt, LPS22HB_p, LPS22HB_p_previous, LPS22HB_t, HTS221_h, HTS221_t;
float angle;
int32_t LSM303AGR_a[3],LSM303AGR_m[3];
int32_t LSM6DSL_a[3],LSM6DSL_g[3];

// for Debug of sensors 
  uint8_t id;
  float value1, value2;
  char buffer1[32], buffer2[32];
  int32_t axes[3];
  
// Message Bank  
// To 
/*      Var                     Bank            Msg 
 *      greatingMessages        1               0-2
 *      pressureMessages        2               0-1
 *      ridingMessages          3               0-2
 *      weatherMessages         4               0-5
*/ 
  
const char* greatingMessages[3] = {
    "Good Morning",
    "Good Evening",
    "Good Day"
};
const char* pressureMessages[2] = {
    "Up Up Up the hill we go",
    "Down we goooooo.... Yeeaahhh....."
};
const char* ridingMessages[3] = {
    "Com'on its a Pleasent Fine AWESOME day :)",
    "Hey Its a Holiday ! I will pickup dirt :(",
    "I know you are sleeping :| "
};
const char* weatherMessages[6] = {
    "Its pleasent to ride bike :)",
    "Its not to bad :) a bit Humid :)",
    "Its HUMID !! :|",
    "Not Today, Its very Humid :\\",
    "Its Chilling :O",
    "Its Hot :<"
};

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

//Variables
static volatile bool  triggerSensorPolling = false;            // one sec delay for BL sensor Data
char nextionScreen = '0';                                      // Nextion Screen indicator
char nextionMusic = '0';                                       // Nextion Music controls indicator
bool nextionBoot = false;                                      // Nextion Sucessful Boot indicator
float totalDist, dist, ridingSpeed, avgSpeed[5], avgSpeedF;
uint8_t avgCtr, hallSensorCounter, timeP, timeN, timex, stopC;
char* dispMsg = "";
uint8_t messageInUse, BLEStat;

const static char     DEVICE_NAME[] = "IoToWSbSr";
static const uint16_t uuid16_list[] = {LEDService::LED_SERVICE_UUID,
                                       //GattService::UUID_BATTERY_SERVICE,
                                       GattService::UUID_DEVICE_INFORMATION_SERVICE};

LEDService *ledServicePtr;

void pressed(void)              // TESTING STATEMENT    test
{
    nextion.printf("Test Sucessful\n");
}

void irqcallback_hallSensor(void)
{
#ifdef DEBUG
    nextion.printf("In Hall Callback\n");
#endif
    errorLed = !errorLed;
    //hallSensor.disable_irq();
    stopC = 0;                        // enables calculation of Speed
    timex = tick.read_ms();
    hallSensorCounter++;
    tick.reset();
    //hallSensor.enable_irq();

}

/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if (fractPart >= i) {
      break;
    }
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}

void updateMessage(void){
    hum_temp->get_temperature(&HTS221_t);
    hum_temp->get_humidity(&HTS221_h);
    // printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, value1), print_double(buffer2, value2));
   
    // Use this in another function  
    if(HTS221_h < 70){
        messageInUse=0;
    }else if(HTS221_h > 70 && HTS221_h < 80){
        messageInUse=1;
    }else if(HTS221_h > 80 && HTS221_h < 90){
        messageInUse=2;
    }
    else{
        messageInUse=3;
    }
    if(HTS221_t < 20){
        messageInUse=4;
    }else if(HTS221_t > 20 && HTS221_t < 35){
        messageInUse=0;
    }else{
        messageInUse=5;
    }
    
    press_temp->get_temperature(&LPS22HB_t);
    press_temp->get_pressure(&LPS22HB_p);
    //printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer1, value1), print_double(buffer2, value2));
    if(LPS22HB_p_previous > LPS22HB_p){
        messageInUse=0;
    }else{
        messageInUse=1;
    }
    if(LPS22HB_t < 20){
        messageInUse=4;
    }else if(LPS22HB_t > 20 && LPS22HB_t < 35){
        messageInUse=0;
    }else{
        messageInUse=5;
    }
    
    //#ifdef DEBUG
        nextion.printf("%s\n",weatherMessages[messageInUse]);
    //#endif
}

void perodic(void)
{
#ifdef DEBUG
    nextion.printf("In perodic\n Start Speed Calc\n");
#endif

    if (tick.read_ms() > 10000) { //(tick.read_ms() < 100 || tick.read_ms() > 10000 )       // check if bike is ideal standing ..
        stopC = 1;
        //tick.reset();
    } else  stopC = 0;

    if (stopC == 0) {
        timex = timex * 100 * 60; //min
        ridingSpeed = (2 * M_PI * cycleTyreRadius) / timex ;   //  in mtr/sec
    } else if(stopC == 1) {
        ridingSpeed = 0;

    }
    
    #ifdef DEBUG
        nextion.printf("Riding Speed = %f\n",ridingSpeed);
    #endif
        
    totalDist = 2* M_PI * cycleTyreRadius * hallSensorCounter;     // calculating total distance

    #ifdef DEBUG
        nextion.printf("Total Distance  = %f\n",totalDist);
    #endif
    
    avgSpeed[avgCtr] = ridingSpeed;
    avgCtr++;
    if(avgCtr == 4) avgCtr = 0;

    for(int i = 0; i == 4; i++) {    // calculation of avg speed
        avgSpeedF += avgSpeed[i];
    }
    avgSpeedF = avgSpeedF/5;
    
    #ifdef DEBUG
        nextion.printf("Average Speed  = %f\n",avgSpeedF);
    #endif

    triggerSensorPolling = true;
    
    #ifdef DEBUG
        nextion.printf("Speedy Calc Over ---- \n Sensor Data Polling Begin\n");
    #endif    
    
    hum_temp->get_temperature(&HTS221_t);
    hum_temp->get_humidity(&HTS221_h);
    
    #ifdef DEBUG
    printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, value1), print_double(buffer2, value2));
    #endif
    
    press_temp->get_temperature(&LPS22HB_t);
    press_temp->get_pressure(&LPS22HB_p_previous);
    //wait(5);
    
    #ifdef DEBUG
    printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer1, value1), print_double(buffer2, value2));
    printf("---\r\n");
    #endif
    
    magnetometer->get_m_axes(LSM303AGR_m);
    
    #ifdef DEBUG
    printf("LSM303AGR [mag/mgauss]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    #endif
    
    accelerometer->get_x_axes(LSM303AGR_a);
    
    #ifdef DEBUG
    printf("LSM303AGR [acc/mg]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    #endif
    
    acc_gyro->get_x_axes(LSM6DSL_a);
    
    #ifdef DEBUG
    printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    #endif
    
    acc_gyro->get_g_axes(LSM6DSL_g);
    
    #ifdef DEBUG
    printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    #endif
    
    //Altitude computation based on http://keisan.casio.com/exec/system/1224585971 formula 
    alt = pow((double)(seaLevelPressure/LPS22HB_p),(double)(1/5.257)) - 1;
    alt = alt * (LPS22HB_t + 273.15);
    alt = alt / 0.0065;
    
    angle -= declinationAngle;
    // Correct for when signs are reversed.
    if(angle < 0){   
      angle += 2*M_PI;
    }
    // Check for wrap due to addition of declination.
    if(angle > 2*M_PI){
      angle -= 2*M_PI;
    }
    
    angle = angle * 180/M_PI;
    
    updateMessage();
    
    // Screen and Controlls Checks
    if(dispBoot){
        nextionBoot = true;
        nextionScreen ='B';
    }else if(rideS){
        nextionScreen ='R';
    }else if(bumpP){
        nextionScreen ='B';
    }else if(musicS){
        nextionScreen ='M';
    }else if(scS){
        nextionScreen ='S';
    }
    
    if(mPnP){
        nextionMusic ='P';
    }else if(mLt){
        nextionMusic ='L';
    }else if(mRt){
        nextionMusic ='R';
    }
    
    //Update the Display Screen
    if(nextionScreen =='R') {             // checking if nextio display is on rise screen
        nextion.printf(dispSpeed);                                   // updating Speed Value on Nextion Display
        nextion.printf("%d",(int)(ridingSpeed));
        nextion.printf("%c%c%c",terminator,terminator,terminator);
        
        nextion.printf(dispAvg);                                   // updating Avg Speed Value on Nextion Display
        nextion.printf("%d",(int)(avgSpeedF));
        nextion.printf("%c%c%c",terminator,terminator,terminator);
        
        nextion.printf(dispDist);                                   // updating Distance Value on Nextion Display
        nextion.printf("%d",(int)(totalDist));
        nextion.printf("%c%c%c",terminator,terminator,terminator);
    }else if(nextionScreen =='S' && scS){
        if(hum_temp->read_id(&id)){
            nextion.printf(hx_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }else {
            nextion.printf(hx_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
            nextion.printf(temp_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }
        if(press_temp->read_id(&id)){
            nextion.printf(bx_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }else{
            nextion.printf(bx_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }
        if(magnetometer->read_id(&id)){
            nextion.printf(temp_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }else {
            nextion.printf(temp_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);
        }
        if(accelerometer->read_id(&id)){
            nextion.printf(accl_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);   
        }else {
            nextion.printf(accl_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);   
        }
        if(acc_gyro->read_id(&id)){
            nextion.printf(gyro_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);   
        }else{
            nextion.printf(gyro_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);  
        }
        if(BLEStat){
            nextion.printf(ble_check);
            nextion.printf(pass);
            nextion.printf("%c%c%c",terminator,terminator,terminator);   
        }else{
            nextion.printf(ble_check);
            nextion.printf(fail);
            nextion.printf("%c%c%c",terminator,terminator,terminator);  
        }
    while(scS);
    }
    
    #ifdef DEBUG
        nextion.printf("sensor Collection Over \n perodic over\n");
    #endif
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    (void)params;
    BLE::Instance().gap().startAdvertising(); // restart advertising
}

/**
 * This callback allows the LEDService to receive updates to the ledState Characteristic.
 *
 * @param[in] params
 *     Information about the characterisitc being updated.
 */
void onDataWrittenCallback(const GattWriteCallbackParams *params) {
    if ((params->handle == ledServicePtr->getValueHandle()) && (params->len == 1)) {
        actuatedLED = *(params->data);
    }
}

/** 
 * This function is called when the ble initialization process has failled 
 */ 
void onBleInitError(BLE &ble, ble_error_t error) 
{ 
    /* Initialization error handling should go here */ 
    BLEStat = 1;
} 

/** 
 * Callback triggered when the ble initialization process has finished 
 */ 
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params) 
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gattServer().onDataWritten(onDataWrittenCallback);

    /* Setup Device Identification service. */
    DeviceInformationService devInfoService(ble, "ST Micro", "NUCLEO-L476RG", "XXXX", "v1.0", "v1.0", "v1.0");
    
    bool initialValueForLEDCharacteristic = true;
    ledServicePtr = new LEDService(ble, initialValueForLEDCharacteristic);

    /* setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms. */
    ble.gap().startAdvertising();

    while (true) {
        ble.waitForEvent();
    }
}

int main(void)
{
    stopC = 1;                         // skips calculation of speed if set to : 1
    BLEStat = 0;
    hallSensorCounter = 0;
    nextion.baud(9600);                //Setting Up nextion Display  see the Issues.txt for more info
    errorLed = 1;      //To confirm System has Started :: Loading Screen on the Display
    wait(0.5);
    errorLed = 0;
#ifdef DEBUG
    nextion.printf("Welcome To SbSr Version %s\n",ver); // Serial Print only Debug MODE as serial is shared with Nextion Display
#endif
    wait(0.1);
    
    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    
    mybutton.fall(&pressed);                          // TESTING STATEMENT    test
    hallSensor.fall(&irqcallback_hallSensor);
    
    Ticker ticToc;
    ticToc.attach(perodic, 0.2f); // blink LED every second
    
    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);
}

