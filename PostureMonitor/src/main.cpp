#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEUtils.h>
#include <Wire.h>

#define DEVICE_NAME "ESP32 Posture monitor"

#define SERVICE_CONTROL_UUID            "5e28d7d9-ca4b-4515-ad38-abc65b550b82"
#define SERVICE_DELTA_UUID              "5e0a58c4-7c3a-4383-a6de-5dd61980defc"
#define SERVICE_WARNING_UUID            "9e05cc19-a99e-46cf-9021-fa50cecb34bf"

#define CONTROL_COORDINATES_FIRST_UUID  "bf902c6d-088a-4524-8cda-59708437da8f"
#define CONTROL_COORDINATES_SECOND_UUID "c2e2a8d7-5795-4de1-8fee-537b26ae6843"

#define DELTA_CALIBRATE_UUID            "7176d92d-0af4-42f2-949a-fd34188800de"
#define DELTA_LEAN_UUID                 "fa79718c-e7f8-435b-b9b6-21f309a485e5"
#define DELTA_COMPRESSION_UUID          "4e92962b-ff6e-4f42-8e76-f0e7fdd4fb82"
#define DELTA_TILT_UUID                 "531f584f-6566-4789-9962-0bdcbe1cd90c"

#define WARNING_TIME_UUID               "332898e4-6581-4249-97e1-9cc9db963e58"
#define WARNING_LEAN_UUID               "fe8c6f2f-92b2-4fee-a2bd-ebb6165180e0"
#define WARNING_COMPRESSION_UUID        "4e17ce99-53b2-4f68-8590-96654d05f841"
#define WARNING_TILT_UUID               "3c8d0834-485c-41f6-bdf3-9f7704922ac7"

BLECharacteristic* controlCoordinatesFirst;
BLECharacteristic* controlCoordinatesSecond;

BLECharacteristic* deltaCalibrate;
BLECharacteristic* deltaLean;
BLECharacteristic* deltaCompression;
BLECharacteristic* deltaTilt;

BLECharacteristic* warningLean;
BLECharacteristic* warningCompression;
BLECharacteristic* warningTilt;
BLECharacteristic* warningTime;

const int MPU_addr = 0x68; 
const int MPU1_addr = 0x69;

int16_t accX, accY, accZ;
float angAX, angAY, angAZ, ang1AX, ang1AY, ang1AZ;
float zeroangX, zeroangY, zeroangZ, zeroang1X, zeroang1Y, zeroang1Z;
float delta1, delta2, delta3;
float filterC = 0.1;
float filterW = 0.25;

bool isCalibrated = false;
bool isWarned1 = false;
bool isWarned2 = false;
bool isWarned3 = false;

unsigned long lastTime = 0;   
unsigned long lastWarnTime = 0;  
unsigned long lastWarnedTime = 0;
unsigned long accelerometerDelay = 50;
unsigned long warningDelay = 500;
unsigned long warningTimeDelay = 120000;

void zeroDeltaAngle(){
    zeroangX = angAX; 
    zeroang1X = ang1AX;
    zeroangY = angAY; 
    zeroang1Y = ang1AY;
    zeroangZ = angAZ; 
    zeroang1Z = ang1AZ;
}

class BluetoothEventCallback : public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic* characteristic){
    Serial.println("Input data: " + String(characteristic->getValue().c_str()));
    isCalibrated = true;
    zeroDeltaAngle();
    lastWarnedTime = millis();
  }
};

void setupBluetooth();

void setup() {
  Serial.begin(115200);
  Serial.println("Launching...");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU1_addr);
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true);

  setupBluetooth();
}

void calcAng(){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); 
    
    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    
    angAX = (RAD_TO_DEG * atan2(accY,accZ)) * filterC + angAX * (1 - filterC);
    angAY = (RAD_TO_DEG * atan2(accX,accZ)) * filterC + angAY * (1 - filterC);
    angAZ = (RAD_TO_DEG * atan2(accX,accY)) * filterC + angAZ * (1 - filterC);;
    
    Wire.beginTransmission(MPU1_addr);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1_addr, 14, true); 
    
    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    
    ang1AX = (RAD_TO_DEG * atan2(accY,accZ)) * filterC + ang1AX * (1 - filterC);
    ang1AY = (RAD_TO_DEG * atan2(accX,accZ)) * filterC + ang1AY * (1 - filterC);
    ang1AZ = (RAD_TO_DEG * atan2(accX,accY)) * filterC + ang1AZ * (1 - filterC);

    std::string s1("X: ");
    s1 += String(angAX).c_str();
    s1 += ", Y: ";
    s1 += String(angAY).c_str();
    s1 += ", Z: ";
    s1 += String(angAZ).c_str();
    std::string s2("X: ");
    s2 += String(ang1AX).c_str();
    s2 += ", Y: ";
    s2 += String(ang1AY).c_str();
    s2 += ", Z: ";
    s2 += String(ang1AZ).c_str();
    controlCoordinatesFirst->setValue(s1);
    controlCoordinatesSecond->setValue(s2);
}

void calcWarn(){
  if(isCalibrated){
    delta1 = ((abs(zeroangY) + abs(zeroang1Y)) - (abs(angAY) + abs(ang1AY))) * filterW + delta1 * (1 - filterW);
    delta2 = ((abs(zeroangX) + abs(zeroang1X)) - (abs(angAX) + abs(ang1AX))) * filterW + delta2 * (1 - filterW);
    delta3 = (abs(zeroangZ - angAZ) + abs(zeroang1Z - ang1AZ)) * filterC + delta3 * (1 - filterW);
  }
    deltaLean->setValue(String(delta1).c_str());
    deltaCompression->setValue(String(delta2).c_str());
    deltaTilt->setValue(String(delta3).c_str());
    if(delta1 >= 10){
      warningLean->setValue(String(1).c_str());
      isWarned1 = true;
    }else{
      warningLean->setValue(String(0).c_str());
      isWarned1 = false;
    }
    if(delta2 >= 5 && delta2 <= 20){
      warningCompression->setValue(String(1).c_str());
      isWarned2 = true;
    }else{
      warningCompression->setValue(String(0).c_str());
      isWarned2 = false;
    }
    if(abs(delta3) >= 4){
      warningTilt->setValue(String(1).c_str());
      isWarned3 = true;
    }else{
      warningTilt->setValue(String(0).c_str());
      isWarned3 = false;
    }
}

void loop() {
  if ((millis() - lastTime) > accelerometerDelay){
    
    calcAng();

    lastTime = millis();
  }
  if ((millis() - lastWarnTime) > warningDelay){
    
    calcWarn();

    lastWarnTime = millis();
  }
  if(isWarned1 == false && isWarned2 == false && isWarned3 == false){
      warningTime->setValue(String(0).c_str());
      lastWarnedTime = millis();
  } else{
      if ((millis() - lastWarnedTime) > warningTimeDelay){
      warningTime->setValue(String(1).c_str());
    }
  }
}

void setupBluetooth(){
  Serial.print("Starting Bluetooth...");
  BLEDevice::init(DEVICE_NAME);
  Serial.print(".");

  BLEServer* server = BLEDevice::createServer();
  BLEService* serviceControl = server -> createService(SERVICE_CONTROL_UUID);
  controlCoordinatesFirst = serviceControl -> createCharacteristic(CONTROL_COORDINATES_FIRST_UUID, BLECharacteristic::PROPERTY_READ);
  controlCoordinatesFirst -> addDescriptor(new BLE2902());
  controlCoordinatesSecond = serviceControl -> createCharacteristic(CONTROL_COORDINATES_SECOND_UUID, BLECharacteristic::PROPERTY_READ);
  controlCoordinatesSecond -> addDescriptor(new BLE2902());

  BLEService* serviceDelta = server -> createService(SERVICE_DELTA_UUID);
  deltaCalibrate  = serviceDelta -> createCharacteristic(DELTA_CALIBRATE_UUID, BLECharacteristic::PROPERTY_WRITE);
  deltaCalibrate -> setCallbacks(new BluetoothEventCallback());
  deltaLean = serviceDelta -> createCharacteristic(DELTA_LEAN_UUID, BLECharacteristic::PROPERTY_READ);
  deltaLean -> addDescriptor(new BLE2902());
  deltaCompression = serviceDelta -> createCharacteristic(DELTA_COMPRESSION_UUID, BLECharacteristic::PROPERTY_READ);
  deltaCompression -> addDescriptor(new BLE2902());
  deltaTilt = serviceDelta -> createCharacteristic(DELTA_TILT_UUID, BLECharacteristic::PROPERTY_READ);
  deltaTilt -> addDescriptor(new BLE2902());

  BLEService* serviceWarning = server -> createService(SERVICE_WARNING_UUID);
  warningLean = serviceWarning -> createCharacteristic(WARNING_LEAN_UUID, BLECharacteristic::PROPERTY_READ);
  warningLean -> addDescriptor(new BLE2902());
  warningCompression = serviceWarning -> createCharacteristic(WARNING_COMPRESSION_UUID, BLECharacteristic::PROPERTY_READ);
  warningCompression -> addDescriptor(new BLE2902());
  warningTilt = serviceWarning -> createCharacteristic(WARNING_TILT_UUID, BLECharacteristic::PROPERTY_READ);
  warningTilt -> addDescriptor(new BLE2902());
  warningTime = serviceWarning -> createCharacteristic(WARNING_TIME_UUID, BLECharacteristic::PROPERTY_READ);
  warningTime -> addDescriptor(new BLE2902());

  serviceControl->start();
  serviceDelta->start();
  serviceWarning->start();
  Serial.print(".");

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_CONTROL_UUID);
  advertising->addServiceUUID(SERVICE_DELTA_UUID);
  advertising->addServiceUUID(SERVICE_WARNING_UUID);
  advertising->start();
  Serial.print(".");
  Serial.println(" Bluetooth ready!");
}
