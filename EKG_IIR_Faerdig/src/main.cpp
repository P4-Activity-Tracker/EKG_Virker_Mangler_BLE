#include <Arduino.h>
#include <HardwareSerial.h>
#include <Testdata.h>

HardwareSerial DataSerial(1);

#define FILTER_LENGTH 4
#define Array_Length 500

#define INCLUDE_vTaskDelayUntil 1


//Buffere som filtreret data lægges i
float EKGbuffer[Array_Length]; //Buffer der fyldes til 499 med filtreret data og overføres til filtData
float filtData[Array_Length];   //Buffer der benyttes til Normalisering og FindPeaks

//Buffer til IIR filter samt B og A koefficienter
float x[FILTER_LENGTH + 1] = {0, 0, 0, 0, 0};
float y[FILTER_LENGTH + 1] = {0, 0, 0, 0, 0};
float b[FILTER_LENGTH + 1] = {0.3503,  0,   -0.7007,  0,    0.3503};
float a[FILTER_LENGTH + 1] = {1.0000, -1.4953, 0.4566, -0.1171, 0.1802};

//Variabel til at holde max værdi
float maxV;

//Buffer til normalisering af data
float y_norm[Array_Length];

//Variablet til antal hjerteslag
float beats;
float BPM;
float finalBeat;
float firstBeat;
//variabler til findepeaks

//Variabler tid måling af eksekveringstid
//uint32_t t1,t2,tid; 

const float beatThreshold = 0.52;
const float beatTimeout = 25; // 0,3 fordi ved en puls på 200 vil der være en afstand på 0,3sek, og der er 100 samples pr. sekund
                    // !!!Sæt således at max puls er 250!!!


//Variabler til læs af sreiel procEKGTaskHandler
uint8_t s1RXpin = 17;
uint8_t s1TXpin = 16;

int16_t serialValue = 0;




//Task der sampler og flitrerer EKG
void sampleEKGDataTask(void *pvParamaters);
//Task der normaliserer og finder peaks
void processEKGDataTask(void *pvParameters);

// Task handler til sample EKG data task
TaskHandle_t EKGTaskHandler;
// Task handler til process activity data task
TaskHandle_t procEKGTaskHandler;

float floor_and_convert(float value)
{
  if (value > 0) // positiv
  {
    return (float)(value + 0.5);
  }
  else // negativ
  {
    return (float)(value - 0.5);
  }
}

void Printarray(float Var_array2[Array_Length],float Array_Len)
{
  for (uint16_t i = 0; i < Array_Len; i++){
    Serial.println(Var_array2[i]);
  }
}

void readTwoBytes(int16_t *val) {
  if (DataSerial.available() > 1) {
    byte lowbyte = DataSerial.read();
    *val = DataSerial.read();
    *val = ((*val)<<8) + lowbyte;
    //byte lowbyte = DataSerial.read();
    //*val = DataSerial.read();
    //*val = ((*val)<<8) + DataSerial.read();
  }
}


float iir_filter(int16_t value)
{
  x[0] =  (float) (value);           // Read received sample and perform typecast
  y[0] = b[0] * x[0];                 // Run IIR filter for first element
  for (int i = 1; i <= FILTER_LENGTH; i++) // Run IIR filter for all other elements
  {
    y[0] += b[i] * x[i] - a[i] * y[i];
  }
  for (int i = FILTER_LENGTH - 1; i >= 0; i--) // Roll x and y arrays in order to hold old sample inputs and outputs
  {
    x[i + 1] = x[i];
    y[i + 1] = y[i];
  }
  //return floor_and_convert(y[0]);     // fix rounding issues;
  return (float) y[0];
}

float findMax(float var_y[Array_Length], float maxvaerdi) {
  for(uint16_t i=0 ;i<Array_Length;i++){
  if (var_y[i] > maxvaerdi) {
    maxvaerdi = var_y[i];
  }
  
}
return maxvaerdi;
}

void copyArrayData(float *fromArray, float *toArray, int16_t Array_Len)
{
  for (uint16_t i = 0; i < Array_Len; i++)
  {
    *(toArray + i) = *(fromArray + i);
  }
}

void Normalize(float *Norm_Array, float *Filt_Array) //Variablet Array
{
  if (maxV <= 0) {
    maxV = 1;
  }
  for (uint16_t i = 0; i < Array_Length+1; i++)
  {
    *(Norm_Array + i) = *(Filt_Array + i) / maxV;
  }
}

float findPeaks(float *Var_array, float *fb, float *lb) //Var_Array = Variablet Array
{
  int16_t lastBeat = 0 - beatTimeout;
  for (uint16_t i = 1; i < Array_Length-1; i++)   //i 2 for i-2 kan lade sig gøre
  {
    if ((i - lastBeat) > beatTimeout){    

      if  ((*(Var_array+i) > beatThreshold)){
    
      	if ((*(Var_array+i)) >= (*(Var_array+i - 1)) 
      
        && (*(Var_array+i)) >= (*(Var_array+i + 1)))
    {
      lastBeat = i;
      //peaks(i-1) = y_norm(i-1);
      beats = beats + 1;
      if (beats == 1)
      {
        *fb = i;
      }
    }
    
  }
  *lb = lastBeat;
}
  }
  return beats;

}

float regnBPM (float Var_beats, float Var_finalbeat, float Var_firstbeat) //NOTE: Alle variabler konverteret fra int16_T til float
{
  if (Var_finalbeat != 0 && Var_firstbeat != 0 && Var_finalbeat != Var_firstbeat)
   {
    BPM = ((beats - 1 ) / (Var_finalbeat - Var_firstbeat)) * 6000;      // (6000 pga 60 sek og 100hz )
  }
  return BPM;
}

void setup() {
  Serial.begin(115200);
  //DataSerial.begin(115200, SERIAL_8N1, s1RXpin, s1TXpin);

  // Lav EKG sampler task
  xTaskCreate(
    sampleEKGDataTask,
    "Sample EKG data",
    1024,
    NULL,
    2,
    &EKGTaskHandler
  );

  // Lav process EKG data task
  xTaskCreate(
    processEKGDataTask,
    "process EKG data",
    8192,
    NULL,
    1,
    &procEKGTaskHandler
  );

  delay(100);

  vTaskResume(EKGTaskHandler);
}



void loop() {
  // put your main code here, to run repeatedly:
}

void sampleEKGDataTask(void *pvParapvParamaters) {
  // Task setup
 
  //Serial.println("Setting up EKG data sampling task");
  // Tick frekvens af task
  const TickType_t frequency = 10;
  //Gem nuværende tid
  TickType_t lastWakeTime = xTaskGetTickCount();

  //variabler til opsamling og filtrering af data
  int16_t y_org;
  //tæller til nuværende index af x buffer og y buffer
  uint16_t dataIndex = 0;

  //vTaskSuspend(NULL);
 
  for (;;) {
 
   // y_org = analogRead(5);
  
   //readTwoBytes(&y_org);

   y_org = EKGhvile[EKGdataIndex];
   EKGdataIndex++;
  
   
    iir_filter(y_org);
    EKGbuffer[dataIndex] = y[0];
    
    if (dataIndex >= Array_Length-1) {
      //sæt data Index tilbage til 0
      dataIndex = 0;

      //Kopier data fra rolling buffer til static buffer
      copyArrayData(EKGbuffer, filtData, Array_Length);
      //Serial.print("5 Sekunders buffer fyldt op");
      EKGdataIndex = 0;

     
            vTaskResume(procEKGTaskHandler);
  
      } else {
      //Optæl data index hvis bufferen ikke er fyldt endnu
      dataIndex++;
    
      }
    
  //Serial.println("Nået slutning af Sample loop");

    vTaskDelayUntil(&lastWakeTime, frequency);

  }
  // Vent indtil der skal samples igen  
}




// Task til normalisering og detektion af peaks
void processEKGDataTask(void *pvParameters) {
  // Task setup
  //Serial.println("Setting up data processing task");
  // Sæt task på pause indtil den skal bruges
  vTaskSuspend(NULL);
  // Task loop
  for (;;) {
    firstBeat = 0;
    finalBeat = 0;
    maxV = 0;
      beats = 0;

      
    //Serial.println("Running data processing");
    // Normaliser EKG
    maxV = findMax(filtData, maxV);
    Normalize(y_norm, filtData);
    //Serial.print("Én filtreret værdi");
    //Serial.print(y_norm[55]);

    //Serial.print("Har Normalized ");
    // Find  Peaks
    findPeaks(y_norm, &firstBeat, &finalBeat);
    Serial.println("Beats er:");
    Serial.println(beats);


    // Find Puls
    regnBPM(beats, finalBeat, firstBeat);


    Serial.println("BPM er:");
    Serial.println(BPM);

    //Serial.println("finalBeat");
   // Serial.println(finalBeat);
    Serial.println("firstBeat");
    Serial.println(firstBeat);
    //Serial.println("Beats =");
    //Serial.println(beats);
    //Serial.println("Har regnet bpm");
    //Serial.println(BPM);


    // Klargør task til næste data processering
  
    //activity = 0;
    //Suspend task indtil der er ny data klar
  
    
   //Serial.println("Printing Filtdata");
   //for (uint16_t i = 0; i<Array_Length-1;i++){
   // Serial.println(y_norm[i]);
  // }
   
   
    vTaskSuspend(NULL);
  }
}





