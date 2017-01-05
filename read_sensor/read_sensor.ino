unsigned long lt;
//Peso dos sensores
#define DS1 -0.03375
#define DS2 -0.024
#define DS3 -0.0145
#define DS4 -0.0045
#define DS5 0.0045
#define DS6 0.0145
#define DS7 0.024
#define DS8 0.03375

//--------------------------------Pinos dos Sensores
//Pinos dos sensores da centroid
#define S1PIN A8
#define S2PIN A9
#define S3PIN A10
#define S4PIN A11
#define S5PIN A12
#define S6PIN A13
#define S7PIN A14
#define S8PIN A15

//Pino para marcas
#define SMARK A7
//pino para curvas
#define SCURVE A0

//--------------------------------Leitura dos sensores
//valor máximo de leitura do sensor /10
#define READMAX 1024
unsigned int sensRead[8];
unsigned int sensMark,sensCurve;

bool debug = false;

void readSens(){
  sensRead[0]=READMAX-analogRead(S1PIN);
  sensRead[1]=READMAX-analogRead(S2PIN);
  sensRead[2]=READMAX-analogRead(S3PIN);
  sensRead[3]=READMAX-analogRead(S4PIN);
  sensRead[4]=READMAX-analogRead(S5PIN);
  sensRead[5]=READMAX-analogRead(S6PIN);
  sensRead[6]=READMAX-analogRead(S7PIN);
  sensRead[7]=READMAX-analogRead(S8PIN);
  sensMark=READMAX-analogRead(SMARK);
  sensCurve=READMAX-analogRead(SCURVE);
  
  if(debug){
    Serial.print(" "); Serial.print(sensRead[0]);
    Serial.print(" "); Serial.print(sensRead[1]);
    Serial.print(" "); Serial.print(sensRead[2]);
    Serial.print(" "); Serial.print(sensRead[3]);
    Serial.print(" "); Serial.print(sensRead[4]);
    Serial.print(" "); Serial.print(sensRead[5]);
    Serial.print(" "); Serial.print(sensRead[6]);
    Serial.print(" "); Serial.print(sensRead[7]);
  }
}


//calculo centroid
double WSens[8] = {DS1, DS2, DS3, DS4, DS5, DS6, DS7, DS8};
double err;

void centroid(){
  int i;
  double sum = 0.0, sumW = 0.0;

  readSens();

  for (i = 0; i < 8; i++) {
    sumW += (double)(sensRead[i] * WSens[i]);
    sum += (double)sensRead[i];
  }
  //Serial.print("\tSW: "); Serial.print(sumW, 4);
  //Serial.print("\tSS: "); Serial.print(sum, 4);
  //err = sumW / sum;
  err = sumW / (sum/8);
}

void setup() {
   Serial.begin(9600);
   lt = millis();
}

//tempo de amostragem
#define T 5
void loop() {
   if ((millis() - lt) > T) {
    lt = millis();
    centroid();
   
     if(debug){
      Serial.print("\terr: "); Serial.println(err, 4);
    }  
   
    
  }
}
