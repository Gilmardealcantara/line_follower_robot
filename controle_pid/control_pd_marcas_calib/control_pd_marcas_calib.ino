#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

//pwm 
#define PWMMIN 50
#define PWMMAX 210
#define Kp 300.0// var angular de 0 - 2123
#define Kd 10.0
#define Ki 2.0


//Maina
bool debugSen = false;
bool debugMotor = false;
bool debugMark = false;

//INF
#define INF 0xffffffff

//motor
typedef struct MotorsS {
  int pwmL;
  int pwmR;
} Motors;

Motors M;

AF_DCMotor motorEsq(1, MOTOR12_64KHZ);
AF_DCMotor motorDir(2, MOTOR12_64KHZ);

//--------------------------------Dimensões do robô
//dimensões do robô
#define COMP 0.1316
#define EIXO 0.1379
#define RAIO 0.0313

//--------------------------------Distâncias dos sensores
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
//#define SMARKDIR A7

//pino para curvas
//#define SMARCKESQ A0

//--------------------------------Leitura dos sensores
//valor máximo de leitura do sensor /10
#define READMAX 1024
unsigned int sensRead[8];
unsigned int sensMarkDir,sensCurve;

//-------------Novas funçoes para o CORA ---------------------

void curvafechadaDir(){
}
void curvafechadaDir(){
}
void seguindolinha(){
        motorEsq.setSpeed(constrain(PWMMIN + abs(M.pwmL), PWMMIN, PWMMAX));
        if (M.pwmL < 0) {
          //motorEsq.setSpeed(PWMMIN - M.pwmL);
          motorEsq.run(BACKWARD);
        } else {
          //motorEsq.setSpeed(PWMMIN + M.pwmL);
          motorEsq.run(FORWARD);
        }
        motorDir.setSpeed(constrain(PWMMIN + abs(M.pwmR), PWMMIN, PWMMAX));
        
        if (M.pwmR < 0) {
          //motorDir.setSpeed(PWMMIN - M.pwmR);
          motorDir.run(BACKWARD);
        } else {
          //motorDir.setSpeed(PWMMIN + M.pwmR);
          motorDir.run(FORWARD);
        }
}
#define TBMARKS 300 //Tempo indicando que saiu da regiao de marcas 
#define TBMARKSBIT 100 //Tempo para proxima verificaçao de marca
#define TDirEsq 10 //Tempo maximo entre a leitura do lado esquerdo e direito
//Conta as marcas diretas
int contDir;
//Conta as marcas esquerdas
int contEsq;
unsigned long tmarkDir, tmarkEsq;//Tempo da ultima leitura de linha do lado esquerdo e direito, usado pra sincronizar as duas leituras


void contamarcasDir(){
  unsigned long tmark;
  int novamarca;
  //Rotina padrao enquanto nao indentificou nenhum marca branca
  if((tmark == INF)){
    if(debugMark){
      Serial.print("Dir: "); 
      Serial.print(sensMarkDir); Serial.print(" ");
    }
    if (sensMarkDir >= THRESHMARK) {
          tmark = millis();
    }
  }else if(tmark != INF){
    //Diferencia se  uma marca de curva ou linha simples
    if ((millis() - tmark) >= TBMARKSBIT && (millis() - tmark) <= TBMARKS) { 
      if (sensMarkDir >= THRESHMARK && novamarca == false) { //Ainda esta lendo branco, adiciona 1 na contagem de marcas
          contDir++;
          novamarca=true;
      }            
    }else if ( (millis() - tmark) >= TBMARKS){ 
      if(novamarca == false){//Quer dizer que nao encontrou uma nova marca, entao  e' uma linha
        trocarestado=true; //ativa troca de estado na maquina de estado
        tmarkDir=millis();
      }
      novamarca=false;
      tmark=INF;
    }
  }  
}  
void contamarcasEsd(){
  unsigned long tmark;
  int novamarca;
  //Rotina padrao enquanto nao indentificou nenhum marca branca
  if((tmark == INF)){
    if(debugMark){
      Serial.print("Esq: "); 
      Serial.print(sensMarkEsq); 
      Serial.println("THERESHMARK:");Serial.print(HRESHMARK);
    }
    if (sensMarkEsq >= THRESHMARK) {
          tmark = millis();
    }
  }else if(tmark != INF){
    //Diferencia se  uma marca de curva ou linha simples
    if ((millis() - tmark) >= TBMARKSBIT && (millis() - tmark) <= TBMARKS) { 
      if (sensMarkEsq >= THRESHMARK && novamarca == false) { //Ainda esta lendo branco, adiciona 1 na contagem de marcas
          contDir++;
          novamarca=true;
      }            
    }else if ( (millis() - tmark) >= TBMARKS){ 
      if(novamarca == false){//Quer dizer que nao encontrou uma nova marca, entao  e' uma linha
        trocarestado=true; //ativa troca de estado na maquina de estado
        tmarkDir=millis(); 
      }
      novamarca=false;
      tmark=INF;
    }
  }  
}
int state;

void tomadordedecisao(){
  unsigned long wait; //Conta o tempo a partir que um maraca foi contada
  contamarcasDir();
  contamarcasEsd();
  if(tmarkDir!=INF || tmarkEsq != INF){
    wait=millis();
    if(millis-wait<=TDirEsq){
       if(tmarkDir!=INF && tmarkEsq != INF){//Se as duas marcas foram lidas 
         //Vira para o lado pre definido
       }
    }
    else{ //Se marca for so de um lado 
      if(contDir==1) curvafechadaDir();
      else if(contEsq==1) curvafechadaEsq();
      else if(contDir>1)rotatoria(contaDir);
      tmarkDir=tmarkEsq=INF;
    }    
  }
  
}

void readSens(){
  sensRead[0]=READMAX-analogRead(S1PIN);
  sensRead[1]=READMAX-analogRead(S2PIN);
  sensRead[2]=READMAX-analogRead(S3PIN);
  sensRead[3]=READMAX-analogRead(S4PIN);
  sensRead[4]=READMAX-analogRead(S5PIN);
  sensRead[5]=READMAX-analogRead(S6PIN);
  sensRead[6]=READMAX-analogRead(S7PIN);
  sensRead[7]=READMAX-analogRead(S8PIN);
  sensMarkDir=READMAX-analogRead(SMARKDIR);
  sensCurve=READMAX-analogRead(SCURVE);
  
  if(debugSen){
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

//--------------------------------Rotina de calibração dos sensores
//valor de corte das marcas
//#define MARKC 10
//#define THRE6
double MARKC;
double THRESHMARK = 200;

void calibrate(){
  int i,j;
  double pista,linha; //sensores Polulu

  MARKC=0.0;
  THRESHMARK=0.0;

  for(i=0;i<10;i++){
    pista=linha=0.0;
    readSens();
    MARKC+=((sensMarkDir+sensCurve)/2.0);
    for(j=0;j<8;j++){
      if((j==3)||(j==4)){
        linha+=sensRead[j];
      }else{
        pista+=sensRead[j];
      }
    }
    linha/=2.0;
    pista/=6.0;
    THRESHMARK+=((linha+pista)/2.0);
  }

  MARKC/=10.0;
  THRESHMARK/=10.0;
}

//--------------------------------Calculo da centroid
double WSens[8] = {DS1, DS2, DS3, DS4, DS5, DS6, DS7, DS8};
double err;

void centroid(){
  int i;
  double sum = 0.0, sumW = 0.0;

  readSens();

  for (i = 0; i < 8; i++) {
    sumW += (double)sensRead[i] * WSens[i];
    sum += (double)sensRead[i];
  }

  err = sumW / sum;
  //err = sumW / (sum/8);
}

//--------------------------------Detecção de marcas
//tempo entre marcas
//#define TBMARKS 300
//controle de tempo entre marcas
//unsigned long tmark;
//estado


void detectaMarcas(){
  if((tmark == INF)){
    if(debugMark){
      Serial.print(state); Serial.print(" ");
      Serial.print(sensMarkDir); Serial.print(" ");
      Serial.println(THRESHMARK);
    }
    switch(state) {
      case 10 :
        delay(500);
        motorEsq.setSpeed(0);
        motorDir.setSpeed(0);
        motorEsq.run(RELEASE);
        motorDir.run(RELEASE);
        delay(200000);
        break;
      default:
        if (sensMarkDir >= THRESHMARK) {
          state++;
          tmark = millis();
        }
    }
  } else if(tmark != INF){
    if ((millis() - tmark) >= TBMARKS) {
      tmark = INF;
    }
  }
}

//--------------------------------Control


//velocidade angular máxima
//#define Wmax 21.2622
#define Wmax 6.0

//tempo de amostragem
#define T 5

double Wk;
double Vk;
double Wr;
double Wl;
double deltaWRoda;
double eixoRaio = EIXO / RAIO;

//tempo
unsigned long lt, var;

double err_ante = 0, P = 0.0, I = 0.0, D = 0.0;
double a = ((PWMMAX - PWMMIN)/Wmax);

void control() {
  P = Kp * err;
  I += Ki * err *(T*0.001);
  D = Kd * (err - err_ante) / (T * 0.001);
  Wk = P + I + D;

  deltaWRoda = eixoRaio * Wk;
  Vk = ((2.0 * Wmax - abs(deltaWRoda)) / 2.0) * RAIO;
  Wl = ((2.0 * Vk - Wk * EIXO) / (2.0 * RAIO));
  Wr = ((2.0 * Vk + Wk * EIXO) / (2.0 * RAIO));

  M.pwmL = a*Wl;
  M.pwmR = a*Wr;
}


int STATESM;

void setup() {
   
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
  state=0;
  STATESM=0; 
  tmark=INF;
  lt = var = millis();
  contDir=contEsq=0;
  tmarkDir=tmarkEsq=INF;
}


void loop() {
  if ((millis() - lt) >  T ) {
    lt = millis();


    centroid();
    //detectaMarcas();
    control();
    MEstadoMotor();

    
    err_ante = err;
     
    if(debugSen || debugMotor){
     Serial.print("\terr: "); Serial.print(err, 4);
    }
    
    if(debugMotor){
      Serial.print("\t Wl: "); Serial.print(Wl);
      Serial.print(" Wr: "); Serial.print(Wr);

      Serial.print("\t ML_pwm: "); Serial.print(M.pwmL);
      Serial.print(" MR_pwm: "); Serial.print(M.pwmR);

      Serial.print("\t PWM_L_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmL), PWMMIN, PWMMAX));
      Serial.print(" PWM_R_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmR), PWMMIN, PWMMAX));
    }
    
    if(debugSen || debugMotor){
      Serial.println(" ");
    }

    }
  
}
