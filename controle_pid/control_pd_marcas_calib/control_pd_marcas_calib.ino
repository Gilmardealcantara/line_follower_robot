#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install

//pwm 
#define PWMMIN 50
#define PWMMAX 210
#define Kp 300.0// var angular de 0 - 2123
#define Kd 10.0
#define Ki 2.0


//Degug
bool debugSen = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               ;
bool debugMotor = false;
bool debugMark = true;

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

//--------------------------------DimensÃµes do robÃ´
//dimensÃµes do robo
#define COMP 0.1316
#define EIXO 0.1379
#define RAIO 0.0313

//--------------------------------DistÃ¢ncias dos sensores
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
#define SMARKDIR A7

//pino para curvas
#define SMARKESQ A0
#define TIMEEMFAIXA 100 //Tempo que vai andar para frente na faixa de pedestre
//--------------------------------Leitura dos sensores
//valor mÃ¡ximo de leitura do sensor /10
#define READMAX 1024
unsigned int sensRead[8];
unsigned int sensMarkDir,sensMarkEsq,sensCurve;
double MARKC;
double THRESHMARK = 180;

//-------------Novas funÃ§oes para o CORA ---------------------
long tstart;
boolean emcurva;
boolean trocaestado;
boolean INVERSAO;
boolean FAXAAVIR;

void sigareto(int periodo){
  if(millis()-tstart<periodo){
     sensRead[0]= 80;
     sensRead[1]= 80;
     sensRead[2]= 400;
     sensRead[3]= 700;
     sensRead[4]= 700;
     sensRead[5]= 400;
     sensRead[6]= 80;
     sensRead[7]= 80;
  }else{
     EMFAIXA=false; 
     FAIXAAVIR=false; 
  }
}

void detectainvecao(){
    if((sensRead[3]< THRESHMARK || sensRead[4]<THRESHMARK) && 
    sensRead[0]>THRESHMARK && sensRead[1]>THRESHMARK && sensRead[6]>THRESHMARK && sensRead[7]>THRESHMARK
    sensMarkDir>THRESHMARK && sensMarkEsq>THRESHMARK){
      INVERSAO=true;   
    } else{
     if(INVERSAO==true)
       FAXAAVIR=true; 
     INVERSAO=false;
    } 
       
}

void faixadepedestre(){
    
    if(FAIXAAVIR==true){
      if(sensRead[3]< THRESHMARK && sensRead[4]<THRESHMARK && 
        sensRead[0]<THRESHMARK && sensRead[1]<THRESHMARK && sensRead[6]<THRESHMARK && sensRead[7]<THRESHMARK
        sensMarkDir<THRESHMARK && sensMarkEsq<THRESHMARK){
          if(FAIXAAVIR==true && EMFAIXA==false){
             delay(5100);
             tstart=millis();
             EMFAIXA=true;                
          }
      }
    }
    if(EMFAIXA==true)
      sigareto(TIMEEMFAIXA);
    
  
}


void curvafechadaDir(){
 // Serial.print ("LE ");
  //Serial.print (sensRead[1]);
  if (sensRead[0]<THRESHMARK){

      /* Serial.print(" ");
       Serial.print (tstart);
       Serial.print (" Inicio");
       Serial.print (sensRead[1]);
       Serial.print ("\n");*/
       motorDir.setSpeed(100);
       motorEsq.setSpeed(100); 
       motorDir.run(BACKWARD);
       motorEsq.run(FORWARD);
  }else{
       emcurva=false;
     /*  Serial.print ("Parei");
       Serial.print (sensRead[1]);
       Serial.print ("\n");*/
       motorDir.setSpeed(0);
       motorEsq.setSpeed(0); 
       motorDir.run(FORWARD);
       motorEsq.run(FORWARD);
     
   //emcurva=false; 
  }
  
}
void curvafechadaEsq(){
   if (sensRead[7]<THRESHMARK){      
       motorEsq.setSpeed(100);
       motorDir.setSpeed(100); 
       motorEsq.run(BACKWARD);
       motorDir.run(FORWARD);
  }else{
       emcurva=false;
       motorDir.setSpeed(0);
       motorEsq.setSpeed(0); 
       motorDir.run(FORWARD);
       motorEsq.run(FORWARD);
     
   //emcurva=false; 
  }
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
#define TDirEsq 10 //Tempo maximo entre a leitura do lado esquerdo e direito
//Conta as marcas diretas
int contDir;
//Conta as marcas esquerdas
int contEsq;
unsigned long tmarkDir, tmarkEsq;//Tempo da ultima leitura de linha do lado esquerdo e direito, usado pra sincronizar as duas leituras
unsigned long tmark1,tmark2;


void novocontamarcaDir(){
  int novamarca;
  //Rotina padrao enquanto nao indentificou nenhum marca branca
  if((tmark1 == INF)){
    if(debugMark){
      Serial.print("Dira: "); 
      Serial.print(sensMarkDir); Serial.print(" "); 
    }
 
    if (sensMarkDir >= THRESHMARK) {
          Serial.print("Nova  \n");  
          if(sensRead[1] < THRESHMARK){
            Serial.print("Nova marca \n");           
            countDir++;
          }else{
             Serial.print("Nova linha\n");
             countLine++;
          } 
          tmark1 = millis();
          //Serial.print ("Alguma coisa");
    }
  }else if(tmark1 != INF){
    if ((millis() - tmark1) >= TBMARKS) {
      tmark1 = INF;
    }
  }  
}





int state;

void tomadordedecisao(){
  unsigned long wait; //Conta o tempo a partir que um maraca foi contada
 // contamarcasDir();
  if(tmarkDir!=INF || tmarkEsq != INF){
    wait=millis();
    if((millis()-wait)<=TDirEsq){
        if(tmarkDir!=INF && tmarkEsq != INF){//Se as duas marcas foram lidas 
         //Vira para o lado pre definido
       }
    }
    else{ //Se marca for so de um lado 
        if(contDir==1) curvafechadaDir();
        else if(contEsq==1) curvafechadaEsq();
  //    else if(contDir>1)rotatoria(contDir);
      tmarkDir=tmarkEsq=INF;
    }    
  }
  
}

void readSens(){
  if(INVERSAO==false){
    sensRead[0]=READMAX-analogRead(S1PIN);
    sensRead[1]=READMAX-analogRead(S2PIN);
    sensRead[2]=READMAX-analogRead(S3PIN);
    sensRead[3]=READMAX-analogRead(S4PIN);
    sensRead[4]=READMAX-analogRead(S5PIN);
    sensRead[5]=READMAX-analogRead(S6PIN);
    sensRead[6]=READMAX-analogRead(S7PIN);
    sensRead[7]=READMAX-analogRead(S8PIN);
    sensMarkDir=READMAX-analogRead(SMARKDIR);
    sensMarkEsq=READMAX-analogRead(SMARKESQ);
  }else{
    sensRead[0]=analogRead(S1PIN);
    sensRead[1]=analogRead(S2PIN);
    sensRead[2]=analogRead(S3PIN);
    sensRead[3]=analogRead(S4PIN);
    sensRead[4]=analogRead(S5PIN);
    sensRead[5]=analogRead(S6PIN);
    sensRead[6]=analogRead(S7PIN);
    sensRead[7]=analogRead(S8PIN);
    sensMarkDir=analogRead(SMARKDIR);
    sensMarkEsq=analogRead(SMARKESQ);    
  }
  if(debugSen){
    Serial.print(" "); Serial.print(sensRead[0]);
    Serial.print(" "); Serial.print(sensRead[1]);
    Serial.print(" "); Serial.print(sensRead[2]);
    Serial.print(" "); Serial.print(sensRead[3]);
    Serial.print(" "); Serial.print(sensRead[4]);
    Serial.print(" "); Serial.print(sensRead[5]);
    Serial.print(" "); Serial.print(sensRead[6]);
    Serial.print(" "); Serial.print(sensRead[7]);
    Serial.print(" Dir "); Serial.print(sensMarkDir);
    Serial.print(" Esq "); Serial.print(sensMarkEsq);  
  }
}

//--------------------------------Rotina de calibraÃ§Ã£o dos sensores
//valor de corte das marcas
//#define MARKC 10
//#define THRE6


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

//--------------------------------DetecÃ§Ã£o de marcas
//tempo entre marcas
//#define TBMARKS 300
//controle de tempo entre marcas
//unsigned long tmark;
//estado

/*
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
}*/

//--------------------------------Control


//velocidade angular mÃ¡xima
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
  pinMode(SMARKDIR, INPUT);
  pinMode(SMARKESQ, INPUT);
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
  state=0;
  STATESM=0; 
  tmark1=tmark2=INF;
  lt = var = millis();
  contDir=contEsq=0;
  tmarkDir=tmarkEsq=INF;
  INVERSAO=false;
  FAXAAVIR=false;


  //Teste rotação 

   tstart=millis(); 
   emcurva=true;
 
}


void loop() {
  faixadepedestre();
  if(EMFAIXA==false)
    readSens();
 
  novocontamarcaDir();

  /*
  if ((millis() - lt) >  T ) {
    lt = millis();


    centroid();
    //detectaMarcas();
    control();
   // MEstadoMotor();

    
    err_ante = err;
     
    if(debugSen || debugMotor){
     Serial.print("\terr: "); Serial.print(err, 4);
    }
    */
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

    //}
  
}
