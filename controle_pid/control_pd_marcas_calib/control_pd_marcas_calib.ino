#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install
#include <avr/sleep.h>
//pwm
#define PWMMIN 50
#define PWMMAX 90
#define Kp 200.0// var angular de 0 - 2123
#define Kd 5.0
#define Ki 0.0


//Degug
bool debugSen = false;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               ;
bool debugMotor = false;
bool debugMark = false;
bool debugcountMark=false;
bool debugInversao = false;
bool debugRotatoria = false;
bool debugfaixadepedestre = false;
//INF
#define INF 0xffffffff
double THRESHSIDE = 250;
double THRESHMARK = 500;


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
unsigned int lastsensRead[8];
unsigned int sensMarkDir, sensMarkEsq;
double MARKC;

//-------------Novas funÃ§oes para o CORA ---------------------
long tstart;
boolean emcurva;
boolean trocaestado;
boolean INVERSAO;
boolean FAIXAAVIR;
boolean EMFAIXA;
boolean EMROTATORIA;
boolean ATRAVESSANDOFAIXA;
//Conta as marcas diretas
int countDir;
//Conta as marcas esquerdas
int countEsq;
int countLine;
unsigned long tmarkcurva,tmarks;
int NumeroMarcas;
double err, lasterr, lasterr_ante, err_ante = 0;


void detectainvecao() {
  if (FAIXAAVIR == false && EMFAIXA == false)
    if ((sensRead[3] < THRESHMARK || sensRead[4] < THRESHMARK) &&
        sensRead[0] > THRESHMARK && sensRead[1] > THRESHMARK && sensRead[6] > THRESHMARK && sensRead[7] > THRESHMARK &&
        sensMarkDir > THRESHSIDE && sensMarkEsq > THRESHSIDE) {
      if (INVERSAO == false) {
        INVERSAO = true;
        if (debugInversao == true)
          Serial.print ("\nInvesao detectada: BRANCO com linha PRETA \n");
      } else {

        if (debugInversao == true)
          Serial.print ("\nInvesao detectada: PRETO com linha BRANCA \n");

        if (debugInversao == true)
          Serial.print ("\nFaixa a vir \n");

        INVERSAO = false;
        FAIXAAVIR = true;
        tstart = millis();
        motorEsq.setSpeed(0);
        motorDir.setSpeed(0);
        motorDir.run(FORWARD);
        motorEsq.run(FORWARD);
      }

    }/*else{

      if(INVERSAO==true){


     }

     INVERSAO=false;*/


}

void faixadepedestre() {

  /*if(FAIXAAVIR==true){
    if(sensRead[3]< THRESHMARK && sensRead[4]<THRESHMARK &&
     sensRead[0]<THRESHMARK && sensRead[1]<THRESHMARK && sensRead[6]<THRESHMARK && sensRead[7]<THRESHMARK &&
     sensMarkDir<THRESHSIDE && sensMarkEsq<THRESHSIDE){
       if(debugfaixadepedestre==true)
         Serial.print("\nLeu tudo preto\n");
       /*if(FAIXAAVIR==true && EMFAIXA==false){
          if(debugfaixadepedestre==true)
             Serial.print("\nReconheceu ponto de parada\n");
          delay(5100);
          tstart=millis();
          EMFAIXA=true;
       }
    }
    }*/
  if (FAIXAAVIR == true) {

    if ((int)(millis() - 2000) > (int)(tstart)) {
      //Serial.print ("Consegui");
      EMFAIXA = true;
      FAIXAAVIR = false;
      //sigareto(TIMEEMFAIXA);

    } else {
      //Serial.print("Tentei");
      motorEsq.setSpeed(0);
      motorDir.setSpeed(0);
      motorDir.run(FORWARD);
      motorEsq.run(FORWARD);
    }
  }
  if (EMFAIXA == true) {
    if ((sensRead[0] < THRESHMARK || sensRead[1] < THRESHMARK) &&
        sensRead[2] < THRESHMARK && sensRead[3] < THRESHMARK && sensRead[4] < THRESHMARK && sensRead[5] < THRESHMARK  //Quando ver tudo ver 
        && sensRead[6] < THRESHMARK && sensRead[7] < THRESHMARK &&
        sensMarkDir < THRESHSIDE && sensMarkEsq < THRESHSIDE) {
      ATRAVESSANDOFAIXA = true;
    }


  }

}


void curvafechadaDir() {
     tmarkcurva=millis();
           while((int)(millis()-1100)<(int)tmarkcurva){
                  motorDir.setSpeed(0);
                  motorEsq.setSpeed(70);
                  motorEsq.run(FORWARD);
                  motorDir.run(FORWARD);
               
          }

}
void curvafechadaEsq() {
     tmarkcurva=millis();
     while((int)(millis()-1100)<(int)tmarkcurva){
                  motorDir.setSpeed(70);
                  motorEsq.setSpeed(0);
                  motorEsq.run(FORWARD);
                  motorDir.run(FORWARD);
              
      }
}
#define TBMARKS 300 //Tempo indicando que saiu da regiao de marcas 
#define TDirEsq 10 //Tempo maximo entre a leitura do lado esquerdo e direito

unsigned long tmarkDir, tmarkEsq;//Tempo da ultima leitura de linha do lado esquerdo e direito, usado pra sincronizar as duas leituras

void contamarca() {
    int novamarca;
    //Rotina padrao enquanto nao indentificou nenhum marca branca
    if ((tmarks == INF)) {
      
      if (sensMarkDir >= THRESHSIDE &&  sensRead[1] < THRESHMARK) {  //Se o sensor da direita ver branco e sensor 1 ver preto, conta uma marca a direita
        tmarks= millis();
        countDir++;
        if (debugcountMark == true) {
            Serial.print("\nN Marcas Dir:");
            Serial.print(countDir);
            Serial.print("\n");
          }
      }
      
      if (sensMarkEsq >= THRESHSIDE &&  sensRead[6] < THRESHMARK) {  //Se o sensor da Esquerda ver branco e sensor 1 ver preto, conta uma marca a direita
        tmarks= millis();
        countEsq++;
        if (debugcountMark == true) {
            Serial.print("N Marcas Esq:");
            Serial.print(countEsq);
            Serial.print("\n");
          }
      }
      
      if ( sensRead[0] > THRESHMARK   &&  sensRead[1] > THRESHMARK && sensRead[2] > THRESHMARK   &&  sensRead[3] > THRESHMARK 
      && sensRead[4] > THRESHMARK   &&  sensRead[5] > THRESHMARK && sensRead[6] > THRESHMARK   &&  sensRead[7] > THRESHMARK) {  //Se o sensor ler tudo branco
          tmarkcurva = millis();
          tmarks=millis();
          countLine=1;
          if(countDir > 0 || countEsq > 0){
            emcurva=true;
          }
        if (debugcountMark == true) {
            Serial.print("Nova linha:");
          }
      }
      

    } else if (tmarks != INF) {
      if (sensMarkDir <= THRESHSIDE && countDir>0   || sensMarkEsq <= THRESHSIDE && countEsq>0 || sensMarkDir < THRESHSIDE && sensMarkEsq < THRESHSIDE && countLine == 1) {
        tmarks = INF;
      }
     
    }
  }





  int state;


  int conDir, conEsq;
  void rotatoria() {
  
  if(countEsq==0 && countDir>1){ //Rotatoria para o lado direito
    switch (countDir) {
      case 2:
        if (sensMarkDir > THRESHMARK) {
              Serial.print("Case 2");
          curvafechadaDir();
          EMROTATORIA = false;
          countDir=countEsq=0;
        }
        break;
      case 3:
        // Serial.print("Caseo 3");
        if (sensMarkDir > THRESHMARK) {
          conDir++;
       //   Serial.print(" novaMarcaDir\n");
        }
       
        if (conDir == 2) {
         // Serial.print(" virando\n");
          curvafechadaDir();
          EMROTATORIA = false;
          countDir=countEsq=0;

        } 
        break;

      case 4:
        if (sensMarkDir > THRESHMARK) {
          conDir++;
        }
        if (conDir == 3) {
          curvafechadaDir();
          EMROTATORIA = false;
          countDir=countEsq=0;          
        } 

        break;
    }//switch end
  }
    if(countEsq>1 && countDir==0 ){//Rotatoria para o lado Esquerdo
      switch (countEsq) {
          case 2:
            //Serial.print("Caseo 2");
            if (sensRead[7] > THRESHMARK) {
              curvafechadaEsq();
              EMROTATORIA = false;
              countDir=countEsq=0;
            }
            break;
          case 3:
            // Serial.print("Caseo 3");
            if (sensRead[7] > THRESHMARK) {
              conEsq++;
              Serial.print(" novaMarcaDir\n");
            }
            if (countEsq == 2) {
              curvafechadaEsq();
              EMROTATORIA = false;
              countDir=countEsq=0;

            }
            break;
          case 4:
            if (sensRead[7] > THRESHMARK) {
              conEsq++;
            }
            if (conEsq == 3) {
              curvafechadaEsq();
              EMROTATORIA = false;
              countDir=countEsq=0;

            } 
            break;
        }//switch end
    }

  }//function end
  void curvasimples(){
    if(countLine==1){
      if(countDir==1&&countEsq==0){ //Vira para a direita
          curvafechadaDir();
          emcurva=false;
           countDir=0;
           countLine=0;
           if(debugcountMark==true)
              Serial.print ("Curva  Direita\n");
      }
       if(countDir==0&&countEsq==1){ //Vira para a Esquerda
           curvafechadaEsq(); 
           emcurva=false;
           countEsq=0;
           countLine=0;
           if(debugcountMark==true)
              Serial.print ("Curva  Esquerda \n");
      }
      if(countDir==1&&countEsq==1){ //Vira direita esques
           
      }
    }
  }
void tomadordedecisao() { //Funçao que gerencia a rotatoria     
    
      if((countEsq>1 || countDir>1) && countLine==1){
        if(emcurva==true){       
          if(countDir>1&&countEsq==0){
           curvafechadaDir();
           emcurva=false;
            if(debugcountMark==true)
              Serial.print ("Primeira curva rotatoria dir\n");
          }else if (countEsq>0&&countDir==0){
           curvafechadaEsq();
           emcurva=false;
            if(debugcountMark==true)
              Serial.print ("Primeira curva Rotatoria Esquerda\n");
          }
        }else{
          if(countEsq==0 && countDir>1 || countEsq>1 && countDir==0 ){
             Serial.print ("Em rotatoria\n"); 
             conDir=conEsq=0;
             EMROTATORIA=true;    
          } 
        }  
      }
    
  }

  void readSens() {
    if (ATRAVESSANDOFAIXA == false) { //Quando o programa estiver em sua rotina normal le o sensores e salva a leitura anterior
      lastsensRead[0] = sensRead[0]; //No momento que ATRAVESSANDOFAIXA é ativa, é so ira receber o valor da leitura antes de ter visto tudo preto
      lastsensRead[1] = sensRead[1];
      lastsensRead[2] = sensRead[2];
      lastsensRead[3] = sensRead[3];
      lastsensRead[4] = sensRead[4];
      lastsensRead[5] = sensRead[5];
      lastsensRead[6] = sensRead[6];
      lastsensRead[7] = sensRead[7];
    }
    if (INVERSAO == false) {
      sensRead[0] = READMAX - analogRead(S1PIN);
      sensRead[1] = READMAX - analogRead(S2PIN);
      sensRead[2] = READMAX - analogRead(S3PIN);
      sensRead[3] = READMAX - analogRead(S4PIN);
      sensRead[4] = READMAX - analogRead(S5PIN);
      sensRead[5] = READMAX - analogRead(S6PIN);
      sensRead[6] = READMAX - analogRead(S7PIN);
      sensRead[7] = READMAX - analogRead(S8PIN);
      sensMarkDir = READMAX - analogRead(SMARKDIR);
      sensMarkEsq = READMAX - analogRead(SMARKESQ);
    } else {
      sensRead[0] = analogRead(S1PIN);
      sensRead[1] = analogRead(S2PIN);
      sensRead[2] = analogRead(S3PIN);
      sensRead[3] = analogRead(S4PIN);
      sensRead[4] = analogRead(S5PIN);
      sensRead[5] = analogRead(S6PIN);
      sensRead[6] = analogRead(S7PIN);
      sensRead[7] = analogRead(S8PIN);
      sensMarkDir = analogRead(SMARKDIR);
      sensMarkEsq = analogRead(SMARKESQ);
    }
    if (ATRAVESSANDOFAIXA == true) {
      if (sensRead[3] > THRESHMARK || sensRead[4] > THRESHMARK ) { //Se  os 2 sensores do meio verem algo branco, ira faze o calculo usando a leitura dos 4 sensores centrais
        lastsensRead[2] = sensRead[2];
        lastsensRead[3] = sensRead[3];
        lastsensRead[4] = sensRead[4];
        lastsensRead[5] = sensRead[5];
        //  Serial.print ("\nEstou lendo a faixa de pedestre\n");
      }
    }
    if (ATRAVESSANDOFAIXA == true) {
      sensRead[0] = lastsensRead[0];
      sensRead[1] = lastsensRead[1];
      sensRead[2] = lastsensRead[2];
      sensRead[3] = lastsensRead[3];
      sensRead[4] = lastsensRead[4];
      sensRead[5] = lastsensRead[5];
      sensRead[6] = lastsensRead[6];
      sensRead[7] = lastsensRead[7];
   

    }


    if (debugSen) {
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

 void paradaTotal(){
    if(sensRead[0] <THRESHMARK && sensRead[1] <THRESHMARK && sensRead[2] <THRESHMARK && sensRead[3] <THRESHMARK && sensRead[4] < THRESHMARK &&  sensRead[5] <THRESHMARK 
    && sensRead[6] < THRESHMARK && sensRead[7] < THRESHMARK  && FAIXAAVIR == false) {
      while(1){
        motorEsq.setSpeed(0);
        motorDir.setSpeed(0);
        }
      }
    }

  //--------------------------------Calculo da centroid
  double WSens[8] = {DS1, DS2, DS3, DS4, DS5, DS6, DS7, DS8};


  void centroid() {
    int i;
    double sum = 0.0, sumW = 0.0;

    readSens();

    for (i = 0; i < 8; i++) {
      sumW += (double)sensRead[i] * WSens[i];
      sum += (double)sensRead[i];
    }

    // if(ATRAVESSANDOFAIXA==false){
    lasterr = err;
    err = sumW / sum;
    // }
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

  double P = 0.0, I = 0.0, D = 0.0;
  double a = ((PWMMAX - PWMMIN) / Wmax);

  void control() {
    P = Kp * err;
    I += Ki * err * (T * 0.001);
    D = Kd * (err - err_ante) / (T * 0.001);
    Wk = P + I + D;

    deltaWRoda = eixoRaio * Wk;
    Vk = ((2.0 * Wmax - abs(deltaWRoda)) / 2.0) * RAIO;
    Wl = ((2.0 * Vk - Wk * EIXO) / (2.0 * RAIO));
    Wr = ((2.0 * Vk + Wk * EIXO) / (2.0 * RAIO));

    M.pwmL = a * Wl;
    M.pwmR = a * Wr;
  }




  void setup() {
    pinMode(SMARKDIR, INPUT);
    pinMode(SMARKESQ, INPUT);
    motorEsq.setSpeed(0);
    motorDir.setSpeed(0);
    motorEsq.run(RELEASE);
    motorDir.run(RELEASE);
    Serial.begin(9600);
    state = 0;
   
    tmarkcurva = INF;
    lt = var = millis();
    countDir = countEsq = 0;
    tmarkDir = tmarkEsq = INF;
    INVERSAO = false;
    FAIXAAVIR = false;
    EMROTATORIA = false;
    EMFAIXA = false;
    ATRAVESSANDOFAIXA = false;
    //Teste rotação
    tmarks=INF;
    tstart = millis();
    
  }


  void loop() {

    if (FAIXAAVIR == false && emcurva==false)
      centroid(); 
    if(EMROTATORIA==false)  
      contamarca();
    tomadordedecisao(); //Usado para rotatoria
    curvasimples();
   // if (EMROTATORIA == true)
   //    rotatoria();
   
      
      
    //detectainvecao();
    //faixadepedestre();
    control();
    if (emcurva == false && FAIXAAVIR == false  ) {
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
      //if(ATRAVESSANDOFAIXA==false){
      lasterr_ante = err_ante;
      err_ante = err;

      // }
    } 
    if (debugSen || debugMotor) {
      Serial.print("\terr: "); Serial.print(err, 4);
    }

    if (debugMotor) {
      Serial.print("\t Wl: "); Serial.print(Wl);
      Serial.print(" Wr: "); Serial.print(Wr);

      Serial.print("\t ML_pwm: "); Serial.print(M.pwmL);
      Serial.print(" MR_pwm: "); Serial.print(M.pwmR);

      Serial.print("\t PWM_L_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmL), PWMMIN, PWMMAX));
      Serial.print(" PWM_R_real: "); Serial.print(constrain(PWMMIN + abs(M.pwmR), PWMMIN, PWMMAX));
    }

    if (debugSen || debugMotor) {
      Serial.println(" ");
    }



  }
