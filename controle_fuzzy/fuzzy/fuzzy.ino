/*
Distancias podem ser 
dgp:	distancia grande positiva
dpp:	distancia pequena positiva
dz:		distancia zero(ou muito proxima de zero)
dpn:	distancia pequena negativa
dgn:	distancia grande negativa

velocidades para os motores podem ser
vgf: 	velocidade grande para frente
vmf: 	velocidade média para frente 
vp: 	velocidade pequena (próxima de zero)
vmt: 	velocidade media para traz 
vgt: 	velocidade grande para traz(não usada)

*/

#include <FuzzyInput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyComposition.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzySet.h>
#include <Fuzzy.h>
#include <AFMotor.h> // https://learn.adafruit.com/adafruit-motor-shield/library-install


//--------------------------------INF
#define INF 0xffffffff

//--------------------------------motor
typedef struct MotorsS {
  int pwmL;
  int pwmR;
} Motors;

Motors M;
AF_DCMotor motorEsq(1, MOTOR12_64KHZ);
AF_DCMotor motorDir(2, MOTOR12_64KHZ);

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


//--------------------------------Instancias Fuzzy para os motores
Fuzzy* fuzzyM2 = new Fuzzy();
Fuzzy* fuzzyM1 = new Fuzzy();

//--------------------------------Tempo de amostragem
#define T 5
unsigned long lt;

void setup() {
  M1();M2(); // Montam a lógica fuzzi de funcionamento dos motores
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
  Serial.begin(9600);
  lt = millis();
}

void loop() {
  if ((millis() - lt) > T) {
    lt = millis();

    centroid();
    control();
    
    motorEsq.setSpeed(255);
    motorDir.setSpeed(255);
    motorEsq.run(FORWARD);
    motorDir.run(FORWARD);
  }
}

//--------------------------------Leitura dos sensores
//valor máximo de leitura do sensor
#define READMAX 1024
unsigned int sensRead[8];


void readSens(){
    sensRead[0]=READMAX-analogRead(S1PIN);
    sensRead[1]=READMAX-analogRead(S2PIN);
    sensRead[2]=READMAX-analogRead(S3PIN);
    sensRead[3]=READMAX-analogRead(S4PIN);
    sensRead[4]=READMAX-analogRead(S5PIN);
    sensRead[5]=READMAX-analogRead(S6PIN);
    sensRead[6]=READMAX-analogRead(S7PIN);
    sensRead[7]=READMAX-analogRead(S8PIN);
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

//--------------------------------Control(consultas fuzzy)

void control() {
  fuzzyM1->setInput(1, err);
  fuzzyM2->setInput(1, err);

  fuzzyM1->fuzzify();
  fuzzyM2->fuzzify();

  M.pwmL = fuzzyM1->defuzzify(1);
  M.pwmR = fuzzyM2->defuzzify(1);
}


// -----------------------------configuracao fuzzy para os motores 
void M1(){  
  
  FuzzyInput* distancia = new FuzzyInput(1);

  FuzzySet* grande_neg = new FuzzySet(-0.0525, -0.035, -0.035, -0.0175); 
  distancia->addFuzzySet(grande_neg); 
  FuzzySet* pequena_neg = new FuzzySet(-0.035, -0.0175, -0.0175, -0.0); 
  distancia->addFuzzySet(pequena_neg); 
  FuzzySet* zero = new FuzzySet(-0.0175, 0.0, 0.0, 0.0175); 
  distancia->addFuzzySet(zero); 
  FuzzySet* pequena_pos = new FuzzySet(0.0, 0.0175, 0.0175, 0.035); 
  distancia->addFuzzySet(pequena_pos); 
  FuzzySet* grande_pos = new FuzzySet(0.0175, 0.035, 0.035, 0.0525); 
  distancia->addFuzzySet(grande_pos); 


  fuzzyM1->addFuzzyInput(distancia); 
  
  FuzzyOutput* velocidade = new FuzzyOutput(1);

  //255/2 = 127
  FuzzySet* velocidade_grande_traz = new FuzzySet(-385, -255, -255, -127); 
  velocidade->addFuzzySet(velocidade_grande_traz); 
  FuzzySet* velocidade_media_traz = new FuzzySet(-255, -127, -127, 0); 
  velocidade->addFuzzySet(velocidade_media_traz); 
  FuzzySet* velocidade_pequena = new FuzzySet(-127, 0.0, 0.0, 127); 
  velocidade->addFuzzySet(velocidade_pequena); 
  FuzzySet* velocidade_media_frente = new FuzzySet(0.0, 127, 127, 255); 
  velocidade->addFuzzySet(velocidade_media_frente); 
  FuzzySet* velocidade_grande_frente = new FuzzySet(127, 255, 255, 385); 
  velocidade->addFuzzySet(velocidade_grande_frente); 

  fuzzyM1->addFuzzyOutput(velocidade); 

  //Passo 4 - Montando as regras Fuzzy
  
  // distancia grande e positiva , 
    // * motor1 -> velocidade media para traz 
    //   motor2 -> velocidade grande para frente
  FuzzyRuleAntecedent* seDgrande_pos = new FuzzyRuleAntecedent();
  seDgrande_pos->joinSingle(grande_pos);
  FuzzyRuleConsequent* velocidade_mt = new FuzzyRuleConsequent();
  velocidade_mt->addOutput(velocidade_media_traz);
  FuzzyRule* fuzzyM1Rule01 = new FuzzyRule(1, seDgrande_pos, velocidade_mt);
  fuzzyM1->addFuzzyRule(fuzzyM1Rule01);

  // distancia pequena e positiva 
    // * motor1 -> velocidade pequena (próxima de zero)
    //   motor2 -> velocidade media para frente 
  FuzzyRuleAntecedent* seDpequeno_pos = new FuzzyRuleAntecedent();
  seDpequeno_pos->joinSingle(pequena_pos);
  FuzzyRuleConsequent* velocidade_p = new FuzzyRuleConsequent();
  velocidade_p->addOutput(velocidade_pequena);
  FuzzyRule* fuzzyM1Rule02 = new FuzzyRule(2, seDpequeno_pos, velocidade_p);
  fuzzyM1->addFuzzyRule(fuzzyM1Rule02);

  // distancia igual a zero , 
    // * motor1 -> v é grande para frente
    //   motor2 -> v é grande para frente
  FuzzyRuleAntecedent* seDzero = new FuzzyRuleAntecedent(); 
  seDzero->joinSingle(zero); 
  FuzzyRuleConsequent* velocidade_gf = new FuzzyRuleConsequent(); 
  velocidade_gf->addOutput(velocidade_grande_frente);
  FuzzyRule* fuzzyM1Rule03 = new FuzzyRule(3, seDzero, velocidade_gf); 
  fuzzyM1->addFuzzyRule(fuzzyM1Rule03); 

  //distancia pequena e negativa,  
    // * motor1 -> velocidade media para frente
    //   motor2 -> velocidade pequena(próxima de zero) 
  FuzzyRuleAntecedent* seDpequeno_neg = new FuzzyRuleAntecedent();
  seDpequeno_neg->joinSingle(pequena_neg);
  FuzzyRuleConsequent* velocidade_mf = new FuzzyRuleConsequent();
  velocidade_mf->addOutput(velocidade_media_frente);
  FuzzyRule* fuzzyM1Rule04 = new FuzzyRule(4, seDpequeno_neg, velocidade_mf);
  fuzzyM1->addFuzzyRule(fuzzyM1Rule04);

  //distancia grande e negativo ]
    // * motor1 -> velocidade grande para frente
    //   motor2 -> velocidade media para traz

  FuzzyRuleAntecedent* seDgrande_neg = new FuzzyRuleAntecedent();
  seDgrande_neg->joinSingle(grande_neg);
  FuzzyRuleConsequent* velocidade_gf2 = new FuzzyRuleConsequent();
  velocidade_gf2->addOutput(velocidade_grande_frente);
  FuzzyRule* fuzzyM1Rule05 = new FuzzyRule(5, seDgrande_neg, velocidade_gf2);
  fuzzyM1->addFuzzyRule(fuzzyM1Rule05);

}


void M2(){
  
  FuzzyInput* distancia = new FuzzyInput(1);
  
  FuzzySet* grande_neg = new FuzzySet(-0.0525, -0.035, -0.035, -0.0175); 
  distancia->addFuzzySet(grande_neg); 
  FuzzySet* pequena_neg = new FuzzySet(-0.035, -0.0175, -0.0175, -0.0); 
  distancia->addFuzzySet(pequena_neg); 
  FuzzySet* zero = new FuzzySet(-0.0175, 0.0, 0.0, 0.0175); 
  distancia->addFuzzySet(zero); 
  FuzzySet* pequena_pos = new FuzzySet(0.0, 0.0175, 0.0175, 0.035); 
  distancia->addFuzzySet(pequena_pos); 
  FuzzySet* grande_pos = new FuzzySet(0.0175, 0.035, 0.035, 0.0525); 
  distancia->addFuzzySet(grande_pos); 

  fuzzyM2->addFuzzyInput(distancia); 

  FuzzyOutput* velocidade = new FuzzyOutput(1);

  //255/2 = 127
  FuzzySet* velocidade_grande_traz = new FuzzySet(-385, -255, -255, -127); 
  velocidade->addFuzzySet(velocidade_grande_traz); 
  FuzzySet* velocidade_media_traz = new FuzzySet(-255, -127, -127, 0); 
  velocidade->addFuzzySet(velocidade_media_traz); 
  FuzzySet* velocidade_pequena = new FuzzySet(-127, 0.0, 0.0, 127); 
  velocidade->addFuzzySet(velocidade_pequena); 
  FuzzySet* velocidade_media_frente = new FuzzySet(0.0, 127, 127, 255); 
  velocidade->addFuzzySet(velocidade_media_frente); 
  FuzzySet* velocidade_grande_frente = new FuzzySet(127, 255, 255, 385); 
  velocidade->addFuzzySet(velocidade_grande_frente); 

  fuzzyM2->addFuzzyOutput(velocidade); 



  //Passo 4 - Montando as regras Fuzzy
  
  // distancia grande e positiva , 
    //   motor1 -> velocidade media para traz 
    // * motor2 -> velocidade grande para frente
  FuzzyRuleAntecedent* seDgrande_pos = new FuzzyRuleAntecedent();
  seDgrande_pos->joinSingle(grande_pos);
  FuzzyRuleConsequent* velocidade_gf = new FuzzyRuleConsequent();
  velocidade_gf->addOutput(velocidade_grande_frente);
  FuzzyRule* fuzzyM2Rule01 = new FuzzyRule(1, seDgrande_pos, velocidade_gf);
  fuzzyM2->addFuzzyRule(fuzzyM2Rule01);


  // distancia pequena e positiva 
    //   motor1 -> velocidade pequena (próxima de zero)
    // * motor2 -> velocidade media para frente 
  FuzzyRuleAntecedent* seDpequeno_pos = new FuzzyRuleAntecedent();
  seDpequeno_pos->joinSingle(pequena_pos);
  FuzzyRuleConsequent* velocidade_mf = new FuzzyRuleConsequent();
  velocidade_mf->addOutput(velocidade_media_traz);
  FuzzyRule* fuzzyM2Rule02 = new FuzzyRule(2, seDpequeno_pos, velocidade_mf);
  fuzzyM2->addFuzzyRule(fuzzyM2Rule02);


  // distancia igual a zero , 
    //   motor1 -> v é grande para frente
    // * motor2 -> v é grande para frente
  FuzzyRuleAntecedent* seDzero = new FuzzyRuleAntecedent(); 
  seDzero->joinSingle(zero); 
  FuzzyRuleConsequent* velocidade_gf2 = new FuzzyRuleConsequent(); 
  velocidade_gf2->addOutput(velocidade_grande_frente);
  FuzzyRule* fuzzyM2Rule03 = new FuzzyRule(3, seDzero, velocidade_gf2); 
  fuzzyM2->addFuzzyRule(fuzzyM2Rule03); 


  //distancia pequena e negativa,  
    //   motor1 -> velocidade media para frente
    // * motor2 -> velocidade pequena(próxima de zero) 
  FuzzyRuleAntecedent* seDpequeno_neg = new FuzzyRuleAntecedent();
  seDpequeno_neg->joinSingle(pequena_neg);
  FuzzyRuleConsequent* velocidade_p = new FuzzyRuleConsequent();
  velocidade_p->addOutput(velocidade_pequena);
  FuzzyRule* fuzzyM2Rule04 = new FuzzyRule(4, seDpequeno_neg, velocidade_p);
  fuzzyM2->addFuzzyRule(fuzzyM2Rule04);


  //distancia grande e negativo ]
    //   motor1 -> velocidade grande para frente
    // * motor2 -> velocidade media para traz

  FuzzyRuleAntecedent* seDgrande_neg = new FuzzyRuleAntecedent();
  seDgrande_neg->joinSingle(grande_neg);
  FuzzyRuleConsequent* velocidade_mt2 = new FuzzyRuleConsequent();
  velocidade_mt2->addOutput(velocidade_media_traz);
  FuzzyRule* fuzzyM2Rule05 = new FuzzyRule(5, seDgrande_neg, velocidade_mt2);
  fuzzyM2->addFuzzyRule(fuzzyM2Rule05);

}
