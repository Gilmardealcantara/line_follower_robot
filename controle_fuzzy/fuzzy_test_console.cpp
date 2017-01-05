//  g++ fuzzy_test_console.cpp eFLL/*.h eFLL/*.cpp

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

#include <iostream>
#include <string.h>

#include "./eFLL/FuzzyRule.h"
#include "./eFLL/FuzzyComposition.h"
#include "./eFLL/FuzzyRuleConsequent.h"
#include "./eFLL/FuzzyOutput.h"
#include "./eFLL/FuzzyInput.h"
#include "./eFLL/FuzzyIO.h"
#include "./eFLL/FuzzySet.h"
#include "./eFLL/FuzzyRuleAntecedent.h"
#include "./eFLL/Fuzzy.h"

using namespace std;

Fuzzy* fuzzyM2 = new Fuzzy();
Fuzzy* fuzzyM1 = new Fuzzy();



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

	//Passo 4 -Montando as regras Fuzzy
	
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



	//Passo 4 -Montando as regras Fuzzy
	
	// distancia grande e positiva , 
		// 	 motor1 -> velocidade media para traz 
		// * motor2 -> velocidade grande para frente
	FuzzyRuleAntecedent* seDgrande_pos = new FuzzyRuleAntecedent();
	seDgrande_pos->joinSingle(grande_pos);
	FuzzyRuleConsequent* velocidade_gf = new FuzzyRuleConsequent();
	velocidade_gf->addOutput(velocidade_grande_frente);
	FuzzyRule* fuzzyM2Rule01 = new FuzzyRule(1, seDgrande_pos, velocidade_gf);
	fuzzyM2->addFuzzyRule(fuzzyM2Rule01);


	// distancia pequena e positiva 
		// 	 motor1 -> velocidade pequena (próxima de zero)
		// * motor2 -> velocidade media para frente 
	FuzzyRuleAntecedent* seDpequeno_pos = new FuzzyRuleAntecedent();
	seDpequeno_pos->joinSingle(pequena_pos);
	FuzzyRuleConsequent* velocidade_mf2 = new FuzzyRuleConsequent();
	velocidade_mf2->addOutput(velocidade_media_frente);
	FuzzyRule* fuzzyM2Rule02 = new FuzzyRule(2, seDpequeno_pos, velocidade_mf2);
	fuzzyM2->addFuzzyRule(fuzzyM2Rule02);


	// distancia igual a zero , 
		// 	 motor1 -> v é grande para frente
		// * motor2 -> v é grande para frente
	FuzzyRuleAntecedent* seDzero = new FuzzyRuleAntecedent(); 
	seDzero->joinSingle(zero); 
	FuzzyRuleConsequent* velocidade_gf2 = new FuzzyRuleConsequent(); 
	velocidade_gf2->addOutput(velocidade_grande_frente);
	FuzzyRule* fuzzyM2Rule03 = new FuzzyRule(3, seDzero, velocidade_gf2); 
	fuzzyM2->addFuzzyRule(fuzzyM2Rule03); 


	//distancia pequena e negativa,  
		// 	 motor1 -> velocidade media para frente
		// * motor2 -> velocidade pequena(próxima de zero) 
	FuzzyRuleAntecedent* seDpequeno_neg = new FuzzyRuleAntecedent();
	seDpequeno_neg->joinSingle(pequena_neg);
	FuzzyRuleConsequent* velocidade_p = new FuzzyRuleConsequent();
	velocidade_p->addOutput(velocidade_pequena);
	FuzzyRule* fuzzyM2Rule04 = new FuzzyRule(4, seDpequeno_neg, velocidade_p);
	fuzzyM2->addFuzzyRule(fuzzyM2Rule04);


	//distancia grande e negativo ]
		// 	 motor1 -> velocidade grande para frente
		// * motor2 -> velocidade media para traz

	FuzzyRuleAntecedent* seDgrande_neg = new FuzzyRuleAntecedent();
	seDgrande_neg->joinSingle(grande_neg);
	FuzzyRuleConsequent* velocidade_mt2 = new FuzzyRuleConsequent();
	velocidade_mt2->addOutput(velocidade_media_traz);
	FuzzyRule* fuzzyM2Rule05 = new FuzzyRule(5, seDgrande_neg, velocidade_mt2);
	fuzzyM2->addFuzzyRule(fuzzyM2Rule05);

	
}

#define TAM 71
	
void print_scilab(float *v, string m){
	
	cout << "M"<< m <<" = [";;

	for(int i=0; i<TAM; i++){
		cout << v[i] << ", ";
	}
	cout << "]" << endl<<endl;

}

int main(){
	float dist;
	float err[71] = { -0.035, -0.034, -0.033, -0.032, -0.031, -0.03 , -0.029, -0.028, -0.027, -0.026, -0.025, -0.024, -0.023, -0.022, -0.021, -0.02 , -0.019, -0.018, -0.017, -0.016, -0.015, -0.014, -0.013, -0.012, -0.011, -0.01 , -0.009, -0.008, -0.007, -0.006, -0.005, -0.004, -0.003, -0.002, -0.001, 0.   , 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009, 0.01 , 0.011, 0.012, 0.013, 0.014, 0.015, 0.016, 0.017, 0.018, 0.019, 0.02 , 0.021, 0.022, 0.023, 0.024, 0.025, 0.026, 0.027, 0.028, 0.029, 0.03 , 0.031, 0.032, 0.033, 0.034, 0.035};
	float output1[71], output2[71];
	M1();M2();
	for(int i=0; i< TAM; i++){
		//cout << "Dist : ";
		//cin >> dist;
			dist = err[i];	
			fuzzyM1->setInput(1, dist);
			fuzzyM2->setInput(1, dist);
		
			fuzzyM1->fuzzify();
			fuzzyM2->fuzzify();
		
			output1[i] = fuzzyM1->defuzzify(1);
			output2[i] = fuzzyM2->defuzzify(1);
		//cout << "SaidaM1 : " <<output1[i] << "\tSaidaM2 : " <<output2[i] << endl<<endl;
	}
	print_scilab(output1, "1");
	print_scilab(output2, "2");
	return 1;

}
