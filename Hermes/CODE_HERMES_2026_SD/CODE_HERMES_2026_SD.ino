/* CODE HERMES 2026 course avec 3 lasers utilisés, avec frein */

#include "Servo.h"
#include <SD.h> 
// variables pour lecteur carte SD.
// Nom du fichier dans lequel écrire
const char filename[] = "Hermes_1.txt";
// Objet fichier pour représenter le fichier
File txtFile;
// chaîne pour mettre en mémoire tampon la sortie
String buffer;

#include <TFMPlus.h>
TFMPlus tfmP1;         // Creation objet TFMini Avant Gauche
TFMPlus tfmP2;        // Creation objet TFMini Avant Droit
TFMPlus tfmP3;        // Creation objet TFMini Frontal Gauche
TFMPlus tfmP4;        // Creation objet TFMini Frontal Droit
TFMPlus tfmP5;        // Creation objet TFMini Frontal Central
                  
String chaine = " ";                          // chaine de caractère contenant le message 
Servo myservo;                       // déclaration objet servo
const int pinTpntH=12;                // pin pont en H forward 
const int pinTpntHrev=11;             // pin pont en H reverse
const int pinServo=5;                 // pin servo direction  
const int pinOdo= 3;                   // pin odométrie
int angleDefault=100;                  // variable de commande servo direction à braquage nul
int angleBraq=angleDefault;           // variable commande braquage

bool idLA=false;          // flag laser a transmis données
bool idLB=false;
bool idLC=false;
bool idLD=false;
bool idL5=false;
unsigned long lastMsg = 0;
                    // variables odométrie 
unsigned long t;                                        // délai entre deux lecture du pignon pour odométrie
unsigned long nbPignon=0;                               // cumul du nbre de tours roue
unsigned long deltaNbPignon;
unsigned long lastNbPignon=0;                               // cumul du nbre de tour pignon
unsigned long deltaMicro;                               
unsigned long lastMicro;                               // pour calcul délai odométrie
float cumDist;                                          // distance parcourue cumulée en cm
//float cumDistInt;                                       // distance parcourue cumulée en cm calculée sur interruption
float nb_tour_sec;                                  // nbre de tour/s roue
//unsigned long curseTime = 90000;                        // Compteur de temps total course en millis (exempl 60 secondes = 60 000
unsigned long curseTime = 10000;                        // Compteur de temps total course en millis (exempl 60 secondes = 60 000
unsigned long lastCurseTime;                            // Temps début course  
                            // variables asservissement vitesse
float VIT;                                              // vitesse calculée en KM/H
float vcible;
float VitInt;                                           // vitesse calculée en KM/H par interrupt
int PwmVIT=0;                                 // Initialisation de la valeur du PWM pont en H
int lastVIT=0;
float kp=5.8;
float kd=2;
int pid;
float corrDebug;
int pwmMult=2.0;
int pwmconst=0;
unsigned long tPrec_v=0;
unsigned long tdeb=0;
float erreurPrec_v = 0.0;
float commandePWM_v=0.0;
float integrale_v=0.0;
float Kp_v = 8.0;
float Ki_v = 0.5;
float Kd_v = 3.0;
                          // variables utilisées pour le freinage
bool okFrein=false;                               // indicateur condition necessitant freinage initialisé à false
bool neutre = false;                              // indicateur état précédent commande pont en H 
int minDist=300;                                   // distance mini capteur frontal déclenchant le freinage
int PwmReverse=180;                                 // valeur du Pwm reverse
int PwmFrein = PwmReverse;
float consiTroncon =0.0;                          // consigne vitesse donnée par la table troncon
float consi=0.0;                                  // consigne vitesse effective
float consiVite =15.0;                               // consigne vitesse ligne droite
float consiLent =9.0;                               // consigne vitesse virages
long int t0frein =0;                              // compteur millis pour intervalle coup de frein
long int dfrein=5;                                // durée action freinage en ms.
bool debFrein = false;                            // indicateur première boucle de freinageconsi=consiVite;
float dstart=150.0;
float tprec=0.0;
float errPrec=0.0;
float tronCon[16][2] {         // tableau 16 tronçons : cm et km/h 
            {0,5},                        // 
            {50,10},                       // 
            {100,10},                      // 
            {500,10},                     // 6m: premier portique
            {3000,0},                     //  Charbonnière     
            {13031,0},                     // portique          
            {5462,0},                     // Portique  
            {5562,0},                     //
            {7893,0},                     // Portique
            {8093,0},                     //
            {10324,0},                    // Portique
            {10524,0},
            {12754,0},                    // Dernier Portique
            {13050,0},                    // fin de course
            {132000,0},                                                
            {999999,0}                    
 };
 int lgTroncon =(sizeof(tronCon)/sizeof(tronCon[0]))-1;                    
                      // Acquisitions mesures capteurs laser
int AVG;                                         //mesure du laser AV G
int AVD;          //mesure du laser AV droit
int FC;           //mesure du laser Frontal Centre                            
int FG;           //mesure du laser frontal gauche
int FD;           //mesure du laser frontal droite
int check;        //variable de contrôle checksum tampon données laser
int i;
const int HEADER=0x59;          //entête tampon données laser
                                // variables spécifiques au pilotage lasers droite
float l=19.5;                                      // distance en cm entre les deux capteurs lasers latéraux droite
float erreur;                                     // erreur totale (erreur alpha + erreur distance)
float lastErreur;
float alpha=0.0;                                  // angle alpha si deux lasers latéraux
//float consigneD=50.0;                             // consigne de suivi de la bordure à la distance D
float consigneAlpha=0.;                           // consigne de suivi de la bordure avec un angle Alpha (= 0 en ligne droite)
float lastAlpha=0.0;                              // erreur précédente pour calculer le delta alpha
float lastDist=0.0;                               // erreur précédente pour calculer le delta distance
                                //Coefficients pilotage lasers latéraux
float K1 = 0.5;                                    // coefficient erreur alpha
float K2 = 0.7;                                   // coefficient erreur distance
float KD1 = 0.0;                                  // coefficient delta erreur alpha
float KD2 = 0.0;                                  // coefficient delta erreur distance
                              // compteurs de temps
long int t0;                                      // valeur micros() en début de boucle
long int t1;                                     
long int t2;
long int tt0;                                     // valeur micros() en début acquisition données laser
long int tt1;                                     // durée d'une acquisition laser
long int topservo;
int freq = 0;

void setup() {
  Serial.begin(115200);
                              //initialisation vitesse port série entre esp32 et pc
  //initSerialTfminiPlus();                           // Initialisation des ports série et des objets tfminiPlus
  //softResetTfmini();                                // Reset des tfminiPlus
  //frameRateTfminiPlus();                            // paramétrage de la fréquence d'acquisition des tfminiPlus
                              // initialisation des pins
  myservo.attach(pinServo);                         // déclaration objet myservo
  pinMode(pinTpntH,OUTPUT);                         // initialisation pin forward pont en H
  pinMode(pinTpntHrev,OUTPUT);                      // initialisation pin reverse pont en H
  pinMode(pinOdo,INPUT);                // initialisation pin signal hall
  attachInterrupt(pinOdo,countInterrupt,FALLING);  // appel interruption pour mesurer le nombre de tours
  movServo();                                     // mise à la position neutre du servo de direction
  delay(6000);
  SdSetup();                                        // Initialisation carte SD
}

void loop() {
  t2=t1;                     // chargement durée début loop 
  t1=micros();

  //litLaser('A');                   // appel lecture laser lateral G: AVG
  //litLaser('B');                   // appel lecture laser lateral D: AVD
  //litLaser('C');                 // appel lecture laser frontal G:FG
  //litLaser('D');                 // appel lecture laser frontal D:FD
  //litLaser('E');                   // appel lecture laser frontal C:FC
compteur();
ouEstil();
if(FC<dstart){
 needFrein();
}
else{
 asservVITPID(VIT,consi);
 motor();
}
//debug();     
Chargebuffer();                                 // appel fonction de chargement du buffer pour écriture sur SD
Serial.println("enregistrement en cours");
delay(5000);
}

void Erreur(){
erreur= 1.*(AVG-AVD);
angleBraq=(-1.07*erreur)+100.; 
//angleBraq=100.;
}

void movServo(){
  angleBraq=constrain(angleBraq,70,130);
  myservo.write(angleBraq);
}

void compteur(){
deltaNbPignon = nbPignon - lastNbPignon;
if(deltaNbPignon >0){
deltaMicro = micros() - lastMicro;
lastNbPignon = nbPignon;
lastMicro = micros();
nb_tour_sec= deltaNbPignon/(deltaMicro/1000000.0)/2.;
VIT = (nb_tour_sec*3600)*(3.1416*7.2/100000);
if(VIT <= 1.0){
  VIT = 1.0;
}

cumDist = (nbPignon)*3.1416*7.2;
}
}

void ouEstil() {
  cumDist = (nbPignon)*3.1416*7.2/2;      // en cm, diam roue 7.2, 2 aimants,calcul cumul distance 
  for (int i=0; i <= 8;i++){ 

    if(cumDist >= tronCon[i][0] and cumDist < tronCon[i+1][0]){
      consi = tronCon[i][1];
    }
  }
}

void needFrein(){
unsigned long tdeb= millis();
float vmin=6.0;
float dstart=150.0;
float dend=60.0;
vcible = max(vmin + (consi-vmin)*((FC-dend)/(dstart-dend)), vmin);
consi = vcible;
float err= vcible-VIT;
unsigned long tnow=millis();
float dt=(tnow-tprec)/1000.0;
float deriv=(err-errPrec)/dt;
float P=10*err;
float D=0.5*deriv;
pid=P+D;
errPrec=err;
tprec=tnow;
pid=constrain(pid,-200,200);
PwmVIT=pid;
if(pid>0)
{
  analogWrite(pinTpntH,pid);
  analogWrite(pinTpntHrev,0);
}
else
{
  analogWrite (pinTpntH,0);
  analogWrite(pinTpntHrev,-pid);
}
}


void asservVITPID(float VIT, float vitesseConsigne) {
                        // Temps écoulé
  unsigned long tNow = millis();
  float dt = (tNow - tPrec_v) / 1000.0;  // secondes
  if (dt < 0.001) dt = 0.001;
                      // Erreur instantanée
  float err = vitesseConsigne - VIT;
                      // PID
  integrale_v += err * dt;
  float derivee = (err - erreurPrec_v) / dt;
  float P = Kp_v * err;
  float I = Ki_v * integrale_v;
  float D = Kd_v * derivee;
  float sortie = P + I + D;
                      // Mémoires
  erreurPrec_v = err;
  tPrec_v = tNow;
                      // Saturation PWM
  sortie = constrain(sortie, -255, 255);
                  // Lissage possible pour éviter à-coups
  float alpha = 0.7;
  commandePWM_v = alpha * commandePWM_v + (1 - alpha) * sortie;
  if(consi>0){
  PwmVIT = commandePWM_v;
  }else {
  PwmVIT = 0;
  }
}

void motor(){
  PwmVIT = constrain(PwmVIT, 0,80);
  analogWrite(pinTpntH,PwmVIT);
 }

void countInterrupt(){                // fonction appelée par l'attach interrupt
nbPignon++;
}


void initSerialTfminiPlus(){      // Initialisation des ports série et des objets tfminiPlus
  Serial1.begin(115200); 
  delay(20);                  // délais pour initialisation du port
    tfmP1.begin( &Serial1);   // initialisation de l'objet tfminiPlus et affectation du port série              
  Serial2.begin(115200);
  delay(20);                  // délais pour initialisation du port
    tfmP2.begin( &Serial2);   // initialisation de l'objet tfminiPlus et affectation du port série
  Serial3.begin(115200);
  delay(20);                  // délais pour initialisation du port
    tfmP3.begin( &Serial3);   // initialisation de l'objet tfminiPlus et affectation du port série   
                              // initialisation port série virtuel et contrôle
  Serial4.begin(115200);
  delay(20);               // délais pour initialisation du port
  tfmP4.begin(&Serial4);   // initialisation de l'objet tfminiPlus et affectation du port série   
  Serial5.begin(115200);
  delay(20);               // délais pour initialisation du port
  tfmP5.begin(&Serial5);   // initialisation de l'objet tfminiPlus et affectation du port série   
}

void softResetTfmini(){   // Reset des tfminiPlus
    Serial.println( "Soft reset Capteur A (avant gauche): ");
    if( tfmP1.sendCommand( SOFT_RESET, 0))
    {
        Serial.println( "reset ok\r\n");
    }
    else tfmP1.printReply();  
    delay(500);  // delai d'attente pour que le rest soit complet


    Serial.println( "Soft reset Capteur B (avant droit): ");
    if( tfmP2.sendCommand( SOFT_RESET, 0))
    {
      Serial.println( "reset ok\r\n");
    }
    else tfmP2.printReply();
    delay(500);   // delai d'attente pour que le rest soit complet


    Serial.println( "Soft reset Capteur C (Frontal Gauche): ");
    if( tfmP3.sendCommand( SOFT_RESET, 0))
    {
        Serial.println( "reset ok\r\n");
    }
    else tfmP3.printReply();
    delay(500);  // delai d'attente pour que le rest soit complet

    Serial.println( "Soft reset Capteur D (Frontal Droit): ");
    if( tfmP4.sendCommand( SOFT_RESET, 0))
    {
        Serial.println( "reset ok\r\n");
    }
    else tfmP4.printReply();
    delay(500);  
    Serial.println( "Soft reset Capteur E (Frontal Centre): ");
    if( tfmP5.sendCommand( SOFT_RESET, 0))
    {
      Serial.println( "reset ok\r\n");
    }
    else tfmP5.printReply();
    delay(500);  // delai d'attente pour que le rest soit complet
}

void frameRateTfminiPlus(){   // paramétrage de la fréquence d'acquisition des tfminiPlus
                              // - - initialize la fréquence du laser avant à 250Hz - - - - - - - -
    Serial.println( "Data-Frame rate: ");
    if( tfmP1.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
      Serial.print( "frame AV "); Serial.println(FRAME_250);
    }
    else tfmP1.printReply();
    delay(500);
    Serial.println( "Data-Frame rate: ");
    if( tfmP2.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
      Serial.print( "frame AR "); Serial.println(FRAME_250);
    }
    else tfmP2.printReply();
    delay(500);   

    Serial.println( "Data-Frame rate: ");
    if( tfmP3.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
      Serial.print( "frame FG "); Serial.println(FRAME_250);
    }
    else tfmP3.printReply();
    delay(500);   
    Serial.println( "Data-Frame rate: ");
    if( tfmP4.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
      Serial.print( "frame FD "); Serial.println(FRAME_250);
    }
    else tfmP4.printReply();
    delay(500);   
    Serial.println( "Data-Frame rate: ");
    if( tfmP5.sendCommand( SET_FRAME_RATE, FRAME_250))
    {
      Serial.print( "frame FC "); Serial.println(FRAME_250);
    }
    else tfmP5.printReply();
    delay(500);   
}                               

void mesureFreqLaser(char ID){
    tt1=micros()-tt0;
    //Serial.print(" FA : ");Serial.print(ID);Serial.print(" ");Serial.println(1000000/tt1);
    tt1=0;
    tt0=micros();
    if(ID=='A'){
      idLA = true;
    }
    if(ID=='B'){
      idLB = true;
    }
    if(ID=='C'){
      idLC = true;
    }    
    if(ID=='D'){
      idLD = true;
    }
    if(ID=='E'){
      idL5 = true;
    }    
}

void litLaser(char car){     // Lecture des lasers
    int16_t tfDist = 0;    // Distance en cm
    int16_t tfFlux = 0;    // qualité du signal (force)
    int16_t tfTemp = 0;    // Température interne du laser
  if(car=='A'){ 
      if( tfmP1.getData( tfDist, tfFlux, tfTemp)) // obtenir les données du laser avant
      {
        mesureFreqLaser('A');
         if (tfDist >0)
        {
          AVG=tfDist;
        }
      }
      else                  // If the command fails...
      {
       //Serial.print(" err A "); 
      //tfmP.printFrame();  // display the error and HEX dataa
    }
  }
    if(car=='B'){
      if( tfmP2.getData( tfDist, tfFlux, tfTemp))  // obtenir les données du laser arriere
      {
        mesureFreqLaser('B');
        if (tfDist >0)
        {
          AVD=tfDist;
        }
      }
      else                  // If the command fails...
      {
       //Serial.print(" err B "); 
      //tfmP2.printFrame();  // display the error and HEX dataa
    }
  }
     if(car=='C'){
      if( tfmP3.getData( tfDist, tfFlux, tfTemp)) // obtenir les données du laser frontal
      {
        mesureFreqLaser('D');                                 // appel fonction permettant d'estimer la fréquence d'acquisition d'un laser
        if (tfDist >0)
        {
            FG=tfDist;
        }
      }
      else                  // If the command fails...
      {
        //Serial.print(" err C "); 
      //tfmP3.printFrame();  // display the error and HEX dataa
    }
  } 
    if(car=='D'){
      if( tfmP4.getData( tfDist, tfFlux, tfTemp)) // obtenir les données du laser frontal
      {
        mesureFreqLaser('D');                                 // appel fonction permettant d'estimer la fréquence d'acquisition d'un laser
        if (tfDist >0)
        {
            FD=tfDist;
        }
      }
      else                  // If the command fails...
      {
        //Serial.print(" err C "); 
      //tfmP3.printFrame();  // display the error and HEX dataa
    }
  } 
    if(car=='E'){
      if( tfmP5.getData( tfDist, tfFlux, tfTemp)) // obtenir les données du laser frontal
      {
        mesureFreqLaser('E');                                 // appel fonction permettant d'estimer la fréquence d'acquisition d'un laser
        if (tfDist >0)
        {
          FC=tfDist;
        }
      }
  }
}




void SdSetup(){               // fonction initialisation carte SD
  delay(1000);
  Serial.println("configuration du SD ...");
  // initialisation de la carte SD
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
  }
  // supprimer le fichier s'il existe
  if (SD.exists(filename)){
      SD.remove(filename);        // pour conserver les informations collectées lors du précédent lancement, il faut commenter cette ligne
  }
   // ⭐ OUVRIR LE FICHIER EN MODE ÉCRITURE
  txtFile = SD.open(filename, FILE_WRITE);
  
  // Vérifier que le fichier est bien ouvert
  if (!txtFile) {
    Serial.println("Erreur : impossible d'ouvrir le fichier !");
    while(1);  // bloquer si impossible d'ouvrir
  }
  Serial.println("Fichier créé et ouvert !");
  // Ajout ligne entête fichier
  txtFile.println();
  txtFile.println("Durée;Distance;PWM;CONSIGNE;VITESSE;laser_FC");
  txtFile.flush();
  }

  void Chargebuffer(){                // Chargement des donénes dans le buffer pour écriture sur carte SD

    // ajout d'une nouvelle ligne dans le buffer
    buffer = "";
    buffer += millis();
    /*buffer += ";";
    buffer += cumDist;
    buffer += ";";
    buffer += PwmVIT;
    buffer += ";";
    buffer += consi;
    buffer += ";";
    buffer += VIT;
    buffer += ";";
    buffer += FC;
    */
    buffer += ";";
    //buffer += "\r\n";
  
    // Écrire directement sans buffer
    txtFile.println(buffer);
    txtFile.flush();
    
    Serial.println("Ligne écrite: " + buffer);
  
  }


