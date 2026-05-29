/* CODE HERMES 2026 course avec 3 lasers utilisés, avec frein et SD et lissage du capteur FC */

/****** initialisation SD ******/
#include <SD.h> 
// Nom du fichier dans lequel écrire
const char filename[] = "TRR_1.csv";
// Objet fichier pour représenter le fichier
File txtFile;
// chaîne pour mettre en mémoire tampon la sortie
String buffer;
long int tbuffer = 0;


/****** initialisation TFMiniPlus ******/
#include <TFMPlus.h>
TFMPlus tfmP1;        // Creation objet TFMini Avant Gauche
TFMPlus tfmP2;        // Creation objet TFMini Avant Droit
TFMPlus tfmP3;        // Creation objet TFMini Frontal Gauche
TFMPlus tfmP4;        // Creation objet TFMini Frontal Droit
TFMPlus tfmP5;        // Creation objet TFMini Frontal Central
                  
bool idLA=false;      // flag laser a transmis données
bool idLB=false;
bool idLC=false;
bool idLD=false;
bool idL5=false;
                      // Acquisitions mesures capteurs laser
int AVG;              //mesure du laser AV G
int AVD;              //mesure du laser AV droit
int FC;               //mesure du laser Frontal Centre                            
int FG;               //mesure du laser frontal gauche
int FD;               //mesure du laser frontal droite

#include "Servo.h"
Servo myservo;               // déclaration objet servo
const int pinServo=5;        // pin servo direction
int angleBraq=101;           // variable commande braquage

const int pinTpntH=12;       // pin pont en H forward 
const int pinTpntHrev=11;    // pin pont en H reverse
                   
const int pinOdo= 3;         // pin odométrie
long int HallMil = 1;        // temps exact au passage devant le capteur 

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



/****** initialisation etat de course ******/
enum RaceState { IDLE, D1, U2, V3, V4, V5, D6, U7, FINISHED };
RaceState etat_courant = D1;
String etat_actuel = "D1";
float odo_debut_virage;


float VIT;                                              // vitesse calculée en KM/H
float vcible;
int PwmVIT=0;                                 // Initialisation de la valeur du PWM pont en H
float FCliss=300;

long int tstart;    // declenchement du chronometre
float dstart=65.0;
float dend=60.0;

float kp=5.8;
float kd=2;
int pid;
int pwmMult=2.0;
int pwmconst=0;
unsigned long tPrec_v=0;
unsigned long tdeb=0;
float erreurPrec_v = 0.0;
float commandePWM_v=0.0;
float integrale_v=0.0;


float tprec=0.0;
float errPrec=0.0;
float erreur = 0.0;
 
/****** calcul de la fréquence de la loop ******/   
long int t1;                                            
long int t2;
int freq = 0;

void setup() {
  Serial.begin(115200);
  initSerialTfminiPlus();                         // Initialisation des ports série et des objets tfminiPlus
  softResetTfmini();                              // Reset des tfminiPlus
  frameRateTfminiPlus();                          // paramétrage de la fréquence d'acquisition des tfminiPlus
  myservo.attach(pinServo);                         // déclaration objet myservo
  pinMode(pinTpntH,OUTPUT);                         // initialisation pin forward pont en H
  pinMode(pinTpntHrev,OUTPUT);                      // initialisation pin reverse pont en H
  pinMode(pinOdo,INPUT);                            // initialisation pin signal hall
  attachInterrupt(pinOdo,countInterrupt,FALLING);   // appel interruption pour mesurer le nombre de tours
  myservo.write(angleBraq);

  delay(3000);
  SdSetup();                                        // Initialisation carte SD
  Serial.print("fin setup"); 

  etat_courant = D1;                                     
  tstart = millis(); 
}


void loop() {
  t2=t1;                     // chargement durée début loop 
  t1=micros();
  litLaser('A');                   // appel lecture laser lateral G: AVG
  litLaser('B');                   // appel lecture laser lateral D: AVD
  //litLaser('C');                 // appel lecture laser frontal G:FG
  //litLaser('D');                 // appel lecture laser frontal D:FD
  litLaser('E');                   // appel lecture laser frontal C:FC

  etat_actuel = ouestil();


  Erreur();

  if (etat_actuel == "D1") {
    Serial.println("⚡ D1");
    angleBraq=constrain(angleBraq,85,115);   // Remettre les parametres de course
  }
  else if (etat_actuel == "U2") {
    Serial.println("🔄 U2");
    angleBraq=constrain(angleBraq,70,130);  // a modifier, a ameliorer!
  }
  else if (etat_actuel == "V3") {
    angleBraq=constrain(angleBraq,65,106);  // a modifier, a ameliorer!
    Serial.println("➡️ V3");
  }
  else if (etat_actuel == "V4") {
    angleBraq=constrain(angleBraq,95,130);  // a modifier, a ameliorer!
    Serial.println("➡️ V4");
  }
  else if (etat_actuel == "V5") {
    Serial.println("➡️ V5");
    angleBraq=constrain(angleBraq,65,106);  // a modifier, a ameliorer!
  }
  /* else if (etat_actuel == "D6") {
    Serial.println("⚡ D6");
    // Code D6
  }
  */
  else if (etat_actuel == "U7") {
    Serial.println("🏁 U7");
    angleBraq=constrain(angleBraq,65,130);  // a modifier, a ameliorer!
  }
  myservo.write(angleBraq);

  comptVitDist();

// demarrage en vitesse réduite; on a experimenté qu'on peut aller direct a Vmax
// if (cumDist<200) vcible = 6;
// freinage a partir d'une distance parcourue !!!!!!!!!


if (cumDist>1100 || millis() - tstart > 30000){               // arret automatique du hermes
  while(1){                      // boucle infinie
      analogWrite(pinTpntH,0);   // freinage
      analogWrite(pinTpntHrev,0);
      delay(1000);
  }
}
// calcul en nominal
else computeSpeed(FC);

if(FC<dstart){                   // cas dans le virage
 needFrein(vcible);
}
else{
 asservVITPID(VIT,vcible);       // cas dans la ligne droite
}

debug();   
Chargebuffer();                  // appel fonction de chargement du buffer pour écriture sur SD
}

void Erreur(){
  if (FC>120){                                // parametres de course
    erreur= 0.2*(AVG-AVD);
  }
  else{
    erreur= 0.6*(AVG-AVD);
  }
  angleBraq=(-1.07*erreur)+101.;   
  
}


void countInterrupt(){                // fonction appelée par l'attach interrupt
  nbPignon++;
  HallMil = millis();
}

void comptVitDist(){
  deltaNbPignon = nbPignon - lastNbPignon;
  if(deltaNbPignon >0){
    deltaMicro = micros() - lastMicro;
    lastNbPignon = nbPignon;
    lastMicro = micros();
    nb_tour_sec= deltaNbPignon/(deltaMicro/1000000.0)/2.;
    VIT = (nb_tour_sec*3600)*(3.1416*7.2/100000);
  }
  
  //if (lastMicro - HallMil > 2000000) VIT =0;  // arrete la vitesse si trop longtemps sans mesure
  
  cumDist = (nbPignon)*3.1416*7.2/2;      // en cm, diam roue 7.2, 2 aimants,calcul cumul distance 

}

void computeSpeed(float dist){
  float vmax=10.0;
  float vmin=4.0;
  float dstart=150.0;
  float dend=60.0;
  if(dist>dstart)
  {
    vcible = vmax;
  }
  else if(dist<dend)
  {
    vcible = vmin;
  }
  else {
  float ratio= (FC-dend)/(dstart-dend);
  vcible = vmin+(vmax-vmin)*ratio;
  }
}


void needFrein(float vcible){
  unsigned long tdeb= millis();
  float vmin=6.0;
  float err= vcible-VIT;
  unsigned long tnow=millis();
  float dt=(tnow-tprec)/1000.0;
  float deriv=(err-errPrec)/dt;
  float P=40*err;
  float D=1*deriv;
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
    analogWrite(pinTpntHrev,120);
  }
}


void asservVITPID(float VIT, float vcible) {

float Kp_v = 15.0;
float Ki_v = 0.5; // attention
float Kd_v = 4;

                        // Temps écoulé
  unsigned long tNow = millis();
  float dt = (tNow - tPrec_v) / 1000.0;  // secondes
  if (dt < 0.001) dt = 0.001;
                      // Erreur instantanée
  float err = vcible - VIT;
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
  PwmVIT = constrain(sortie, -255, 255);
                  // Lissage possible pour éviter à-coups
  float alpha = 0.7;
  commandePWM_v = alpha * commandePWM_v + (1 - alpha) * PwmVIT;

  PwmVIT = constrain(commandePWM_v, 0,100);  
  
  analogWrite(pinTpntH,PwmVIT);
}

String ouestil() {
    // PARTIE 1: Vérifier transitions (if/else)
  if (etat_courant == D1) {
    if (FC < 110) {
      etat_courant = U2;
      odo_debut_virage = cumDist;
    }
  }

  else if (etat_courant == U2) {
    if( cumDist - odo_debut_virage > 300 && FC > 170){
      etat_courant = V3;
      odo_debut_virage = cumDist;
    }
  }

  else if (etat_courant == V3) {
    if( cumDist - odo_debut_virage > 200 && FC > 170){
      etat_courant = V4;
      odo_debut_virage = cumDist;
    }
  }
  else if (etat_courant == V4) {
    if( cumDist - odo_debut_virage > 200 && FC > 170){
      etat_courant = V5;
      odo_debut_virage = cumDist;
    }
  }
  else if (etat_courant == V5) {
    if( cumDist - odo_debut_virage > 200 && FC > 170 ){
      etat_courant = U7;
      odo_debut_virage = cumDist;
    }
  }
  /*
  else if (etat_courant == D6) {
    if( cumDist - odo_debut_virage > 50 && FC > 170) {
      etat_courant = U7;
      odo_debut_virage = cumDist;
    }
  }
  */
  else if (etat_courant == U7) {
    if(cumDist - odo_debut_virage > 100 && FC > 170){
      etat_courant = D1;
      odo_debut_virage = cumDist;
    }
  }

  if (etat_courant == D1) return "D1";
  else if (etat_courant == U2) return "U2";
  else if (etat_courant == V3) return "V3";
  else if (etat_courant == V4) return "V4";
  else if (etat_courant == V5) return "V5";
  // else if (etat_courant == D6) return "D6";
  else if (etat_courant == U7) return "U7";
  else if (etat_courant == IDLE) return "IDLE";        
  else if (etat_courant == FINISHED) return "FINISHED";
}

/**********************    DEBUG    **********************/  

void debug(){
  Serial.print(" A ");
  Serial.print(AVG);                             //affichage distance mesuré par le laser avant
  Serial.print(" B ");
  Serial.print(AVD);                             //affichage distance mesuré par le laser arrière
  Serial.print(" C ");
  Serial.print(FG);                             //affichage distance mesuré par le laser frontal gauche
  Serial.print(" D ");
  Serial.print(FD);                             //affichage distance mesuré par le laser frontal droite 
  Serial.print( " E ");
  Serial.println(FC);                             //affichage distance mesuré par le laser frontal centre,
  Serial.print("   Braquage °  ");Serial.print(angleBraq);
  Serial.print("   Dist parcourue cm  ");Serial.print(cumDist);
  //Serial.print("   PwmVIT asservi  ");Serial.print(PwmVIT);
  Serial.println("   VIT compteur ");Serial.print(VIT );
  Serial.println("   Vcible ");Serial.print(vcible); 
  freq = 1000000/(t2-t1);
  Serial.print("   F loop  ");Serial.println(freq);       // fréquence d'exécution de la loop
 }


/**********************    PARTIE TFMiniPlus    **********************/ 

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
    // compteurs de temps
    long int tt0;                                     // valeur micros() en début acquisition données laser
    long int tt1;                                     // durée d'une acquisition laser

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
          FCliss = 0.90 * FCliss + 0.1 * FC;   // fait un lissage de FC 
        }
      }
  }
}


/**********************    PARTIE Carte SD    **********************/ 

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
  txtFile.println("Durée;Distance;PWM;Vit Cible;VITESSE;laser_FC");
  txtFile.flush();
  }

  void Chargebuffer(){                // Chargement des donénes dans le buffer pour écriture sur carte SD
    if( millis() - tbuffer > 100) {
      // ajout d'une nouvelle ligne dans le buffer
      buffer = "";
      buffer += (millis() - tstart)/1000.0;
      buffer += ";";
      buffer += cumDist;
      buffer += ";";
      buffer += PwmVIT;
      buffer += ";";
      buffer += vcible;
      buffer += ";";
      buffer += VIT;
      buffer += ";";
      buffer += FC;
      buffer += ";";
      buffer += etat_actuel;
      
      buffer += ";";
      //buffer += "\r\n";
    
      // Écrire directement sans buffer
      txtFile.println(buffer);
      txtFile.flush();
      
      Serial.println("Ligne écrite: " + buffer);
      tbuffer= millis();
    }
  
  }


