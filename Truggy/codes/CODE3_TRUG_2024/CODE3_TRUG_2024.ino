/* CODE3 TRUGGY TRR 2024;détect obstacle et frein, reprise VIT max, cde VIT par PWM*/


#include <Servo.h>                  // chargement librairie servo
Servo myServo;                      // création objet servo
Servo myEsc;                        // création objet ESC
const int pinEsc = 9;             // Pin PWM commande ESC
const int pinServo = 8;             // Pin PWM commande servo direction
const int pinHall = A3;             // reçoit impulsions du capteur HALL POUR odométrie
int angleDefault = 87;              // variable commande servo direction à braquage nul
int angleBraq = angleDefault;       // variable commande braquage initialisée à braquage nul
int val;
int lastval;
unsigned long deltat;               // variable: durée en microsecondes entre 2 pulses pour mesurer nbre de tours/s
unsigned long lastmicros = 0;         // variable:  temps pulse précédent
unsigned long pulse_count = 0;
float nb_pulses_sec_hall;           // nbre de pulses ou sauts bas par seconde
float VITROThall;                   // vitesse rotation roue t/mn mesurée par HALL
float cumDist;                      // Cumul distance parcourue en cm
float diamRoue = 10.6;              // diamètre de la roue en cm
float VIT;                          // vitesse en km/h
int PwmVIT = 0;                     // commande pwm
float consi = 0.0;                  // vitesse cible en fonction du tronçon
float kp = 4.;                      // coefficient proportionnel asservissement vitesse
float tronCon[9][2] {               // tableau 9 Tronçons avec commandes distance en cm et vitesses cibles en km/h
  {0, 1590},                 // Tronçon 1 départ à distance 0 et commande VIT avec PWM équivalent à 2 km/h
  {50, 1615},
  {100,1630 },
  {200, 1640},
  {1000, 1500},              // 1 tour Tiny fait 24,3 m + 5,7 m de marge avant commande vitesse 0
  {15000, 0},            // 5 tours de course font (5x24,3)+7=128,5 m =10m de marge
  {20000, 0},
  {50000, 0},
  {99999, 0},
};
int dbordure = 0;
int d2;                                          //mesure distance en cm du laser avant.
int strengthAv;                                  //Force du signal laser de gauche ou laser avant
int d1;                                          //mesure  distance en cm du laser arrière
int strengthAr;                                  //Force du signal laser de droite ou laser arrière
int d3;
int strengthFront;
int check;                                       //variable de contrôle checksum tampon données laser
int i;
int uart[9];                                      //variable tableau stockage tampon données laser
const int HEADER = 0x59;                          //entête tampon données laser
bool TESTF = 1;
float l = 34;                                    // distance en cm entre les deux capteurs lasers latéraux du Truggy
float erreurs;                                     // somme des erreurs alpha + distance)
float alpha = 0.0;                                // angle alpha si deux lasers latéraux
float consigneD = 50.0;                          // consigne de suivi de la bordure à la distance D= ... cm
float consigneAlpha = 0.;                         // consigne de suivi de la bordure avec un angle Alpha = 0 en ligne droite
float K1 = 0.7;                                   // coefficient erreur alpha
float K2 = 0.7;                                  // coefficient erreur distance
long int t0;                                      // variables mesure durée de traitement boucle loop
long int t1;


void setup() {
  Serial.begin(115200);                             //initialisation vitesse port série entre arduino mega et pc
  Serial1.begin(115200);                            //initialisation vitesse port série entre arduino mega et 1° laser
  Serial2.begin(115200);                            //initialisation vitesse port série entre arduino mega et 2° laser
  Serial3.begin(115200);                            //initialisation vitesse port série entre arduino mega et 3° laser
  pinMode(pinHall, INPUT);                          // initialisation pin campteur Hall odométrie
  pinMode(pinServo, OUTPUT);                        // initialisation pin Servo
  myServo.attach(pinServo);                         // initialisation servo
  myEsc.attach(9);                                  // initialisation ESC
  myServo.write(87);
  myEsc.writeMicroseconds(1500);
  //SdSetup();                                        // Initialisation carte SD
  delay(3000);                                      // délai avant démarrage en ms
  t0 = millis();
  //consi=consiVite;
}


void loop() {
litLaserAR();                                   // appel fonction lecture laser arrière
litLaserAV();                                   // appel fonction lecture laser avant
litLaserFRONT();                                // appel fonction lecture laser frontal distance obstacle
Erreurs();                                       // appel fonction calcul erreur de trajectoire
movServo();                                     // appel fonction commande servo de direction
compteur();                                     // appel fonction pour calculer la vitesse km/h
ouEstil();                                      // appel fonction pour savoir quel tronçon et donc quelle vitesse cible consi km/h
motor();                                // appel fonction commande ESC/moteur
// debug();
}


void litLaserAV() {
  if (Serial1.available()) {                            //contrôle si data en entrées sur port série
    if (Serial1.read() == HEADER) {                   //contrôler la valeur de l'entête du paquet de données présente dans l'octet 0
      uart[0] = HEADER;                               // charger cette valeur dans l'élément 0 du tableau
      if (Serial1.read() == HEADER) {                 //contrôler la valeur de l'entête du paquet de données présente dans l'octet 1
        uart[1] = HEADER;                             // charger cette valeur dans l'élément 1 du tableau
        for (i = 2; i < 9; i++) {                     // boucle de chargement des données des octets suivants dans le tableau
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {                  //contrôle de l'intégrité des données : le dernier octet fde la somme des éléménts 0 à 7
          // du tableau doit être égale à la valeur de l'éléments 8.
          d2 = uart[2] + uart[3] * 256;                   //calcul de la distance en cm mesurée par le laser
          strengthAv = uart[4] + uart[5] * 256;           //calcul de la force du signal
        }
      }
    }
  }
}


void litLaserAR() {
  if (Serial2.available()) {                          //contrôle si data en entrées sur port série

    if (Serial2.read() == HEADER) {                   //contrôler la valeur de l'entête du paquet de données présente dans l'octet 0
      uart[0] = HEADER;                               // charger cette valeur dans l'élément 0 du tableau
      if (Serial2.read() == HEADER) {                 //contrôler la valeur de l'entête du paquet de données présente dans l'octet 1
        uart[1] = HEADER;                             // charger cette valeur dans l'élément 1 du tableau
        for (i = 2; i < 9; i++) {                     // boucle de chargement des données des octets suivants dans le tableau
          uart[i] = Serial2.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {                  //contrôle de l'intégrité des données : le dernier octet fde la somme des éléménts 0 à 7
          // du tableau doit être égale à la valeur de l'éléments 8.
          d1 = uart[2] + uart[3] * 256;                   //calcul de la distance mesurée en cm par le laser
          strengthAr = uart[4] + uart[5] * 256;            //calcul de la force du signal
        }
      }
    }
  }
}


void litLaserFRONT() {
  if (Serial3.available()) {                            //contrôle si data en entrées sur port série
    if (Serial3.read() == HEADER) {                   //contrôler la valeur de l'entête du paquet de données présente dans l'octet 0
      uart[0] = HEADER;                               // charger cette valeur dans l'élément 0 du tableau
      if (Serial3.read() == HEADER) {                 //contrôler la valeur de l'entête du paquet de données présente dans l'octet 1
        uart[1] = HEADER;                             // charger cette valeur dans l'élément 1 du tableau
        for (i = 2; i < 9; i++) {                     // boucle de chargement des données des octets suivants dans le tableau
          uart[i] = Serial3.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {                  //contrôle de l'intégrité des données : le dernier octet fde la somme des éléménts 0 à 7
          // du tableau doit être égale à la valeur de l'éléments 8.
          d3 = uart[2] + uart[3] * 256;                   //calcul de la distance en cm mesurée par le laser
          strengthFront = uart[4] + uart[5] * 256;           //calcul de la force du signal
        }
      }
    }
  }
}


void Erreurs() {
  // cette fonction calcul l'erreur par rapport a l'angle et la distance de consigne
  alpha = atan((d2 - d1) / l) * 180 / 3.14159;        // calcul en ° de l'angle axe véhicule avec tangente locale trajectoire. atan sort des rd donc à transformer en °
  erreurs = K1 * (consigneAlpha - alpha) + K2 * (consigneD - d2); // erreur totale en ° : somme erreur alpha + erreur distance
  angleBraq = ( -1.*erreurs) + 87;
  //angleBraq=angleDefault;
}


void compteur() {
  // cette fonction réalise l'incrementation de l'odometrie  
  val = digitalRead(pinHall);
  if (val == HIGH && lastval == LOW) {
    lastval = HIGH;
  }
  else if (val == LOW && lastval == HIGH) {
    lastval = LOW;
    deltat = micros() - lastmicros;
    lastmicros = micros();
    pulse_count++;
 //nb_pulses_sec_hall = (1000000.0) / deltat;        // nombre tours si un seul aimant, donc sur 1000 000 microsec
    //VITROThall = nb_pulses_sec_hall * (60.);          // vitesse rotation roue en t/mn si un seul aimant
    //VIT = (VITROThall / 2) * 60 * (3.1416 * diamRoue / 100000.); // avec 2 aimants, diamètre roue 10.6 cm,transformé en km, d'où vitesse en km/h
  }
}


void ouEstil() {
  cumDist = (pulse_count) * (3.1416 * diamRoue) / 2; // cumul de la distance en cm
  for (int i = 0; i <= 8; i++) {
    if (cumDist >= tronCon[i][0] and cumDist < tronCon[i + 1][0]) {
      PwmVIT = tronCon[i][1];
    }
  }
}


void movServo() {
  angleBraq = constrain(angleBraq, 55, 115);
  Serial.print("myservo");
  Serial.println(angleBraq);
  myServo.write(angleBraq);
}


void motor() {
  if (d3 >= 300.) {
    myEsc.writeMicroseconds(1635.);      // commande vitesse du tronçon avec l'asservissement
    TESTF=1;
  }
  else {
    if(TESTF == 1){
      PwmVIT=1500.;
      delay(500);
      PwmVIT=1420;                              // impulsion marche AR pour freiner
      myEsc.writeMicroseconds(PwmVIT);
      delay(150);
      TESTF=0;
    }
    myEsc.writeMicroseconds(1610);     // cde vitesse max 6 km/h pendant le virage - à ajuster sur piste
  }
}


void debug() {
  Serial.print("dist d3 = ");
  Serial.print(d3);
  Serial.print("  dist d2 = ");
  Serial.print(d2);                             //affichage distance mesuré par le laser AV
  Serial.print("  dist d1 = ");
  Serial.print(d1);                             //affichage distance mesuré par le laser AR
  Serial.print("    alpha= ");
  Serial.print(alpha);                         //affichage angle
  Serial.print("    erreurs= ");
  Serial.print(erreurs);                         //affichage angle
  //Serial.print(" Consigne km/h ");
  //Serial.print(consi);
  //Serial.print(" PWM ");
  //Serial.print(PwmVIT);
  //Serial.print("    Frequence loop : ");
  //Serial.print(1000000/t1);                   // fréquence d'exécution de la boucle
  //Serial.print("    nb de tours= ");
  //Serial.print(pulse_count);
  //Serial.print("    tours/s= ");
  //Serial.print(nb_pulses_sec_hall);
  //Serial.print("    vit km/h= ");
  //Serial.println(VIT);
  Serial.print(" BRAQUAGE ");
  Serial.print(angleBraq);
  Serial.print("    Distance parcourue cm: ");
  Serial.println(cumDist);
}
