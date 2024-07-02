#include <Servo.h> // incluir Servo library
// 180 horizontal MAX
Servo horizontal; // servo horizontal
int servoh = 180; // 90; // ficar servo na horizontal
int servohLimitHigh = 180;
int servohLimitLow = 65;
// 65 degrees MAX
Servo vertical; // vertical servo
int servov = 45; // 90; // ficar servo na vertical
int servovLimitHigh = 80;
int servovLimitLow = 15;
// LDR pin conexões
// name = analogpin;
int ldrlt = 0; //LDR do canto superior à esquerda - canto inferior esquerdo
int ldrrt = 1; //LDR do canto superior à direita - canto inferior direito
int ldrld = 2; //LDR do canto inferior à esquerda - canto superior esquerdo
int ldrrd = 3; //LDR do canto inferior à direita - canto superior direito
void setup()
{
 Serial.begin(9600);
// conexões servo
// name.attacht(pin);
horizontal.attach(9);
 vertical.attach(10);
 horizontal.write(180);
 vertical.write(45);
 delay(3000);
}
void loop()
{
 int lt = analogRead(ldrlt); // canto superior à esquerda
 int rt = analogRead(ldrrt); // canto superior à direita
 int ld = analogRead(ldrld); // canto inferior à esquerda
 int rd = analogRead(ldrrd); // canto inferior à direita
 // int dtime = analogRead(4)/20; // ler potenciômetros
 // int tol = analogRead(5)/4;
 int dtime = 10;
 int tol = 50;
 int avt = (lt + rt) / 2; // valor médio superior
 int avd = (ld + rd) / 2; // valor médio inferior
 int avl = (lt + ld) / 2; // valor médio esquerda
 int avr = (rt + rd) / 2; // valor médio direita
 int dvert = avt - avd; // Verificar a diferença de cima e para baixo
 int dhoriz = avl - avr;// Verificar a diferença de esquerda e direita
 Serial.print(avt);
 Serial.print(" ");
 Serial.print(avd);
 Serial.print(" ");
 Serial.print(avl);
 Serial.print(" ");
 Serial.print(avr);
 Serial.print(" ");
 Serial.print(dtime);
 Serial.print(" ");
 Serial.print(tol);
 Serial.println(" ");
if (-1*tol > dvert || dvert > tol)
 {
 if (avt > avd)
 {
 servov = ++servov;
 if (servov > servovLimitHigh)
 {
 servov = servovLimitHigh;
 }
 }
 else if (avt < avd)
 {
 servov= --servov;
 if (servov < servovLimitLow)
 {
 servov = servovLimitLow;
 }
 }
 vertical.write(servov);
 }

 if (-1*tol > dhoriz || dhoriz > tol)
 {
 if (avl > avr)
 {
 servoh = --servoh;
 if (servoh < servohLimitLow)
 {
 servoh = servohLimitLow;
 }
 }
 else if (avl < avr)
 {
 servoh = ++servoh;
 if (servoh > servohLimitHigh)
 {
 servoh = servohLimitHigh;
 }
 }
 else if (avl = avr)
 {
 // nothing
 }
 horizontal.write(servoh);
 }
 delay(dtime);
}
int motor1Pin1 = 3; // pin 2 on L293D IC
int motor1Pin2 = 4; // pin 7 on L293D IC
int enable1Pin = 6; // pin 1 on L293D IC
int motor2Pin1 = 8; // pin 10 on L293D IC
int motor2Pin2 = 9; // pin 15 on L293D IC
int enable2Pin = 11; // pin 9 on L293D IC
int state;
int flag=0;
int stateStop=0;
void setup() {
// define os pinos como saídas:
pinMode(motor1Pin1, OUTPUT);
pinMode(motor1Pin2, OUTPUT);
pinMode(enable1Pin, OUTPUT);
pinMode(motor2Pin1, OUTPUT);
pinMode(motor2Pin2, OUTPUT);
pinMode(enable2Pin, OUTPUT);
// define enable1Pin e enable2Pin para que o motor possa ligar:
digitalWrite(enable1Pin, HIGH);
digitalWrite(enable2Pin, HIGH);
// Inicializa a comunicação serial em 9600 bits por segundo:
Serial.begin(9600)
}
void loop() {
if(Serial.available() > 0){
state = Serial.read();
flag=0;
 }
// Se o estado for '1' o motor ligará
 if (state == '1') {
 digitalWrite(motor1Pin1, HIGH);
 digitalWrite(motor1Pin2, LOW);
 digitalWrite(motor2Pin1, LOW);
 digitalWrite(motor2Pin2, HIGH);
 if(flag == 0){
 Serial.println("Liga!");
 flag=1;

}

}
 // Se o estado for '3' o motor irá parar
 else if (state == '3' || stateStop == 1) {
 digitalWrite(motor1Pin1, LOW);
 digitalWrite(motor1Pin2, LOW);
 digitalWrite(motor2Pin1, LOW);
 digitalWrite(motor2Pin2, LOW);
 if(flag == 0){
 Serial.println("Pare!");
 flag=1;

}
 stateStop=0;

}
 //Para fins de depuração
 //Serial.println(state);
} 
