#define Bittle
//#define Nybble
//#define Cub

#define BiBoard
//#define BiBoard2

#if defined BiBoard
#define DOF 12
#elif defined BiBoard2
#define DOF 16
#else 
#define DOF 16
#endif

#if defined Nybble || defined Bittle
#define WALKING_DOF 8
#elif defined Cub
#define WALKING_DOF 12
#else
#define WALKING_DOF 12
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.print(DOF);
  Serial.print('\t');
  Serial.print(WALKING_DOF);
  Serial.print('\t');

}

void loop() {
  // put your main code here, to run repeatedly:

}
