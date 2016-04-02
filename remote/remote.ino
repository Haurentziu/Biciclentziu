#define xPin 0
#define yPin 1
#define MSPin 11
void setup() {
  Serial.begin(9600);
  
  pinMode(xPin, INPUT);
  pinMode(yPin, OUTPUT);

  pinMode(MSPin, INPUT_PULLUP);

}

void loop() {
  int xPos = analogRead(xPin);
  int yPos = analogRead(yPin);

  boolean state = digitalRead(MSPin);

  Serial.print("X: ");
  Serial.print(xPos);
  Serial.print(" |Y: ");
  Serial.print(yPos);
  Serial.print(" | Button: ");
  Serial.println(state);
  delay(100);
}
