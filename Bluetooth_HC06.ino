String command = ""; // Stores response of the HC-06 Bluetooth device

void setup() {
  Serial.begin(9600);
  delay(10);
  pinMode(13,OUTPUT);
  delay(10);
  digitalWrite(13, HIGH);
}

void loop() {
    if (Serial.available()) {
    while(Serial.available()) { // While there is more to be read, keep reading.
      digitalWrite(13, LOW);
      command += (char)Serial.read();
    }
    Serial.println(command);
    command = ""; // No repeats
  }
}
