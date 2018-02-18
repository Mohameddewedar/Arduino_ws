void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print("reading:");
Serial.print(analogRead(A2));
Serial.print("||Actual: ");
Serial.print(map(analogRead(A2), 29.0, 663.0, 420.0, 590.0));
Serial.print("      ");
Serial.print("reading:");
Serial.print(analogRead(A1));
Serial.print("||Actual: ");
Serial.print(map(analogRead(A1), 99.0, 398.0, 330.0, 380.0));
Serial.print("      ");
Serial.print("reading:");
Serial.print(analogRead(A0));
Serial.print("||Actual: ");
Serial.println(map(analogRead(A0), 44.0, 428.0, 260.0,300.0));
delay(100);
}
// Conclusion
// Home Position for Arm
// d1 : 451
// d2 : 334
// d3 : 260