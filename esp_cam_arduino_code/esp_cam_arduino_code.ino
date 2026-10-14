void setup() {
  Serial.begin(115200);                        // USB, voor de seriële monitor
  Serial1.begin(115200, SERIAL_8N1, 44, 43);   // RX = GPIO44 (D7), TX = GPIO43 (D6)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);             // LED uit (actief laag)
}

void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    Serial.print("Ontvangen: ");
    Serial.println(cmd);
    if (cmd == "START") {
      digitalWrite(LED_BUILTIN, LOW);          // LED aan
      Serial1.print("ACK\n");
    }
  }
}