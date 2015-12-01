#define PIN_ESP8266_CHIP_POWERDOWN  (uint8_t)2U
#define PIN_ESP8266_RESET           (uint8_t)3U

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  pinMode(PIN_ESP8266_CHIP_POWERDOWN, OUTPUT);// CH_PD
  pinMode(PIN_ESP8266_RESET, OUTPUT);// RST
  digitalWrite(PIN_ESP8266_CHIP_POWERDOWN, HIGH);
  digitalWrite(PIN_ESP8266_RESET, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  int num_bytes_available = Serial.available();
  
  if(num_bytes_available > 0)
  {
    String send = Serial.readString();

    if(send.equals("RESET"))
    {
      Serial.println("\n\nRESET\n\n");
      digitalWrite(PIN_ESP8266_RESET, LOW);
      delay(100);
      digitalWrite(PIN_ESP8266_RESET, HIGH);
    }
    Serial.println("\nI SAID");
    Serial.println(send);
    Serial1.println(send);
  }

  delay(100);

  num_bytes_available = Serial1.available();
  if(num_bytes_available > 0)
  {
    String receive = Serial1.readString();
    Serial.println("\nWIFI SAID");
    Serial.println(receive);
  }
}
