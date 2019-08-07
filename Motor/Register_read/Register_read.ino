#define   pin_left_EncoderB     5 //PD5

void setup() {
  pinMode(pin_left_EncoderB, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  Serial.println(((PIND & (1 << pin_left_EncoderB)) >> pin_left_EncoderB) ? "Off" : "On");
  delay(100);
}
