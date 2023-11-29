char msgs[5][15] = {
  "Up Key OK    ",
  "Left Key OK  ",
  "Down Key OK  ",
  "Right Key OK ",
  "Select Key OK"
};

char start_msg[15] = {
  "Start loop "
};

int adc_key_val[5] = { 30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;

void setup(void) {
  pinMode(13, OUTPUT);              //  We'll use the debug LED to output a heartbeat
  Serial.begin(9600);
 
  /* Print that we made it here */
  Serial.println(start_msg);
}
 
void loop() {
  adc_key_in = analogRead(7);       //  Read the value from the sensor 
  digitalWrite(13, HIGH);

  //  Get the key
  key = get_key(adc_key_in);        //  Convert into key press

  if (key != oldkey) {              //  If keypress is detected
    delay(50);                      //    wait for debounce time

    adc_key_in = analogRead(7);     //  Read the value from the sensor

    key = get_key(adc_key_in);      //  Convert into key press

    if (key != oldkey) {        
      oldkey = key;

      if (key >= 0){
        Serial.println(adc_key_in, DEC);
        Serial.println(msgs[key]);
      }
    }
  }

  digitalWrite(13, LOW);
}

// Convert ADC value to key number
int get_key (unsigned int input) {  
  int k;
  int retval = -1;

  for (k = 0; ((k < NUM_KEYS) && (retval < 0)); k++) {
    if (input < adc_key_val[k]) { 
      retval = k; 
    }
  }
  
  return retval;
}
