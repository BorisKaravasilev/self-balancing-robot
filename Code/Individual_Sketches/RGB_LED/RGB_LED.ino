#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    1
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];


void setup() {
    delay( 3000 ); // power-up safety delay
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );
    
}

void loop() {
    leds[0].r = 255; 
    leds[0].g = 68; 
    leds[0].b = 21;

    
    FastLED.show();
    FastLED.delay(100);
}
