/* Include All Required Libraries */
//tigom is the test
#include <Adafruit_GFX.h>               // Include core graphics library for the display
#include <Adafruit_SSD1306.h>           // Include Adafruit_SSD1306 library to drive the display
#include <Fonts/FreeMonoBold12pt7b.h>   // Add a custom font
#include <Fonts/FreeMono9pt7b.h>        // Add a custom font
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Arduino.h>                    // Magnetometer
#include <Wire.h>                       // Magnetometer
#include <HMC5883L_Simple.h>            // Magnetometer

/* Define SPI Pins For OLED Display */
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8

Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);  // Create display
TinyGPSPlus gps; 
HMC5883L_Simple Compass;
SoftwareSerial gps_serial(5,4);            // (Rx, Tx)

int Variable;                              // Create a variable to have something dynamic to show on the display

/* Values At Current Position (Home Point) Prints In Float Format - Raw GPS Data */
float lat_home;
float long_home;

float lat_away_present, long_away_present;

/* Values Of Destination Position (Away Point) in Float Array Format - GPS Data Will Be Fed by Us In The Coding */


float lat_away[] = {13.02713297753741, 13.027149726383, 13.02802844896464, 13.02790884259175};        //changed value lat
float long_away[] = {80.25633983991604, 80.25548711610242, 80.25544412958953, 80.25635188432968};     //changed value lon


/* Button Setup */
int button = 3;
int button_state;
int counter;


void setup()  
{      
  Serial.begin(9600);
  gps_serial.begin(9600);
  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC);      // Initialize display 
  display.clearDisplay();                   // Clear the buffer
  display.setTextColor(WHITE);              // Set color of the text
  display.setRotation(0);                   // Set orientation. Goes from 0, 1, 2 or 3
  display.setTextWrap(false);               // By default, long lines of text are set to automatically “wrap” back to the leftmost column. To override this behavior (so text will run off the right side of the display - useful for scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored with setTextWrap(true).
  display.dim(0); 

  pinMode(button,INPUT);                    // Button input
  counter = 0;                              // Counter
}

void loop()
{
  display.clearDisplay();                       // Clear the display so we can refresh
  
  display.drawRect(0, 0, 128, 30, WHITE);       // Draw Rectangle - X-axis, Y-axis Min Max Values

  /* Lat Long Values Obtained From GPS And Displayed On OLED Display */
  //display.setFont(&FreeMono9pt7b);            // Set a custom font
  display.setTextSize(1);
  display.setCursor(3, 2);                      // (x,y)
  display.print("LAT ");      
  //display.setFont(&FreeMono9pt7b);            // Set a custom font
  display.setTextSize(1);
  display.setCursor(38, 2);
  display.println(lat_home, 6);                 // OLED Output & 6 Denotes Number Of Decimal Points To Be Printed When We Infer Data From GPS In Float Format
  //Serial.println(lat_home);                   // Serial Monitor
  lat_home = float(gps.location.lat());         // Float Command To Get Long Raw Data, No Need To Mention Decimals Point When It Is Derived In Float Format. Instead You Can Mention It In Display/Serial Command.

  //display.setFont(&FreeMono9pt7b);            // Set a custom font
  display.setTextSize(1);
  display.setCursor(3, 10);                     // (x,y)
  display.println("LONG ");                     // Text or value to print       
  //display.setFont(&FreeMono9pt7b);            // Set a custom font
  display.setTextSize(1);
  display.setCursor(38, 10);
  display.println(long_home, 6);                // OLED Output & 6 Denotes Number Of Decimal Points To Be Printed When We Infer Data From GPS In Float Format
  //Serial.println(long_home);                  // Serial Monitor 
  long_home = float(gps.location.lng());        // Float Command To Get Long Raw Data, No Need To Mention Decimals Point When It Is Derived In Float Format. Instead You Can Mention It In Display/Serial Command.
  
  while (gps_serial.available() > 0) 
    if (gps.encode(gps_serial.read()))
    {
      //gpsCoordinates();                       // To Print Raw Data - Function Called
      //lat_long_compare();                     // Void lat_long_compare - Function Called
      //DistanceMeasurement();
      //display.display();                      // Print everything we set previously
    }
      lat_long_compare();
      DistanceMeasurement();
      display.display(); 
      
  button_state = digitalRead (button);
  
  if(button_state == HIGH)
  {
    // Importing CSV File For Feeding Lat & Long Data, Instead Of Feeding One By One In The Code
 
           if(counter < 4)                      // Number Of Values You Feed in lat_away[] & long_away[] Array
           {
             lat_away_present = lat_away[counter];
             long_away_present = long_away[counter];

             //lat_long_compare();
             //DistanceMeasurement();
             display.display();
             delay(1000);
             counter++;
             }
             /* Once Counter Value Reaches The Given Value i.e. 11 In This Code,
                It Will Go Back To 0th Value And Starts Counting From First */
             if(counter == 4)
             {
             counter = 0;
             }
   }       
}

void lat_long_compare()
{   
  if ((lat_home < lat_away_present) && (long_home < long_away_present))
  {
    display.setTextSize(1);
    display.setCursor(80, 18);              // (x,y)
    display.println(" North ");             // Text or value to print
    //Serial.println(" North ");            // Serial Monitor 
  }
  else if ((lat_home > lat_away_present) && (long_home > long_away_present))
  {
    display.setTextSize(1);
    display.setCursor(80, 18);              // (x,y)
    display.println(" South ");             // Text or value to print
    //Serial.println(" South ");            // Serial Monitor  
  }
  else if ((lat_home > lat_away_present) && (long_home < long_away_present))
  {
    display.setTextSize(1);
    display.setCursor(80, 18);              // (x,y)
    display.println(" East ");              // Text or value to print
    //Serial.println(" East ");             // Serial Monitor  
  }
  else if ((lat_home < lat_away_present) && (long_home > long_away_present))
  {
    display.setTextSize(1);
    display.setCursor(80, 18);              // (x,y)
    display.println(" West ");              // Text or value to print
    //Serial.println(" West ");             // Serial Monitor  
  }
}

void DistanceMeasurement()
{  
   float ToRad = PI / 180.0;
   float R = 6371;                            // Radius of Earth in Km
   
   float Dist_Lat = ((lat_away_present - lat_home)) * ToRad;
   float Dist_Long = ((long_away_present - long_home)) * ToRad;   
   float a = sin(Dist_Lat/2) * sin(Dist_Lat/2) + cos(lat_home * ToRad) * cos(lat_away_present * ToRad) * sin(Dist_Long/2) * sin(Dist_Long/2);     // a - square of half the chord length between the points)         
   float c = 2 * atan2(sqrt(a), sqrt(1-a));   // c - Angular Distance in Radians
   
   float DistanceToCover = R * c * 1000;
   //return DistanceToCover;
   
   //display.setFont(&FreeMono9pt7b);         // Set a custom font
   display.setTextSize(1);
   display.setCursor(3, 18);                  // (x,y)
   display.println(counter);                  // Text or value to print       
   //display.setFont(&FreeMono9pt7b);         // Set a custom font
   display.setTextSize(1);
   display.setCursor(38, 18);
   display.println(DistanceToCover);          // OLED Output
   //Serial.print(" DistanceToCover =  ");    // Serial Monitor
   //Serial.println(DistanceToCover);         // Serial Monitor
}
