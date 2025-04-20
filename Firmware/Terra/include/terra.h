// imagelist.h
#ifndef TERRA_H
#define TERRA_H

#define BITMAP_WIDTH  240
#define BITMAP_HEIGHT 240

// Nagigation States
enum NavigationState {
    E_NOT_STARTED,
    E_NAVIGATING,
    E_AT_CHECKPOINT,
    E_TRAIL_ENDED
  };
  
// Image Types
enum ImageType {
    E_NONE,
    E_PENDING,
    E_GOTOSTART,
    E_ARROW_N,
    E_ARROW_NNE,
    E_ARROW_NE,
    E_ARROW_ENE,
    E_ARROW_E,
    E_ARROW_ESE,
    E_ARROW_SE,
    E_ARROW_SSE,
    E_ARROW_S,
    E_ARROW_SSW,
    E_ARROW_SW,
    E_ARROW_WSW,
    E_ARROW_W,
    E_ARROW_WNW,
    E_ARROW_NW,
    E_ARROW_NNW,
    E_CHECKPOINT_1,
    E_CHECKPOINT_2,
    E_CHECKPOINT_3,
    E_CHECKPOINT_4,
    E_CHECKPOINT_5,
    E_CHECKPOINT_6,
    E_CHECKPOINT_7,
    E_CHECKPOINT_8,
    E_CHECKPOINT_9,
    E_CHECKPOINT_10
  };

// Function Prototypes
void setup();
void loop();
void handleGPSData();
bool readSerialGPS();
bool readGPS();
void determineTrailStatusAndNavigate();
void processGPSData(double lat, double lon);
bool nonBlockingDelay(unsigned long ms);
void displayImage(ImageType image);
void fadeOut();
void fadeIn();
double getDistanceTo(double lat, double lon);
String getCardinalTo(double lat, double lon);
int getCourseTo(double lat, double lon);
int readCompass();
int calculateRelativeDirection(int currentAngle, int targetAngle);
void drawBitmap(const unsigned char* bitmap);
ImageType selectArrowImage(int relativeDirection);
void triggerProximityVibration();
static void smartDelay(unsigned long ms);

#endif // TERRA_H