// imagelist.h
#ifndef TERRA_H
#define TERRA_H

// Nagigation States
enum NavigationState {
    NOT_STARTED,
    NAVIGATING,
    AT_CHECKPOINT,
    TRAIL_ENDED
  };
  
// Image Types
enum ImageType {
    NONE,
    I_PENDING,
    GOTOSTART,
    ARROW_N,
    ARROW_NNE,
    ARROW_NE,
    ARROW_ENE,
    ARROW_E,
    ARROW_ESE,
    ARROW_SE,
    ARROW_SSE,
    ARROW_S,
    ARROW_SSW,
    ARROW_SW,
    ARROW_WSW,
    ARROW_W,
    ARROW_WNW,
    ARROW_NW,
    ARROW_NNW,
    CHECKPOINT_1,
    CHECKPOINT_2,
    CHECKPOINT_3,
    CHECKPOINT_4,
    CHECKPOINT_5,
    CHECKPOINT_6,
    CHECKPOINT_7,
    CHECKPOINT_8,
    CHECKPOINT_9,
    CHECKPOINT_10
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