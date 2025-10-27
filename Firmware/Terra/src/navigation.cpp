/**
 * @file navigation.cpp
 * @author James Mendel (jrm.mendel@gmail.com)
 * @brief Implements GPS driver and navigation logic
 * @date 2025-04-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>
#include "navigation.h"
#include "terra.h"
#include "config.h"
#include "compass.h"
#include "haptics.h"
#include "display.h"
#include "driver/timer.h"
#include "esp_log.h"
static const char* LOGTAG = "Navigation";

// Private members
void triggerProximityVibration();
void IRAM_ATTR navUpdateISR();
void IRAM_ATTR navGPSFixISR();

bool trailStarted = false;            //  Have we reached the start of the trail?
bool navProximityHapticsFlag = false; // Haptic proximity flag
int currentStop = 0;                  // Current index into list of stops
double distance = 0;                  // Distance to the next checkpoint
unsigned long lastVibrationTime = 0;  // Tracks the last time vibration was triggered
unsigned long lastCheckpointTime = 0; // Timestamp of when the last checkpoint was reached (ms)

/**
 * @brief Initialize the GPS module and navigation state
 *
 */
void initNav()
{
    // GPS
    pinMode(PIN_GPS_FIX, INPUT);
    pinMode(PIN_GPS_RST, OUTPUT);
    pinMode(LED_BUILTIN,  OUTPUT);  // Using LED_BUILTIN as FIX indicator

    attachInterrupt(PIN_GPS_FIX, navGPSFixISR, CHANGE);
    gpsHWFix = digitalRead(PIN_GPS_FIX);

    digitalWrite(PIN_GPS_RST, !LOW);
    digitalWrite(LED_BUILTIN,  LOW);

#ifndef DEBUG_NAVIGATION
    gpsStream.begin(9600, SERIAL_8N1, PIN_RX1, PIN_TX1);
    // gpsStream.println(GPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);
    // gpsStream.println(GPS_PMTK_API_SET_GNSS_SEARCH_MODE);
#endif // DEBUG_NAVIGATION

    navigationState = NAV_NOT_STARTED;
}

/**
 * @brief Refreshses location and udpates navigation
 *
 */
void navUpdateTask()
{
    while(1)
    {
        if(!cmpReady) {
            displaySetImage(I_PENDING);
            ESP_LOGW(LOGTAG, "Compass is not ready to navigate.");
            initCompass();  // try to re-init
            terraSleep(500);
            goto taskEnd;
        }
             
        navFeedGPSData();
        navUpdateTrailStatusAndNavigate();

        taskEnd:
        vTaskDelay(NAV_UPDATE_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Interrupt service for GPS Fix state
 *
 */
void IRAM_ATTR navGPSFixISR()
{
    gpsHWFix = digitalRead(PIN_GPS_FIX);
    digitalWrite(LED_BUILTIN, gpsHWFix);    // Fix indicator
}

/**
 * @brief Tells us if there is a lock on our current location
 *
 * @return true location is known and fresh
 * @return false location is unknown
 */
bool navLocationKknown()
{
    static bool prevState = true;
    bool state = false;
    if(gps.location.isValid()) {
        state = (gps.location.age() < 15 * 1000); // 15 second fix expiration
    }
    if (prevState != state) {
        prevState = state;
        ESP_LOGI(LOGTAG, "GPS Fix: %s", state ? "Fixed" : "Unknown");
    }
    return state; 
    // Removed while integrated GPS is not working.
    // if (gpsHWFix)
    // {
    //     return gps.location.isValid();
    // }
    // return false;
}

/**
 * @brief Read the GPS module signal for current lat/lon
 *
 */
void navUpdateLocationGlobals()
{
    if (gps.location.isValid())
    {
        currentLat = gps.location.lat();
        currentLon = gps.location.lng();
        navDataReceived = true;
    }
}

/**
 * @brief Feeds GPS object with serial data for a set duration.
 *
 * @param gpsStream Reference to the serial object to read from
 * @param ms Maximum time duration to read GPS
 */
void navFeedGPSData()
{
#ifndef DEBUG_NAVIGATION
    unsigned long readStart = millis();
    while (gpsStream.available() > 0)
    {
        char gpsDat = (char)gpsStream.read();
        gps.encode(gpsDat);

        #ifdef DEBUG_GPS_TO_SERIAL
        printf("%c", gpsDat);
        #endif // DEBUG_GPS_TO_SERIAL
    }
    navUpdateLocationGlobals();

    //TODO: REMOVE DEBUG PRINT
    #ifdef DEBUG_GPS
    char dbg[16];
    snprintf(dbg, 16, "SAT: %d  FIX? %d", gps.satellites.value(), navLocationKknown());
    tftDrawDebugOverlay(dbg, 0);
    char dbg3[16], dbg4[16];
    if(navLocationKknown()) {
        snprintf(dbg3, 16, "LAT: %03.03f", gps.location.lat());
        snprintf(dbg4, 16, "LNG: %03.03f", gps.location.lng());
    } else {
        memset(dbg3, ' ', 15);
        dbg3[15]='\0';
        memset(dbg4, ' ', 15);
        dbg4[15]='\0';
    }
    tftDrawDebugOverlay(dbg3, 1, 1);
    tftDrawDebugOverlay(dbg4, 1.5, 1);
    char dbg2[16];
    snprintf(dbg2, 16, "HEAD: %03d", compassGetHeading());
    tftDrawDebugOverlay(dbg2, 2);
    #endif

#else
    if (Serial.available() > 0)
    {
        if (navReadSerialGPS())
        {
            navDataReceived = true;
        }
    }
#endif
}

/**
 * @brief Main navigation logic & state handler
 *
 */
void navUpdateTrailStatusAndNavigate()
{
    static double lastDistance = -1; // Retain value across function calls

    if (navigationState == NAV_NOT_STARTED)
    {
        _handleNotStartedState();
        return;
    }

    double targetLat = !trailStarted ? startLat : stopLats[currentStop - 1];
    double targetLon = !trailStarted ? startLon : stopLons[currentStop - 1];
    
    double distance = getDistanceTo(targetLat, targetLon);
    #ifndef NAV_COMPASS_DEMO
    int targetAngle = getCourseTo(targetLat, targetLon);
    #else
    int targetAngle = 0; // point to N
    #endif
    int currentAngle = compassGetHeading();
    int relativeDirection = calculateRelativeDirection(currentAngle, targetAngle);

    if (navigationState == NAV_NAVIGATING)
    {
        // Update distance and haptics only if there is a significant change
        #ifndef NAV_COMPASS_DEMO
        if (abs(lastDistance - distance) > 0.5)
        #endif
        {
            lastDistance = distance;
            
            _handleNavigatingState(distance, relativeDirection);
            _updateDistanceAndHaptics(distance);

            printf("Distance to next stop: %.1f meters.\n", distance);
            printf("Direction to next stop: %d degrees.\n", relativeDirection);
        }

    }
    else if (navigationState == NAV_AT_CHECKPOINT)
    {
        _handleAtCheckpointState();
    }

}

/**
 * @brief NAV_STATE: Not started. Checks for user to get to the begining of trail,
 * and starts the navigation.
 * 
 */
void _handleNotStartedState()
{
    #ifdef NAV_COMPASS_DEMO
    static volatile long dbgNavStartedAt = millis();
    volatile long now = millis();
    if (now - dbgNavStartedAt > 10*1000) {
        navigationState= NAV_NAVIGATING;
        ESP_LOGD(LOGTAG, "Demo: compass navigation heading start even though distance threshold may not be met!");
        return;
    } else if (now - dbgNavStartedAt > 5*1000) {
        ESP_LOGD(LOGTAG, "Demo: Proceed to start of trail.");
        displaySetImage(I_GOTOSTART);
        return;
    }
    #endif

    if (!navDataReceived)
    {
        ESP_LOGI(LOGTAG, "Waiting for GPS lock to begin navigation.");
        displaySetImage(I_PENDING);
    }
    else
    {
        ESP_LOGI(LOGTAG, "Please proceed to the start of the trail.");
        displaySetImage(I_GOTOSTART);
    }

    if (navDataReceived && getDistanceTo(startLat, startLon) <= NAV_CHECKPOINT_THRESH_M)
    {
        trailStarted = true;
        currentStop = 1;
        ESP_LOGI(LOGTAG, "Trail started. Heading to Stop 1.");
        navigationState = NAV_NAVIGATING;
    }
}

/**
 * @brief NAV_STATE: Navigating. Checks distance between user and next stop and
 * transitions to checkpoint handler, if near enough. Otherwise, updates display
 * based on heading.
 * 
 * @param distance current distance between user and current stop
 * @param relativeDirection target heading towards current stop
 */
void _handleNavigatingState(double distance, int relativeDirection)
{
    if (distance <= NAV_CHECKPOINT_THRESH_M && currentStop <= numberOfStops)
    {
        _arriveAtCheckpoint();
    }
    else
    {
        ImageType arrowImage = selectArrowImage(relativeDirection);
        displaySetImage(arrowImage);
    }
}

/**
 * @brief NAV_STATE: At checkpoint. Performs checkpoint logic for 
 * the set period of time then returns to navigating state.s
 * 
 */
void _handleAtCheckpointState()
{
    if (millis() - lastCheckpointTime > 5000)
    {
        navigationState = NAV_NAVIGATING;
        navProximityHapticsFlag = false;
    }
}

/**
 * @brief NAV_STATE: Arrive at checkpoint. Updates display and state machine. 
 * 
 */
void _arriveAtCheckpoint()
{
    currentStop++;
    navProximityHapticsFlag = false;
    
    if (currentStop >= numberOfStops)
    {
        printf("Final stop reached. Trail is complete.\n");
        displaySetImage(I_PENDING);
        navigationState = NAV_TRAIL_ENDED;
        return;
    }
    
    ImageType checkpointImage = (ImageType)(I_CHECKPOINT_1 + currentStop - 1);
    if (checkpointImage > I_CHECKPOINT_10)
        checkpointImage = I_CHECKPOINT_10;
    displaySetImage(checkpointImage);
    
    navigationState = NAV_AT_CHECKPOINT;
    lastCheckpointTime = millis();

    Serial.printf("Arrived at Stop %d\n", currentStop);
}

/**
 * @brief Update distance if it has changed significantly, and 
 * trigger haptic feedback flag if within a certain distance. 
 * 
 * @param distance between user and current stop.
 * @param lastDistance to check if an update to distance is required.
 * @return double distance to the next stop.
 */
void _updateDistanceAndHaptics(double distance)
{
    if (distance <= NAV_HAPTICS_THRESH_M && !navProximityHapticsFlag)
    {
        navProximityHapticsFlag = true;
        lastVibrationTime = millis();
    }
}

/**
 * @brief Manage playback of proximity-based haptics
 *
 */
void navServiceHaptics()
{
    // Continuously trigger vibration when within a certain distance of the next stop
    if (navProximityHapticsFlag && ((millis() - lastVibrationTime) >= NAV_HAPTICS_DELAY_MS))
    {
        // Check if we should still be vibrating (if within a certain distance and navigating)
        if (distance <= navProximityHapticsFlag && navigationState == NAV_NAVIGATING)
        {
            playEffect(HAP_EFFECT_PROX);
            lastVibrationTime = millis();
        }
        else
        {
            // Stop vibrating if no longer within 20 meters or not in navigating state
            navProximityHapticsFlag = false;
        }
    }
}

/**
 * @brief Helper function to calculate the positive delta between two angles
 * 
 * @param currentAngle user's heading in degrees
 * @param targetAngle destination heading in degrees
 * @return int angle delta in positive degrees
 */
int calculateRelativeDirection(int currentAngle, int targetAngle)
{
    int difference = targetAngle - currentAngle;
    if (difference < 0)
    {
        difference += 360; // Adjust for negative differences
    }
    return difference % 360; // Ensure the result is within 0-359 degreess
}

/**
 * @brief Calculate the distance between current location and target
 *
 * @param lat target lattitude
 * @param lon target longitude
 * @return double distance (meters)
 */
double getDistanceTo(double lat, double lon)
{
    return TinyGPSPlus::distanceBetween(currentLat, currentLon, lat, lon);
}

/**
 * @brief Find the cardinal direction between current location and target
 *
 * @param lat target lattitude
 * @param lon target longitude
 * @return String cardinal direction (e.g. 'N', 'SWS', ...)
 */
String getCardinalTo(double lat, double lon)
{
    double courseTo = TinyGPSPlus::courseTo(currentLat, currentLon, lat, lon);
    String cardinal = TinyGPSPlus::cardinal(courseTo);
    return cardinal;
}

/**
 * @brief Find course heading between current location and target
 *
 * @param lat target lattitude
 * @param lon target longitude
 * @return int course heading (degrees)
 */
int getCourseTo(double lat, double lon)
{
    return TinyGPSPlus::courseTo(currentLat, currentLon, lat, lon);
}

#ifdef DEBUG_NAVIGATION
/**
 * @brief Debug method for setting global position lat and long via serial console (separated by commas)
 *
 * @return true successfully parsed serial position
 * @return false failed to parse seial position
 */
bool navReadSerialGPS()
{
    String inputString = Serial.readStringUntil('\n');
    int commaIndex = inputString.indexOf(',');

    // Check if the comma exists and it's not at the end of the string
    if (commaIndex != -1 && commaIndex < inputString.length() - 1)
    {
        currentLat = inputString.substring(0, commaIndex).toDouble();
        currentLon = inputString.substring(commaIndex + 1).toDouble();
        return true; // Data was successfully parsed
    }
    else
    {
        printf("Invalid format. Please enter in format: lat,lon\n");
        return false; // Data parsing failed
    }
}
#endif // DEBUG_NAVIGATION