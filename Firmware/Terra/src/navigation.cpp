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

// Private members
void triggerProximityVibration();
void IRAM_ATTR navUpdateISR();
void IRAM_ATTR navGPSFixISR();

// Update timer
hw_timer_t *navUpdateTimer = NULL;
static volatile bool navPendingUpdate = false; //  Nav update timer expired

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
    pinMode(PIN_GPS_STBY, OUTPUT);
    pinMode(LED_BUILTIN,  OUTPUT);  // Using LED_BUILTIN as FIX indicator

    attachInterrupt(PIN_GPS_FIX, &navGPSFixISR, CHANGE);
    gpsHWFix = digitalRead(PIN_GPS_FIX);

    digitalWrite(PIN_GPS_RST, !LOW);
    digitalWrite(PIN_GPS_STBY, LOW);
    digitalWrite(LED_BUILTIN,  LOW);

#ifndef DEBUG_NAVIGATION
    gpsStream.begin(9600, SERIAL_8N1, PIN_RX1, PIN_TX1);
#endif // DEBUG_NAVIGATION

    navigationState = E_NOT_STARTED;

    // Navigation update timer
    navUpdateTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(navUpdateTimer, &navUpdateISR, true);
    timerAlarmWrite(navUpdateTimer, NAV_UPDATE_INTERVAL_US, true);
    timerAlarmEnable(navUpdateTimer);
}

/**
 * @brief Refreshses location and udpates navigation
 *
 */
void navUpdate()
{
    if (navPendingUpdate)
    {
        navPendingUpdate = false;
        Serial.printf("GPS Fix: %s\n", navLocationKknown() ? "Fixed" : "Unknown");

        navFeedGPSData();
        navUpdateTrailStatusAndNavigate();

    }
    navUpdateHaptics();
}

/**
 * @brief Interrupt service for navigation update timer
 *
 */
void IRAM_ATTR navUpdateISR()
{
    navPendingUpdate = true;
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
 * @return true location is known
 * @return false location is unknown
 */
bool navLocationKknown()
{
    if (gpsHWFix)
    {
        return gps.location.isValid();
    }
    return false;
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
        // TODO: Check how long this really runs for... Ideally we are not blocking for 500ms...
        //       May need to increase GPS baudrate on hardware
        gps.encode(gpsStream.read());
        if (millis() - readStart < NAV_GPS_MAX_READ_MS)
            break;
    }
    navUpdateLocationGlobals();
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
 * @brief Main navigation logic
 * @todo REFACTOR ME!!
 */
void navUpdateTrailStatusAndNavigate()
{
    static double lastDistance = -1;

    // Obtain the target coordinates based on the trail's current status
    double targetLat = !trailStarted ? startLat : stopLats[currentStop - 1];
    double targetLon = !trailStarted ? startLon : stopLons[currentStop - 1];

    // Calculate the current distance and direction to the target
    double distance = getDistanceTo(targetLat, targetLon);
    String cardinal = getCardinalTo(targetLat, targetLon);
    int targetAngle = getCourseTo(targetLat, targetLon); // Assume implementation exists
    int currentAngle = compassReadHeading();

    // Calculate the relative direction for navigation
    int relativeDirection = calculateRelativeDirection(currentAngle, targetAngle);

    if (navigationState == E_NOT_STARTED)
    {
        if (!navDataReceived)
        {
            displaySetImage(E_PENDING); // Show E_PENDING only when waiting for the first GPS data
        }
        else
        {
            Serial.println("Please proceed to the start of the trail.");
            displaySetImage(E_GOTOSTART); // Now we're sure we've received data, show GOTOSTART
        }

        // Transition to navigating state once within close range to the start and data has been received
        if (navDataReceived && distance <= NAV_CHECKPOINT_THRESH_M)
        {
            trailStarted = true;
            currentStop = 1;
            Serial.println("Trail started. Heading to Stop 1.");
            navigationState = E_NAVIGATING;
        }
        return; // Continue to skip rest of the function logic when NOT_STARTED
    }

    // Only update navigation arrow if we are in the navigating phase
    if (navigationState == E_NAVIGATING)
    {
        ImageType arrowImage = selectArrowImage(relativeDirection);
        displaySetImage(arrowImage);
    }

    // Start of the trail
    if (!trailStarted)
    {
        if (distance <= NAV_CHECKPOINT_THRESH_M)
        { // "Closeness" threshold
            trailStarted = true;
            currentStop = 1; // Moving towards the first checkpoint
            Serial.println("Trail started. Heading to Stop 1.");
            navigationState = E_NAVIGATING;
        }
        else
        {
            Serial.println("Please proceed to the start of the trail.");
            if (navDataReceived)
            {
                displaySetImage(E_GOTOSTART); // Indicating to go to the starting point
            }
        }
    }
    // Navigating the trail
    else
    {
        if (distance <= NAV_CHECKPOINT_THRESH_M && currentStop <= numberOfStops)
        {
            if (navigationState != E_AT_CHECKPOINT)
            {
                // Just arrived at this checkpoint
                Serial.print("Arrived at Stop ");
                Serial.println(currentStop);
                ImageType checkpointImage = static_cast<ImageType>(E_CHECKPOINT_1 + currentStop - 1);
                displaySetImage(checkpointImage); // Show the checkpoint image
                navigationState = E_AT_CHECKPOINT; // Update state to at checkpoint
                lastCheckpointTime = millis();     // Capture the time we arrived at the checkpoint

                // Check if this is the final stop
                if (currentStop == numberOfStops)
                {
                    Serial.println("Final stop reached. Trail is complete.");
                    // Display I_PENDING image to indicate completion
                    displaySetImage(E_PENDING);
                    // Optionally, you might want to change the navigation state or take other actions here
                    navigationState = E_TRAIL_ENDED; // Resetting the state to NOT_STARTED or another appropriate state
                }

                navProximityHapticsFlag = false; // Allow vibration to trigger again for the next stop
                currentStop++;                   // Prepare for the next stop or complete the trail
            }
        }
        else if (navigationState == E_AT_CHECKPOINT)
        {
            if (millis() - lastCheckpointTime > 5000)
            {                                    // 5 seconds have passed since arriving at the checkpoint
                navigationState = E_NAVIGATING;  // Transition back to navigating after the delay
                navProximityHapticsFlag = false; // Reset vibration trigger flag
            }
        }
        else if (navigationState == E_NAVIGATING && currentStop <= numberOfStops)
        {
            // Continue with the condition to update navigation info only if there's a significant change in distance
            if (abs(lastDistance - distance) > 0.5)
            {
                Serial.print("Distance to next stop: ");
                Serial.print(distance, 1); // One decimal place for distance
                Serial.print(" meters. Direction to next stop: ");
                Serial.print(cardinal);
                Serial.print(" (Target angle: ");
                Serial.print(targetAngle);
                Serial.println(" degrees)");
                lastDistance = distance; // Update lastDistance for next comparison

                // Here, potentially display the arrow again if needed, based on your logic for selecting and displaying arrows
                ImageType arrowImage = selectArrowImage(relativeDirection);
                displaySetImage(arrowImage);

                // Set the flag to start continuous vibration when within a certain distance from the next stop
                if (distance <= NAV_HAPTICS_THRESH_M && !navProximityHapticsFlag)
                {
                    navProximityHapticsFlag = true;
                    lastVibrationTime = millis(); // Ensure we start timing from now
                }
            }
        }
    }
}

/**
 * @brief Manage playback of proximity-based haptics
 *
 */
void navUpdateHaptics()
{
    // Continuously trigger vibration when within a certain distance of the next stop
    if (navProximityHapticsFlag && ((millis() - lastVibrationTime) >= NAV_HAPTICS_DELAY_MS))
    {
        // Check if we should still be vibrating (if within a certain distance and navigating)
        if (distance <= navProximityHapticsFlag && navigationState == E_NAVIGATING)
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
        Serial.println("Invalid format. Please enter in format: lat,lon");
        return false; // Data parsing failed
    }
}
#endif // DEBUG_NAVIGATION