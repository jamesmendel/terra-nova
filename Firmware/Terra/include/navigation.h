/**
 * @file navigation.h
 * @author James Mendel (jrm.mendel@gmail.com)
 * @brief Implements GPS driver and navigation logic
 * @date 2025-04-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <TinyGPSPlus.h>

// #define DEBUG_NAVIGATION    // Global flag to enable GPS debugging by feeding dummy data through serial.
#define NAV_UPDATE_INTERVAL_US  1000000 // 1 second
#define NAV_GPS_MAX_READ_MS     500     // 0.5 second
#define NAV_CHECKPOINT_THRESH_M 10  // Distance from the checkpoint that is considered an arrival.
#define NAV_HAPTICS_THRESH_M    20  // When to trigger vibration to indicate the checkpoint is getting close.
#define NAV_HAPTICS_DELAY_MS    500 // To determine the time between proximity vibrations when close to the current stop.

enum NavigationState {
    NAV_NOT_STARTED,
    NAV_NAVIGATING,
    NAV_AT_CHECKPOINT,
    NAV_TRAIL_ENDED
};
static NavigationState navigationState = NAV_NOT_STARTED;

static HardwareSerial& gpsStream = Serial1;
static TinyGPSPlus gps;
static double currentLat = 0;
static double currentLon = 0;
static bool navDataReceived = false;
static bool gpsHWFix = false;


void initNav();
void navUpdate();
bool navLocationKknown();
void navFeedGPSData();
void navUpdateTrailStatusAndNavigate();
void navUpdateLocationGlobals();
void navServiceHaptics();
double getDistanceTo(double lat, double lon);
String getCardinalTo(double lat, double lon);
int getCourseTo(double lat, double lon);
int calculateRelativeDirection(int currentAngle, int targetAngle);

void _handleNotStartedState();
void _handleNavigatingState(double distance, int relativeDirection);
void _handleAtCheckpointState();
void _arriveAtCheckpoint();
void _updateDistanceAndHaptics(double distance);

#ifdef DEBUG_NAVIGATION
bool navReadSerialGPS();
#endif // DEBUG_NAVIGATION

#endif // NAVIGATION_H