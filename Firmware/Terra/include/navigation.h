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
#define  NAVIGATION_H

#include <TinyGPSPlus.h>

// #define DEBUG_NAVIGATION    // Global flag to enable GPS debugging by feeding dummy data through serial.
#define NAV_UPDATE_INTERVAL_US  1000000 // 1 second
#define NAV_CHECKPOINT_THRESH_M 10  // Distance from the checkpoint that is considered an arrival.
#define NAV_HAPTICS_THRESH_M    20  // When to trigger vibration to indicate the checkpoint is getting close.
#define NAV_HAPTICS_DELAY_MS    500 // To determine the time between proximity vibrations when close to the current stop.

enum NavigationState {
    E_NOT_STARTED,
    E_NAVIGATING,
    E_AT_CHECKPOINT,
    E_TRAIL_ENDED
};
static NavigationState navigationState = E_NOT_STARTED;


static TinyGPSPlus gps;
static double currentLat = 0;
static double currentLon = 0;
static bool navDataReceived = false;


void initNav();
void navUpdate();
void navFeedGPSData();
void navUpdateTrailStatusAndNavigate();
bool navReadGPS();
void navUpdateHaptics();
double getDistanceTo(double lat, double lon);
String getCardinalTo(double lat, double lon);
int getCourseTo(double lat, double lon);
int calculateRelativeDirection(int currentAngle, int targetAngle);

#ifdef DEBUG_NAVIGATION
bool navReadSerialGPS();
#endif // DEBUG_NAVIGATION

#endif // NAVIGATION_H