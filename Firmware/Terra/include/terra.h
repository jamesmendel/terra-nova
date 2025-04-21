/**
 * @file terra.h
 * @brief Entrypoint for Terra
 * @date 2025-04-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef TERRA_H
#define TERRA_H

// #define TERRA_DISABLE_IDLE_SHUTDOWN

#ifndef  TERRA_DISABLE_IDLE_SHUTDOWN
#define TERRA_IDLE_SHUTDOWN_SEC 360 // time in seconds until Terra shuts down from no motion
#else
#define TERRA_IDLE_SHUTDOWN_SEC 0   // auto-shutdown disabled
#endif    // TERRA_DISABLE_IDLE_SHUTDOWN

// Function Prototypes
void setup();
void loop();

#endif // TERRA_H