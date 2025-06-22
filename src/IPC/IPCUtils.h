#pragma once

#include <mutex>
#include "GunstockConfigData.h" // Changed from PoseUpdateData.h

// Global mutex for synchronizing access to shared memory
extern std::mutex g_sharedMemoryMutex;

// Initializes the Inter-Process Communication (IPC) mechanism (e.g., shared memory)
void InitializeIPC();

// Cleans up the IPC mechanism
void CleanupIPC();

// Reads the latest config data from shared memory
GunstockConfigData ReadFromSharedMemory();

// Writes config data to shared memory
void WriteToSharedMemory(const GunstockConfigData& data);
