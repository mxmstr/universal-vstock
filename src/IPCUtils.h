#pragma once

#include <mutex>
#include "../include/PoseUpdateData.h" // Adjusted path if necessary

// Global mutex for synchronizing access to shared memory
extern std::mutex g_sharedMemoryMutex;

// Initializes the Inter-Process Communication (IPC) mechanism (e.g., shared memory)
void InitializeIPC();

// Cleans up the IPC mechanism
void CleanupIPC();

// Reads the latest pose data from shared memory
PoseUpdateData ReadFromSharedMemory();

// Writes pose data to shared memory
void WriteToSharedMemory(const PoseUpdateData& data);
