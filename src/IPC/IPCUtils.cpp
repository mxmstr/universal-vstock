#include "IPCUtils.h"
#include <windows.h>
#include <string>
#include <iostream> // For placeholder messages, remove in actual implementation
#include "GunstockConfigData.h" // Make sure this is included if not pulled by IPCUtils.h

// Define the global mutex
std::mutex g_sharedMemoryMutex;

// Global variables for shared memory handles and configuration
HANDLE hMapFile_ = nullptr;
LPVOID pBuf_ = nullptr;
const wchar_t* szName_ = L"GunstockConfigSharedMemory"; // Changed name for clarity
const DWORD dwSize_ = sizeof(GunstockConfigData); // Changed to use GunstockConfigData

void InitializeIPC() {
    hMapFile_ = CreateFileMappingW(
        INVALID_HANDLE_VALUE,    // use paging file
        NULL,                    // default security
        PAGE_READWRITE,          // read/write access
        0,                       // maximum object size (high-order DWORD)
        dwSize_,                 // maximum object size (low-order DWORD)
        szName_);                // name of mapping object

    if (hMapFile_ == NULL) {
        fprintf(stderr, "CreateFileMapping failed with error: %d\n", GetLastError());
        return;
    }

    pBuf_ = MapViewOfFile(
        hMapFile_,   // handle to map object
        FILE_MAP_ALL_ACCESS, // read/write permission
        0,
        0,
        dwSize_);

    if (pBuf_ == NULL) {
        fprintf(stderr, "MapViewOfFile failed with error: %d\n", GetLastError());
        CloseHandle(hMapFile_);
        return;
    }

    printf("IPC Initialized successfully. Shared memory for GunstockConfig created/opened.\n");
}

void CleanupIPC() {
    bool errorOccurred = false;
    if (pBuf_ != nullptr) {
        if (!UnmapViewOfFile(pBuf_)) {
            fprintf(stderr, "UnmapViewOfFile failed with error: %d\n", GetLastError());
            errorOccurred = true;
        }
        pBuf_ = nullptr;
    }

    if (hMapFile_ != nullptr) {
        if (!CloseHandle(hMapFile_)) {
            fprintf(stderr, "CloseHandle failed with error: %d\n", GetLastError());
            errorOccurred = true;
        }
        hMapFile_ = nullptr;
    }

    if (!errorOccurred) {
        printf("IPC Cleaned up successfully.\n");
    }
}

GunstockConfigData ReadFromSharedMemory() {
    GunstockConfigData data;
    // Initialize with default values, especially config_updated to false
    memset(&data, 0, sizeof(GunstockConfigData)); // Zero out the struct
    data.config_updated = false;

    std::lock_guard<std::mutex> lock(g_sharedMemoryMutex);

    if (pBuf_ != nullptr) {
        memcpy(&data, pBuf_, sizeof(GunstockConfigData));
    } else {
        fprintf(stderr, "Shared memory not mapped, cannot read. Returning default config data.\n");
        // data.config_updated remains false
    }

    return data;
}

void WriteToSharedMemory(const GunstockConfigData& data) {
    std::lock_guard<std::mutex> lock(g_sharedMemoryMutex);

    if (pBuf_ != nullptr) {
        memcpy(pBuf_, &data, sizeof(GunstockConfigData));
    } else {
        fprintf(stderr, "Shared memory not mapped, cannot write.\n");
    }
}
