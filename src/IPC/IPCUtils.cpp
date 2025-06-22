#include "../include/IPCUtils.h" // Adjusted path
#include <windows.h>
#include <string>
#include <iostream> // For placeholder messages, remove in actual implementation

// Define the global mutex
std::mutex g_sharedMemoryMutex;

// Global variables for shared memory handles and configuration
HANDLE hMapFile_ = nullptr;
LPVOID pBuf_ = nullptr;
const wchar_t* szName_ = L"PoseUpdateSharedMemory";
const DWORD dwSize_ = sizeof(PoseUpdateData);

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

    printf("IPC Initialized successfully. Shared memory created/opened.\n");
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

PoseUpdateData ReadFromSharedMemory() {
    PoseUpdateData data;
    data.shouldUpdate = false; // Default to no update

    std::lock_guard<std::mutex> lock(g_sharedMemoryMutex);

    if (pBuf_ != nullptr) {
        memcpy(&data, pBuf_, sizeof(PoseUpdateData));
    } else {
        fprintf(stderr, "Shared memory not mapped, cannot read.\n");
        // data.shouldUpdate remains false
    }

    return data;
}

void WriteToSharedMemory(const PoseUpdateData& data) {
    std::lock_guard<std::mutex> lock(g_sharedMemoryMutex);

    if (pBuf_ != nullptr) {
        memcpy(pBuf_, &data, sizeof(PoseUpdateData));
    } else {
        fprintf(stderr, "Shared memory not mapped, cannot write.\n");
    }
}
