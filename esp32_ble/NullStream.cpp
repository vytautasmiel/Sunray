// NullStream.cpp
#include "NullStream.h" // Include the header file we just created

// --- Method Definitions for NullStream ---

size_t NullStream::write(uint8_t c) {
    (void)c; // Indicate variable is intentionally unused
    return 1; // Must return 1 as per Print class specification
}

size_t NullStream::write(const uint8_t *buffer, size_t size) {
    (void)buffer; // Indicate variable is intentionally unused
    return size; // Pretend all bytes were written
}

// Define the dummy methods
void NullStream::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms) {
    // Do absolutely nothing
    (void)baud; (void)config; (void)rxPin; (void)txPin; (void)invert; (void)timeout_ms;
}

void NullStream::end() { /* Do nothing */ }
int NullStream::available() { return 0; }
int NullStream::read() { return -1; }
int NullStream::peek() { return -1; }
void NullStream::flush() { /* Do nothing */ }

// Define the boolean operator
NullStream::operator bool() const {
    return true; // Always pretend to be valid/initialized
}


// --- Definition of the global instance ---
// This line actually allocates memory for the object.
// It should only appear ONCE in the entire project (here in the .cpp file).
NullStream NullSerial;