// NullStream.h
#ifndef NULL_STREAM_H
#define NULL_STREAM_H

#include <Arduino.h> // Includes Print.h implicitly
#include <Print.h>

// Declaration of the NullStream class
class NullStream : public Print {
public:
    // Method declarations (implementations are in the .cpp file)
    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL);
    void end();
    int available();
    int read();
    int peek();
    void flush();
    operator bool() const; // Allow checks like if(CONSOLE)
};

// Declaration of the global instance.
// 'extern' means it is DEFINED elsewhere (in NullStream.cpp)
extern NullStream NullSerial;

#endif // NULL_STREAM_H