#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#include <stddef.h>
#include <stdint.h>
#include <deque>
#include <vector>

#define F(value) value

class Stream {
public:
    virtual ~Stream() {}
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual size_t write(const uint8_t *, size_t size) { return size; }

    template <typename T>
    size_t print(const T &) { return 0; }

    template <typename T>
    size_t println(const T &) { return 0; }

    size_t println() { return 0; }
};

class HardwareSerial : public Stream {
public:
    std::deque<uint8_t> testRx;
    std::vector<uint8_t> testTx;
    long testBaud = 0;
    bool testAutoComplete = false;
    uint8_t testFailCommand = 0;

    virtual void begin(long baud) { testBaud = baud; }
    int available() override { return (int)testRx.size(); }
    int read() override
    {
        if (testRx.empty()) return -1;
        uint8_t value = testRx.front();
        testRx.pop_front();
        return value;
    }
    size_t write(const uint8_t *data, size_t size) override
    {
        testTx.insert(testTx.end(), data, data + size);
        if (testAutoComplete && size >= 9 && data[3] == 2 && data[4] >= 0x63 && data[4] <= 0x66) {
            uint8_t reply[] = {0xDF,0x97,1,0x6F,data[4],2,0xFF,
                               (uint8_t)(data[4] == testFailCommand ? 1 : 0),0xFD,0,0};
            uint16_t sum = 0;
            for (unsigned i=0;i<9;i++) sum+=reply[i];
            reply[9]=(uint8_t)sum; reply[10]=(uint8_t)(sum>>8);
            testRx.insert(testRx.end(),reply,reply+sizeof(reply));
        }
        return size;
    }
    void testClear()
    {
        testRx.clear();
        testTx.clear();
    }
};

extern HardwareSerial Serial;

uint32_t millis();
void delay(unsigned long ms);

#endif
