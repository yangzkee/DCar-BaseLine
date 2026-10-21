#include <assert.h>
#include <stdio.h>
#include <vector>
#include "Arduino.h"
#ifdef SINGLE_FILE
#include "../../DFCom_Arduino/SingleFile/DcarON_Stadium_AllInOne/DcarON_Stadium_AllInOne.ino"
#else
#include "../../DFCom_Arduino/DFCom/examples/DcarON_Stadium/DcarON_Stadium.ino"
#endif
HardwareSerial Serial;
static uint32_t nowMs;
uint32_t millis() { return nowMs++; }
void delay(unsigned long ms) { nowMs += ms; }
static std::vector<uint8_t> telemetry(uint8_t cmd, uint8_t len, uint8_t ver) {
    std::vector<uint8_t> f = {0xDF,0x97,1,0x6C,cmd,len};
    f.resize(6+len); f[6]=ver; f.push_back(0xFD);
    uint16_t sum=0; for (auto b:f) sum+=b;
    f.push_back((uint8_t)sum); f.push_back((uint8_t)(sum>>8)); return f;
}
static void inject(const std::vector<uint8_t> &f) { Serial.testRx.insert(Serial.testRx.end(),f.begin(),f.end()); }
int main() {
    setup();
    assert(nowMs >= 4570 && !DFCom.state.received); // No telemetry/state gate.
    // Corrupt length before bytewise Odom; parser recovers.
    inject({0xDF,0x97,1,0x6C,0x80,60});
    for (auto b:telemetry(0x80,51,4)) { inject({b}); DFCom.update(); }
    for (auto b:telemetry(0x81,35,2)) { inject({b}); DFCom.update(); }
    assert(DFCom.odom.frameCount==1);
    inject(telemetry(0x80,1,4)); DFCom.update();
    assert(DFCom.odomDiag.wire==2 && DFCom.odomDiag.rejected==1);
    assert(DFCom.odomAvailable());
    nowMs+=2000;
    assert(!DFCom.odomAvailable());
    Serial.testClear(); Serial.testAutoComplete=true;
    for (unsigned n=0;n<3;n++) loop();
    assert(completedLaps==3 && !testStopped);
    unsigned pos=0, segment=0;
    while (pos<Serial.testTx.size()) {
        const uint8_t *f=&Serial.testTx[pos];
        assert(f[4] == (segment%4==0 || segment%4==2 ? 0x66 : segment%4==1 ? 0x64 : 0x65));
        if (f[4]==0x66) assert(f[17]&0x80); // Signed reverse speed in s32 payload.
        pos+=f[5]+9;segment++;
    }
    assert(segment==12);
    Serial.testFailCommand=0x66; loop();
    assert(testStopped && completedLaps==3);
    size_t before=Serial.testTx.size(); loop(); assert(before==Serial.testTx.size());
    puts("Arduino Odom telemetry, resync, independent motion, three stadium laps and failure stop passed");
}
