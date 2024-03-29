#include "Arduino.h"
#include "timer.h"

byte Timer::poll(word ms) {
    byte ready = 0;
    if (armed) {
        word remain = next - millis();
        // since remain is unsigned, it will overflow to large values when
        // the timeout is reached, so this test works as long as poll() is
        // called no later than 5535 millisecs after the timer has expired
        if (remain <= 60000)
            return 0;         // not expired yet
        // return a value between 1 and 255, being msecs+1 past expiration
        // note: the actual return value is only reliable if poll() is
        // called no later than 255 millisecs after the timer has expired
        remain = ~remain;
        ready = remain > 255 ? 255 : 1; // avoid returning 0 on truncating n*256

        //ready = -remain;
    }
    set(ms);
    return ready;
}

word Timer::remaining() const {
    word remain = armed ? next - millis() : 0;
    return remain <= 60000 ? remain : 0;
}

void Timer::set(word ms) {
    armed = ms != 0;
    if (armed)
        next = millis() + ms - 1;
}
