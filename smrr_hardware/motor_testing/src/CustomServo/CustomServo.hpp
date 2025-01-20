#include <Servo.h>

class CustomServo : public Servo
{
private:
    /* data */
    int period = 30;
    int step = 1;
    unsigned long lastMoveMillis = 0;
    int currPos = 90;

public:
    CustomServo();
    ~CustomServo();
    void init(int pin, int pos = 90,int period = 30, int step = 1);
    bool move(int pos);
    bool move(int pos, int period);
    bool move(int pos, int period, int step);
};

CustomServo::CustomServo()
{
}

CustomServo::~CustomServo()
{
}

void CustomServo::init(int pin,  int pos,int period,int step)
{
    this->period = period;
    this->currPos = pos;
    Servo::attach(pin);
}
bool CustomServo::move(int pos)
{
    return move(pos, this->period, this->step);
}

bool CustomServo::move(int pos,int period)
{
    return move(pos, period, this->step);
}

bool CustomServo::move(int pos,int period,int step)
{
    unsigned long currentMoveMillis = millis();
    if ((currentMoveMillis - lastMoveMillis) < period)
        return pos != currPos;

    lastMoveMillis = currentMoveMillis;

    int diff;

    if (currPos > pos)
    {
        diff = currPos - pos;
        if (diff < step)
            step = diff;            
        currPos-=step;
        Servo::write(currPos);
        return true;
    }
    else if (currPos < pos)
    {
        diff = pos - currPos;
        if (diff < step)
            step = diff;
        currPos += step;
        Servo::write(currPos);
        return true;
    }
    else
    {
        return false;
    }
}
