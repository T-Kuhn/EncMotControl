/*
  EncMotControl.cpp - A DC motor (geared) position controller 
  Created by Tobias Kuhn. Sapporo, February 29, 2016.
  Released into the public domain.
*/

#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "EncMotControl.h"
#include "Encoder.h"
#include "MPU6050.h"

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl CONSTRUCTOR - - -
// - - - - - - - - - - - - - - - - - - -
EncMotControl::EncMotControl(int in1Pin, int in2Pin, int pwmPin, int mpuAddPin)
{
    _mpuAddPin = mpuAddPin;
    _motorDriverPWMpin = pwmPin;
    _motorDriverIN1pin = in1Pin;
    _motorDriverIN2pin = in2Pin;
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl BEGIN - - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::begin()
{
    pinMode(_motorDriverPWMpin, OUTPUT);
    pinMode(_motorDriverIN1pin, OUTPUT);
    pinMode(_motorDriverIN2pin, OUTPUT);
    pinMode(_mpuAddPin, OUTPUT);
    pathFollowing = false;
    pid.begin(1.2f, 0.000f, 0.0f, 3.0f);
    pid.setSetPoint(0.0f);
    setMode(1);
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl INIT 1  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::initStep1()
{
    digitalWrite(_mpuAddPin, HIGH);
    delay(5);
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69);
    mpu.setDLPFMode(MPU6050_DLPF_4);   
    pid.begin(5.2f, 0.04f, 0.0f, 0.01f);
    pathFollowing = true;
    pid.setSetPoint(4.71f);
    _updatePIDinit();
    digitalWrite(_mpuAddPin, LOW);
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl INIT 2  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::initStep2()
{
    pathFollowing = false;
    pid.setSetPoint(0.0f);
    analogWrite(_motorDriverPWMpin, 0);
    digitalWrite(_motorDriverIN1pin, HIGH);
    digitalWrite(_motorDriverIN2pin, HIGH);
    pid.begin(0.9f, 0.01f, 0.00f, 3.0f);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl UPDATE  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::update(Encoder enc)
{
    _currentEncCount = enc.count;
    if(pathFollowing){
        _followPath();
    }
    if(_currentEncCount == _goalPos){
        pathFollowing = false;
    }
    _updatePID();
}

// - - - - - - - - - - - - - - - - - - -
// - - EncMotControl UPDATE PID  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_updatePID()
{
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(_getEncCountFloat(), pathFollowing)));
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl UPDATE PID INIT - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_updatePIDinit()
{
    analogWrite(_motorDriverPWMpin, _setRotDirInit(pid.update(_getRotAngle(), pathFollowing)));
}

// - - - - - - - - - - - - - - - - - - -
// EncMotControl UPDATE PID EXTERNAL PATH
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::updatePID_ext(float setPoint, Encoder enc)
{
    int _dif;
    //find out in which direction the motor is moving!
    _dif = enc.count - _enc_lastCount;
    if(_dif > 0){
        //moveing in the positive direction!
        _directionOfMovement = 1;
    }else if (_dif < 0){
        //moving towards the negative
        _directionOfMovement = 2;
    }else{
        //not moving at all!
        _directionOfMovement = 3;
    }
    _enc_lastCount = enc.count;
    Serial.println(_dif);
    Serial.println(_directionOfMovement);

    pid.setSetPoint(setPoint);
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update((float)enc.count, true)));
}

// - - - - - - - - - - - - - - - - - - -
// EncMotControl SET ROT DIRECTION INIT
// - - - - - - - - - - - - - - - - - - -
int EncMotControl::_setRotDirInit(int val)
{
    if(val == 0.0f){
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else if(val > 0.0f){
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, LOW);
    }else{
        digitalWrite(_motorDriverIN1pin, LOW);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }
    return abs(val); 
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl SET ROT DIRECTION - -
// - - - - - - - - - - - - - - - - - - -
int EncMotControl::_setRotDir(int val)
{
    if(val == 0.0f){
        digitalWrite(_motorDriverIN1pin, LOW);
        digitalWrite(_motorDriverIN2pin, LOW);
    }else if(val > 0.0f){
        //PID wants to move to the positive...
        if(_directionOfMovement == 2){
            //...but the Motor is Moving towards the negative
            // -> just put on the breaks.
            digitalWrite(_motorDriverIN1pin, LOW);
            digitalWrite(_motorDriverIN2pin, LOW);
            return 0.0f;
        }else{
            digitalWrite(_motorDriverIN1pin, LOW);
            digitalWrite(_motorDriverIN2pin, HIGH);
        }
    }else{
        //PID wants to move to the negative...
        if(_directionOfMovement == 1){
            //...but the Motor is Moving towards the positive
            // -> just put on the breaks.
            digitalWrite(_motorDriverIN1pin, LOW);
            digitalWrite(_motorDriverIN2pin, LOW);
            return 0.0f;
        }else{
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, LOW);
        }
    }
    return abs(val); 
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl GET ROTATION COUNT  -
// - - - - - - - - - - - - - - - - - - -
float EncMotControl::_getEncCountFloat()
{
    return (float)_currentEncCount;
    
    
}

// - - - - - - - - - - - - - - - - - - -
//  AccMotControl GET ROTATIONAL ANGLE -
// - - - - - - - - - - - - - - - - - - -
float EncMotControl::_getRotAngle()
{
    Vector _tmp;
    float _ZAxis;
    float _YAxis;
    float _aTan;
    _tmp = mpu.readRawAccel();
    _ZAxis = _tmp.ZAxis;
    _YAxis = _tmp.YAxis;
    if(_YAxis == 0.0f){
        _YAxis = 0.000000001f;
    }
    _aTan = (float)atan(_ZAxis / _YAxis);
    if(_YAxis >= 0.0f){
        return _aTan + PI; 
    }else if(_ZAxis < 0.0f){
        return _aTan;
    }else if(_ZAxis >= 0.0f){
        return _aTan + 2*PI;
    }
    return 0.0f;
}

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl MOVE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::move(float goalPos, unsigned int moveTime, unsigned int moveSlopeTime)
{
    pathFollowing = true;
    _goalPos = goalPos;
    _moveTime = moveTime;
    _moveSlopeTime = moveSlopeTime;
    _startPos = _getEncCountFloat();
    _startTime = millis();
    if(_mode == 1){
        _calculatePathVars1();
    }else{
        //sinusoidal ramps. Not yet implemented
    }
}

// - - - - - - - - - - - - - - - - - - - - - 
// - EncMotControl CALCULATE PATHVARS  1 - -
// - - - - - - - - - - - - - - - - - - - - -
void EncMotControl::_calculatePathVars1()
{
    _distance = _goalPos - _startPos; // can be minus
    if(2 * _moveSlopeTime < _moveTime){
        _moveStraightTime = _moveTime - 2 * _moveSlopeTime;  
    }else{
        _moveStraightTime = 0;
    }
    _maxSpeed = _distance / (_moveSlopeTime + _moveStraightTime);
    _slope = _maxSpeed / _moveSlopeTime;
    
    _startSlopeEndPos = _maxSpeed / 2 * _moveSlopeTime;
    _straightMoveEndPos = _startSlopeEndPos + _maxSpeed * _moveStraightTime;
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl FOLLOW PATH - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_followPath()
{
    _currentTime = millis();
    _passedTime = _currentTime - _startTime;
    if(_moveSlopeTime >= _passedTime){
        _followStartSlope();
        return;
    }else if(_moveSlopeTime + _moveStraightTime >= _passedTime){
        _followStraightLine();
        return;
    }else if(2 * _moveSlopeTime + _moveStraightTime >= _passedTime){
        _followEndSlope();
        return;
    }else{
        pid.setSetPoint(_goalPos);
        //pathFollowing = false; // in the END...
    }
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl FOLLOW START SLOPE  -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_followStartSlope()
{
    if(_mode == 1){
        pid.setSetPoint(_startPos + _passedTime * _passedTime * _slope / 2);
    }else{
        pid.setSetPoint(_startPos + ( sin(-PI/2 + (float)_passedTime / _moveSlopeTime * PI) 
                + 1 ) / 4 * _slope * _passedTime * _passedTime);
    }
}

// - - - - - - - - - - - - - - - - - - -
//  EncMotControl FOLLOW STRAIGHT LINE -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_followStraightLine()
{
    pid.setSetPoint(_startPos + _startSlopeEndPos + (_passedTime - _moveSlopeTime) * _maxSpeed);    
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl FOLLOW END SLOPE  - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_followEndSlope()
{
    unsigned int passedTimeSinceStartEndSlope = _passedTime - _moveSlopeTime - _moveStraightTime;
    if(_mode == 1){
        pid.setSetPoint(_startPos + _straightMoveEndPos
                + (_maxSpeed - _slope / 2 * passedTimeSinceStartEndSlope) 
                * passedTimeSinceStartEndSlope); 
    }else{
        pid.setSetPoint(_startPos + _straightMoveEndPos
                + (_maxSpeed - (sin( -PI/2 + (float)passedTimeSinceStartEndSlope / _moveSlopeTime * PI) 
                + 1 ) / 2 * _slope / 2 * passedTimeSinceStartEndSlope)
                * passedTimeSinceStartEndSlope);
    }
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl SET MODE  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::setMode(int nmbr)
{
    // nmbr ->  1:  linear speedgain
    //          2:  S-curveed speedgain
    _mode = nmbr;
}


