/*
  EncMotControl.cpp - A DC motor (geared) position controller using
  only data from a MPU-6050 accelerometer as position feedback 
  Created by Tobias Kuhn. Sapporo, February 29, 2016.
  Released into the public domain.
*/

#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include "EncMotControl.h"
#include "Encoder.h"

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl CONSTRUCTOR - - -
// - - - - - - - - - - - - - - - - - - -
EncMotControl::EncMotControl(int in1Pin, int in2Pin, int pwmPin)
{
    _motorDriverPWMpin = pwmPin;
    _motorDriverIN1pin = in1Pin;
    _motorDriverIN2pin = in2Pin;
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl BEGIN - - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::begin(float pos1, float pos2, float pos3)
{
    _pos1 = pos1;
    _pos2 = pos2;
    _pos3 = pos3;
    pinMode(_motorDriverPWMpin, OUTPUT);
    pinMode(_motorDriverIN1pin, OUTPUT);
    pinMode(_motorDriverIN2pin, OUTPUT);
    _pathFollowing = false;
    pid.begin(0.8f, 0.00005f, 0.0f);
    pid.setSetPoint(0.0f);
    setMode(1);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl UPDATE  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::update(Encoder enc)
{
    _currentEncCount = enc.count;
    if(_pathFollowing){
        _followPath();
    }
    _updatePID();
}

// - - - - - - - - - - - - - - - - - - -
// - - EncMotControl UPDATE PID  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_updatePID()
{
    _moveFunctionCaller();
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(_getEncCountFloat(), _pathFollowing)));
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl SET ROT DIRECTION - -
// - - - - - - - - - - - - - - - - - - -
int EncMotControl::_setRotDir(int val)
{
    if(val == 0.0f){
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else if(val > 0.0f){
        digitalWrite(_motorDriverIN1pin, LOW);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else{
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, LOW);
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
// - - - - EncMotControl MOVE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::move(float goalPos, unsigned int moveTime, unsigned int moveSlopeTime)
{
    _pathFollowing = true;
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
        _pathFollowing = false; // in the END...
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

// - - - - - - - - - - - - - - - - - - -
// - - MOVE FUNCTION CALLER [DEBUG]  - -  
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_moveFunctionCaller()
{
    _cntr++;
    if(_cntr % 500 == 0){
        move(_pos1, 3000, 500);
    }
    if(_cntr % 1000 == 0){
        move(_pos2, 3000, 500);
    }
    if(_cntr % 1499 == 0 || _cntr >= 1500){
        move(_pos3, 3000, 500); 
        _cntr = 1;
    }
}

