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
EncMotControl::EncMotControl(int in1Pin, int in2Pin, int pwmPin, int mpuAddPin, double calib, bool debug)
{
    _mpuAddPin = mpuAddPin;
    _calib = calib;
    _motorDriverPWMpin = pwmPin;
    _motorDriverIN1pin = in1Pin;
    _motorDriverIN2pin = in2Pin;
    _debug = debug;
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
    pid.begin(0.2, 0.000, 0.0, 0.5, 0.1, 0, false);
    pid.setSetPoint(0.0);
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
    mpu.setDLPFMode(MPU6050_DLPF_2);   
    pathFollowing = true;
    pid.begin(20.0, 0.000, 0.0, 0.01, 0.1, 0, false);
    pid.setSetPoint(_calib);
    digitalWrite(_mpuAddPin, LOW);
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl INIT 1  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::initStep1_5()
{
    digitalWrite(_mpuAddPin, HIGH);
    delay(5);
    if(_getRotAngle() > _calib - 0.002 && _getRotAngle() < _calib + 0.002){
        pathFollowing = false;
    }
    _updatePIDinit();
    digitalWrite(_mpuAddPin, LOW);
}

// - - - - - - - - - - - - - - - - - - -
// - - - EncMotControl INIT 2  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::initStep2()
{
    pathFollowing = false;
    pid.setSetPoint(0.0);
    analogWrite(_motorDriverPWMpin, 0);
    digitalWrite(_motorDriverIN1pin, HIGH);
    digitalWrite(_motorDriverIN2pin, HIGH);
    pid.begin(2.5, 0.00001, 00.0, 1.5, 1.0, 0, _debug);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl UPDATE  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::update(Encoder enc)
{
    _currentEncCount = enc.count;
    if(_currentEncCount == _goalPos){
        pathFollowing = false;
    }
    if(pathFollowing){
        _followPath();
    }
    _updatePID();
}

// - - - - - - - - - - - - - - - - - - -
// - - EncMotControl UPDATE PID  - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_updatePID()
{
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update(_getEncCountDouble(), pathFollowing)));
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl UPDATE PID INIT - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::_updatePIDinit()
{
    if(_debug){
        Serial.println(_getRotAngle());
    }
    analogWrite(_motorDriverPWMpin, _setRotDirInit(pid.update(_getRotAngle(), pathFollowing)));
}

// - - - - - - - - - - - - - - - - - - -
// EncMotControl UPDATE PID EXTERNAL PATH
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::updatePID_ext(double setPoint, Encoder enc)
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

    pid.setSetPoint(setPoint);
    analogWrite(_motorDriverPWMpin, _setRotDir(pid.update((double)enc.count, true)));
}

// - - - - - - - - - - - - - - - - - - -
// EncMotControl SET ROT DIRECTION INIT
// - - - - - - - - - - - - - - - - - - -
int EncMotControl::_setRotDirInit(int val)
{
    if(val == 0){
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else if(val > 0){
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
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else if(val > 0.0f){
        digitalWrite(_motorDriverIN1pin, LOW);
        digitalWrite(_motorDriverIN2pin, HIGH);
    }else{
        digitalWrite(_motorDriverIN1pin, HIGH);
        digitalWrite(_motorDriverIN2pin, LOW);
    }
    if(abs(val) == 255){        //BUG fix for analogWrite functions! 255 isn't allowed!
        val = 256;
    }
    if(_debug){
        //Serial.println(abs(val));
    }
   return abs(val); 
}

// - - - - - - - - - - - - - - - - - - -
// - EncMotControl GET ROTATION COUNT  -
// - - - - - - - - - - - - - - - - - - -
double EncMotControl::_getEncCountDouble()
{
    return (double)_currentEncCount;
    
    
}

// - - - - - - - - - - - - - - - - - - -
//  AccMotControl GET ROTATIONAL ANGLE -
// - - - - - - - - - - - - - - - - - - -
double EncMotControl::_getRotAngle()
{
    Vector _tmp;
    double _ZAxis;
    double _YAxis;
    double _aTan;
    _tmp = mpu.readRawAccel();
    _ZAxis = _tmp.ZAxis;
    _YAxis = _tmp.YAxis;
    if(_YAxis == 0.0){
        _YAxis = 0.000000001;
    }
    _aTan = atan(_ZAxis / _YAxis);
    if(_YAxis >= 0.0){
        return _aTan + PI; 
    }else if(_ZAxis < 0.0){
        return _aTan;
    }else if(_ZAxis >= 0.0){
        return _aTan + 2*PI;
    }
    return 0.0;
}

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl MOVE  - - - - -
// - - - - - - - - - - - - - - - - - - -
void EncMotControl::move(int goalPos, unsigned int moveTime, unsigned int moveSlopeTime)
{
    pathFollowing = true;
    _goalPos = (double)goalPos;
    _moveTime = moveTime;
    _moveSlopeTime = moveSlopeTime;
    _startPos = _getEncCountDouble();
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
        pid.setSetPoint(_startPos + ( sin(-PI/2 + (double)_passedTime / _moveSlopeTime * PI) 
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
                + (_maxSpeed - (sin( -PI/2 + (double)passedTimeSinceStartEndSlope / _moveSlopeTime * PI) 
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


