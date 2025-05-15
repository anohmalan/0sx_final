#include "Alarm.h"
#include <Arduino.h>

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float& distancePtr)
  : _rPin(rPin), _gPin(gPin), _bPin(bPin), _buzzerPin(buzzerPin), _distance(&distancePtr) {

  pinMode(_rPin, OUTPUT);
  pinMode(_gPin, OUTPUT);
  pinMode(_bPin, OUTPUT);
  pinMode(_buzzerPin, OUTPUT);

  _setRGB(0, 0, 0);
  noTone(_buzzerPin);
}

void Alarm::update() {
  _currentTime = millis();

  switch (_state) {
    case OFF:
      _offState();
      break;
    case WATCHING:
      _watchState();
      break;
    case ON:
      _onState();
      break;
    case TESTING:
      _testingState();
      break;
  }

  if (_turnOnFlag) {
    _state = ON;
    _turnOnFlag = false;
  } else if (_turnOffFlag) {
    _state = OFF;
    _turnOffFlag = false;
    _turnOff();
  }
}

void Alarm::setColourA(int r, int g, int b) {
  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {
  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}

void Alarm::setVariationTiming(unsigned long ms) {
  _variationRate = ms;
}

void Alarm::setDistance(float d) {
  _distanceTrigger = d;
}

void Alarm::getDistance() {
  Serial.print("Distance de déclenchement : ");
  Serial.println(_distanceTrigger);
}

void Alarm::setTimeout(unsigned long ms) {
  _timeoutDelay = ms;
}

void Alarm::turnOff() {
  _turnOffFlag = true;
}

void Alarm::turnOn() {
  _turnOnFlag = true;
}

void Alarm::test() {
  _state = TESTING;
  _testStartTime = millis();
}

AlarmState Alarm::getState() const {
  return _state;
}

// ---------------- MÉTHODES INTERNES ------------------

void Alarm::_setRGB(int r, int g, int b) {
  analogWrite(_rPin, r);
  analogWrite(_gPin, g);
  analogWrite(_bPin, b);
}

void Alarm::_turnOff() {
  _setRGB(0, 0, 0);
  noTone(_buzzerPin);
}

void Alarm::_offState() {
  _turnOff(); // On s’assure que tout est éteint
  if (*_distance <= _distanceTrigger) {
    _state = WATCHING;
    _lastDetectedTime = millis();
  }
}

void Alarm::_watchState() {
  if (*_distance <= _distanceTrigger) {
    _lastDetectedTime = millis();
  }

  if (_currentTime - _lastDetectedTime >= _timeoutDelay) {
    _state = OFF;
    _turnOff();
    return;
  }

  if (*_distance <= _distanceTrigger) {
    _state = ON;
  }
}

void Alarm::_onState() {
  // Gestion alternance des couleurs
  if (_currentTime - _lastUpdate >= _variationRate) {
    _currentColor = !_currentColor;

    if (_currentColor) {
      _setRGB(_colA[0], _colA[1], _colA[2]);
    } else {
      _setRGB(_colB[0], _colB[1], _colB[2]);
    }

    _lastUpdate = _currentTime;
  }

  // Sirène
  static int toneFreq = 400;
  static bool goingUp = true;
  static unsigned long lastToneTime = 0;
  const int toneStep = 20;
  const int minTone = 400;
  const int maxTone = 1000;
  const int toneInterval = 10;

  if (_currentTime - lastToneTime >= toneInterval) {
    tone(_buzzerPin, toneFreq);

    if (goingUp) {
      toneFreq += toneStep;
      if (toneFreq >= maxTone) goingUp = false;
    } else {
      toneFreq -= toneStep;
      if (toneFreq <= minTone) goingUp = true;
    }

    lastToneTime = _currentTime;
  }

  if (*_distance > _distanceTrigger) {
    _state = WATCHING;
    _lastDetectedTime = millis();
  }
}

void Alarm::_testingState() {
  if (_currentTime - _testStartTime >= 3000) {
    _state = OFF;
    _turnOff();
    return;
  }

  if (_currentTime - _lastUpdate >= _variationRate) {
    _currentColor = !_currentColor;

    if (_currentColor) {
      _setRGB(_colA[0], _colA[1], _colA[2]);
    } else {
      _setRGB(_colB[0], _colB[1], _colB[2]);
    }
    _lastUpdate = _currentTime;
  }

  tone(_buzzerPin, 800);
}
