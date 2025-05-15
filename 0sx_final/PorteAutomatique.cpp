// Fichier : PorteAutomatique.cpp

#include "PorteAutomatique.h"

// Constructeur
PorteAutomatique::PorteAutomatique(int p1, int p2, int p3, int p4, float& distancePtr)
    : _stepper(AccelStepper::FULL4WIRE, p1, p3, p2, p4), _distance(distancePtr) {
    _stepper.setMaxSpeed(1000);
    _stepper.setAcceleration(500);
    _stepper.setCurrentPosition(0); 
}
// Méthodes publiques
void PorteAutomatique::update() {
    _mettreAJourEtat();
    _stepper.run();
}
void PorteAutomatique::setAngleOuvert(float angle) {
    _angleOuvert = angle;
}
void PorteAutomatique::setAngleFerme(float angle) {
    _angleFerme = angle;
}
void PorteAutomatique::setPasParTour(int steps) {
    _stepsPerRev = steps;
}
void PorteAutomatique::setDistanceOuverture(float distance) {
    _distanceOuverture = distance;
}
void PorteAutomatique::setDistanceFermeture(float distance) {
    _distanceFermeture = distance;
}
const char* PorteAutomatique::getEtatTexte() const {
    switch (_etat) {
        case FERMEE: return "FERMEE    ";
        case OUVERTE: return "OUVERTE   ";
        case EN_FERMETURE: return "EN_FERMETURE  ";
        case EN_OUVERTURE: return "EN_OUVERTURE   ";
        default: return "INCONNU";
    }
}
float PorteAutomatique::getAngle() const {
    long position = _stepper.currentPosition();
    return position * 360.0 / _stepsPerRev;
}
// Méthodes privées
void PorteAutomatique::_mettreAJourEtat() {
    switch (_etat) {
        case FERMEE:
            _fermeState();
            break;
        case OUVERTE:
            _ouvertState();
            break;
        case EN_FERMETURE:
            _fermetureState();
            break;
        case EN_OUVERTURE:
            _ouvertureState();
            break;
    }
}
void PorteAutomatique::_ouvertState() {
    if (_distance > _distanceFermeture) {
        _etat = EN_FERMETURE;
        _fermer();
    }
}
void PorteAutomatique::_fermeState() {
    if (_distance < _distanceOuverture) {
        _etat = EN_OUVERTURE;
        _ouvrir();
    }
}
void PorteAutomatique::_ouvertureState() {
    if (!_stepper.isRunning()) {
        _etat = OUVERTE;
    }
}
void PorteAutomatique::_fermetureState() {
    if (!_stepper.isRunning()) {
        _etat = FERMEE;
    }
}
void PorteAutomatique::_ouvrir() {
    long target = _angleEnSteps(_angleOuvert);
    _stepper.moveTo(target);
}
void PorteAutomatique::_fermer() {
    long target = _angleEnSteps(_angleFerme);
    _stepper.moveTo(target);
}
long PorteAutomatique::_angleEnSteps(float angle) const {
    return static_cast<long>((angle / 360.0) * _stepsPerRev);
}

void PorteAutomatique::activer() {
    _etat = FERMEE;
    _stepper.enableOutputs();

}
void PorteAutomatique::desactiver() {
    _stepper.disableOutputs();
}