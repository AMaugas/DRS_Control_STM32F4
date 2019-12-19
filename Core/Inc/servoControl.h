/*
 * servoControll.h
 *
 *  Created on: 19 déc. 2019
 *      Author: Antoine
 */

#ifndef INC_SERVOCONTROL_H_
#define INC_SERVOCONTROL_H_

#include <stdint.h>
#include "HerkulexServo.h"

HerkulexServo *articulations[8];

typedef enum _articulation
{
    chevilleDroite,
    genouDroit,
    hancheDroite,
    bassinDroit,
    bassinGauche,
    hancheGauche,
    genouGauche,
    chevilleGauche
} Articulation;

typedef enum _walkPhase
{
    WalkPhase_None,
    WalkPhase_One,
    WalkPhase_Two,
    WalkPhase_Three,
    WalkPhase_Four
} WalkPhase;

void walk(HerkulexServo **articulations, uint8_t numberOfStep);
void initialisePosition(HerkulexServo **articulations);

#endif /* INC_SERVOCONTROL_H_ */
