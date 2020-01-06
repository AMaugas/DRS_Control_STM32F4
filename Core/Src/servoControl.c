/*
 * servoControl.c
 *
 *  Created on: 19 déc. 2019
 *      Author: Antoine
 */

#include "servoControl.h"

void walk(HerkulexServo **articulations, uint8_t numberOfStep)
{
    WalkPhase phase = WalkPhase_None;
    uint8_t step = 0;

    while (step < numberOfStep)
    {
        switch (phase)
        {
        case WalkPhase_None:
            phase = WalkPhase_One;
            break;

        case WalkPhase_One:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 581, 0, HerkulexLed_Blue);
            setPosition(*(articulations + bassinGauche), 581, 0, HerkulexLed_Blue);
            setPosition(*(articulations + chevilleDroite), 450, 0, HerkulexLed_Blue);
            setTorqueOff(*(articulations + chevilleGauche));

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Blue);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Blue);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Blue);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Blue);
            HAL_Delay(2000);
            setTorqueOn(*(articulations + chevilleGauche));
            phase = WalkPhase_Two;
            break;

        case WalkPhase_Two:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + hancheGauche), 604, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + genouGauche), 427, 0, HerkulexLed_Cyan);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + bassinDroit), HerkulexLed_Cyan);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Cyan);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_Cyan);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Cyan);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_Cyan);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_Cyan);
            HAL_Delay(2000);
            phase = WalkPhase_Three;
            break;

        case WalkPhase_Three:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 442, 0, HerkulexLed_Purple);
            setPosition(*(articulations + bassinGauche), 442, 0, HerkulexLed_Purple);
            setPosition(*(articulations + chevilleGauche), 574, 0, HerkulexLed_Purple);
            setTorqueOff(*(articulations + chevilleDroite));

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Purple);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Purple);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Purple);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Purple);
            HAL_Delay(2000);
            setTorqueOn(*(articulations + chevilleDroite));
            phase = WalkPhase_Four;
            break;

        case WalkPhase_Four:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + hancheGauche), 512, 0, HerkulexLed_White);
            setPosition(*(articulations + genouGauche), 512, 0, HerkulexLed_White);
            setPosition(*(articulations + hancheDroite), 419, 0, HerkulexLed_White);
            setPosition(*(articulations + genouDroit), 596, 0, HerkulexLed_White);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_White);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_White);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_White);
            setLedColor(*(articulations + bassinDroit), HerkulexLed_White);
            HAL_Delay(2000);
            phase = WalkPhase_Five;
            break;

        case WalkPhase_Five:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 581, 0, HerkulexLed_Green);
            setPosition(*(articulations + bassinGauche), 581, 0, HerkulexLed_Green);
            setPosition(*(articulations + chevilleDroite), 450, 0, HerkulexLed_Green);
            setTorqueOff(*(articulations + chevilleGauche));

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Green);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Green);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Green);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Green);
            HAL_Delay(2000);
            setTorqueOn(*(articulations + chevilleGauche));
            phase = WalkPhase_Six;
            break;

        case WalkPhase_Six:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + hancheDroite), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + genouDroit), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + hancheGauche), 596, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + genouGauche), 419, 0, HerkulexLed_Yellow);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_Yellow);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_Yellow);
            setLedColor(*(articulations + bassinDroit), HerkulexLed_Yellow);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_Yellow);
            phase = WalkPhase_None;
            step++;
            break;

        default:
            break;
        }
    }
    initialisePosition(articulations);
}

void initialisePosition(HerkulexServo **articulations)
{
    HerkulexServoBus *bus = (**(articulations)).m_bus;
    prepareSynchronizedMove(bus, 100);

    setPosition(*(articulations + bassinDroit), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + bassinGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleDroite), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheDroite), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouDroit), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouGauche), 512, 0, HerkulexLed_Green);

    executeMove(bus);
    HAL_Delay(100);
}
