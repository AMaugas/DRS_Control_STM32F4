/*
 * servoControl.c
 *
 *  Created on: 19 déc. 2019
 *      Author: Antoine
 */

#include "servoControl.h"

void walk(HerkulexServo **articulations, uint8_t numberOfStep)
{
    uint16_t walkTime = 1120;
    WalkPhase phase = WalkPhase_None;
    uint8_t step = 0;

    while (step < numberOfStep)
    {
        switch (phase)
        {
        case WalkPhase_None:
            if (step == 0)
            {
                phase = WalkPhase_One;
            }
            else
            {
                phase = WalkPhase_Three;
            }
            break;

        case WalkPhase_One:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + bassinDroit), 22.5f, 0, HerkulexLed_Blue);
            setPosition(*(articulations + bassinGauche), 22.5f, 0, HerkulexLed_Blue);
            setPosition(*(articulations + chevilleDroite), -20.0f, 0, HerkulexLed_Blue);
            setPosition(*(articulations + chevilleGauche), -20.0f, 0, HerkulexLed_Blue);
            // setTorqueOff(*(articulations + chevilleGauche));

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Blue);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Blue);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Blue);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Blue);
            HAL_Delay(walkTime);
            setTorqueOn(*(articulations + chevilleGauche));
            phase = WalkPhase_Two;
            break;

        case WalkPhase_Two:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + hancheGauche), 30.0f, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + genouGauche), -30.0f, 0, HerkulexLed_Cyan);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + bassinDroit), HerkulexLed_Cyan);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_Cyan);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_Cyan);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_Cyan);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Cyan);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Cyan);
            HAL_Delay(walkTime);
            phase = WalkPhase_Three;
            break;

        case WalkPhase_Three:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + bassinDroit), -22.5f, 0, HerkulexLed_Purple);
            setPosition(*(articulations + bassinGauche), -22.5f, 0, HerkulexLed_Purple);
            // setTorqueOff(*(articulations + chevilleDroite));
            setPosition(*(articulations + chevilleGauche), 20.0f, 0, HerkulexLed_Purple);
            setPosition(*(articulations + chevilleDroite), 20.0f, 0, HerkulexLed_Purple);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Purple);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Purple);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Purple);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Purple);
            HAL_Delay(walkTime);
            setTorqueOn(*(articulations + chevilleDroite));
            phase = WalkPhase_Four;
            break;

        case WalkPhase_Four:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + genouDroit), 27.5f, 0, HerkulexLed_White);
            setPosition(*(articulations + genouGauche), 0, 0, HerkulexLed_White);
            setPosition(*(articulations + hancheDroite), -30.0f, 0, HerkulexLed_White);
            setPosition(*(articulations + hancheGauche), 0, 0, HerkulexLed_White);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + bassinDroit), HerkulexLed_White);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_White);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_White);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_White);
            HAL_Delay(walkTime);
            phase = WalkPhase_Five;
            break;

        case WalkPhase_Five:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + bassinDroit), 22.5, 0, HerkulexLed_Green);
            setPosition(*(articulations + bassinGauche), 22.5, 0, HerkulexLed_Green);
            setPosition(*(articulations + chevilleDroite), -20.0f, 0, HerkulexLed_Green);
            // setTorqueOff(*(articulations + chevilleGauche));
            setPosition(*(articulations + chevilleGauche), -20.0f, 0, HerkulexLed_Purple);

            executeMove((**(articulations + 0)).m_bus);
            setLedColor(*(articulations + hancheDroite), HerkulexLed_Green);
            setLedColor(*(articulations + genouGauche), HerkulexLed_Green);
            setLedColor(*(articulations + genouDroit), HerkulexLed_Green);
            setLedColor(*(articulations + hancheGauche), HerkulexLed_Green);
            HAL_Delay(walkTime);
            setTorqueOn(*(articulations + chevilleGauche));
            phase = WalkPhase_Six;
            break;

        case WalkPhase_Six:
            prepareSynchronizedMove((**(articulations + 0)).m_bus, walkTime);

            setPosition(*(articulations + genouDroit), 0, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + genouGauche), -30.0f, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + hancheDroite), 0, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + hancheGauche), 30.0f, 0, HerkulexLed_Yellow);

            executeMove((**(articulations + 0)).m_bus);

            setLedColor(*(articulations + bassinDroit), HerkulexLed_Yellow);
            setLedColor(*(articulations + bassinGauche), HerkulexLed_Yellow);
            setLedColor(*(articulations + chevilleDroite), HerkulexLed_Yellow);
            setLedColor(*(articulations + chevilleGauche), HerkulexLed_Yellow);
            HAL_Delay(walkTime);
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
    prepareSynchronizedMove(bus, 1120);

    setPosition(*(articulations + bassinDroit), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + bassinGauche), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleDroite), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleGauche), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheDroite), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheGauche), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouDroit), 0, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouGauche), 0, 0, HerkulexLed_Green);

    executeMove(bus);
    HAL_Delay(100);
}
