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
            (**(articulations + 0)).m_bus->prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 581, 0, HerkulexLed_Blue);
            setPosition(*(articulations + bassinGauche), 581, 0, HerkulexLed_Blue);
            setPosition(*(articulations + chevilleDroite), 450, 0, HerkulexLed_Blue);
            setPosition(*(articulations + chevilleGauche), 450, 0, HerkulexLed_Blue);
            setPosition(*(articulations + hancheDroite), 450, 0, HerkulexLed_Blue);
            setPosition(*(articulations + hancheGauche), 512, 0, HerkulexLed_Blue);
            setPosition(*(articulations + genouDroit), 443, 0, HerkulexLed_Blue);
            setPosition(*(articulations + genouGauche), 512, 0, HerkulexLed_Blue);

            (**(articulations + 0)).m_bus->executeMove((**(articulations + 0)).m_bus);
            HAL_Delay(100);
            phase = WalkPhase_Two;
            break;

        case WalkPhase_Two:
            (**(articulations + 0)).m_bus->prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 512, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + bassinGauche), 512, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + chevilleDroite), 527, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + chevilleGauche), 512, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + hancheDroite), 512, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + hancheGauche), 651, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + genouDroit), 512, 0, HerkulexLed_Cyan);
            setPosition(*(articulations + genouGauche), 266, 0, HerkulexLed_Cyan);

            (**(articulations + 0)).m_bus->executeMove((**(articulations + 0)).m_bus);
            HAL_Delay(100);

            phase = WalkPhase_Three;
            break;

        case WalkPhase_Three:
            (**(articulations + 0)).m_bus->prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 512, 0, HerkulexLed_Purple);
            setPosition(*(articulations + bassinGauche), 512, 0, HerkulexLed_Purple);
            setPosition(*(articulations + chevilleDroite), 512, 0, HerkulexLed_Purple);
            setPosition(*(articulations + chevilleGauche), 512, 0, HerkulexLed_Purple);
            setPosition(*(articulations + hancheDroite), 551, 0, HerkulexLed_Purple);
            setPosition(*(articulations + hancheGauche), 574, 0, HerkulexLed_Purple);
            setPosition(*(articulations + genouDroit), 512, 0, HerkulexLed_Purple);
            setPosition(*(articulations + genouGauche), 443, 0, HerkulexLed_Purple);

            (**(articulations + 0)).m_bus->executeMove((**(articulations + 0)).m_bus);
            HAL_Delay(100);
            phase = WalkPhase_Four;
            break;

        case WalkPhase_Four:
            (**(articulations + 0)).m_bus->prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

            setPosition(*(articulations + bassinDroit), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + bassinGauche), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + chevilleDroite), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + chevilleGauche), 497, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + hancheDroite), 651, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + hancheGauche), 512, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + genouDroit), 266, 0, HerkulexLed_Yellow);
            setPosition(*(articulations + genouGauche), 512, 0, HerkulexLed_Yellow);

            (**(articulations + 0)).m_bus->executeMove((**(articulations + 0)).m_bus);
            HAL_Delay(100);

            phase = WalkPhase_None;
            step++;
            break;

        default:
            break;
        }
    }
}

void initialisePosition(HerkulexServo **articulations)
{
    (**(articulations + 0)).m_bus->prepareSynchronizedMove((**(articulations + 0)).m_bus, 100);

    setPosition(*(articulations + bassinDroit), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + bassinGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleDroite), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + chevilleGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheDroite), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + hancheGauche), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouDroit), 512, 0, HerkulexLed_Green);
    setPosition(*(articulations + genouGauche), 512, 0, HerkulexLed_Green);

    (**(articulations + 0)).m_bus->executeMove((**(articulations + 0)).m_bus);
    HAL_Delay(100);
}
