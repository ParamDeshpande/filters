#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <iostream> 
#include <stddef.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

//Do not change instead UPscale the values
#define delT 1

#define alpha 0.9
#define beta 0.095
#define gamma 0.01

#define posVarSq 5e3
#define velVarSq 5e3
#define accVarSq 5e3
#define R 20000UL // Measurement noise co-variance

//Just the initial matrix will get updated each iteration
#define estVar_Pos  100
#define estCovar_Pos_Vel  91
#define estVar_Vel 100
#define estCovar_Vel_Acc  71
#define estVar_Acc  100
#define estCovar_Pos_Acc  51

#endif
//End of file