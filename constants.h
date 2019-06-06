#include <iostream> 
#include <stddef.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define alpha 0.9
#define beta 0.095
#define gamma 0.01
#define delT 1

#define posVarSq 1e2
#define velVarSq 1e2
#define accVarSq 1e2
#define R 20000UL // Measurement noise co-variance

#endif
//End of file