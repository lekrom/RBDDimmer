#ifndef RBDMCUAVR_H
#define RBDMCUAVR_H

#include "Arduino.h"
#include "RBDdimmer.h"
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define DIMMER_TIMER 4
#define INT_vect INT4_vect   
#define INTx INT4
#define EICRX EICRB
#define ISCx1 ISC41
#define ISCx0 ISC40
#define ALL_DIMMERS 50

#define TCCRxA_VALUE 0x00 // CTC mode
#define TCCRxB_VALUE 0x0A // 0b1011 // (1 << WGMx2)|(1 << CSx1)|(1 << CSx0)
#define OCRxAH_VALUE 0x00
#define OCRxAL_VALUE 0x0F

#define _OCRxAH(X) OCR ## X ## AH
#define OCRxAH(X) _OCRxAH(X)
#define _OCRxAL(X) OCR ## X ## AL
#define OCRxAL(X) _OCRxAL(X)

#elif defined(__AVR_ATmega32U4__)
#define DIMMER_TIMER 1
#define INT_vect INT6_vect
#define INTx INT6
#define EICRX EICRB
#define ISCx1 ISC61
#define ISCx0 ISC60
#define ALL_DIMMERS 30

#define TCCRxA_VALUE 0x00 // CTC mode
#define TCCRxB_VALUE 0x09// 0b1011 (1 << WGMx2)|(1 << CSx1)|(1 << CSx0)
#define OCRxAH_VALUE 0x00
#define OCRxAL_VALUE 0xBC

#define _OCRxAH(X) OCR ## X ## AH
#define OCRxAH(X) _OCRxAH(X)
#define _OCRxAL(X) OCR ## X ## AL
#define OCRxAL(X) _OCRxAL(X)

#else
#define DIMMER_TIMER 2
#define INT_vect INT0_vect
#define INTx INT0
#define EICRX EICRA
#define ISCx1 ISC01
#define ISCx0 ISC00
#define ALL_DIMMERS 13

#define TCCRxA_VALUE 0x02
#define TCCRxB_VALUE 0x09 // 0b1010 // (1 << WGMx2)|(1 << CSx1)
#define OCRxAH_VALUE 0x00
#define OCRxAL_VALUE 0xFF

#define _OCRxAH(X) OCR ## X ## A
#define OCRxAH(X) _OCRxAH(X)
#define _OCRxAL(X) OCR ## X ## A
#define OCRxAL(X) _OCRxAL(X)

#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
static const uint16_t powerBuf[] = {
      /*
     610,   604, 598,  592,  586,  580,  574,  568,  562,   556,
     550,   544, 538,  532,  526,  520,  514,  508,  502,   496,
     490,   484, 478,  472,  466,  460,  454,  448,  442,   436,
     430,   424, 418,  412,  406,  400,  394,  388,  382,   376,
     370,   364, 358,  352,  346,  340,  334,  328,  322,   316,
     310,   304, 298,  292,  286,  280,  274,  268,  262,   256,
     250,   244, 238,  232,  226,  220,  214,  208,  202,   196,
     180,   174, 168,  162,  156,  150,  144,  138,  132,   126,
     120,   114, 108,  102,  96,   90,   84,   78,   72,     66,
      60,    54,  48,   42,  36,   30,   24,   18,   12,      6*/      
     610,  536,  518,  506,  494,  488,  475,  469,  463,  457,  
     451,  445,  439,  433,  433,  427,  420,  414,  414,  408,  
     402,  402,  396,  390,  390,  384,  384,  378,  372,  372,  
     366,  366,  359,  359,  353,  353,  347,  347,  341,  341,  
     335,  335,  329,  329,  323,  323,  317,  317,  311,  305,  
     305,  305,  298,  292,  292,  286,  286,  280,  280,  274,  
     274,  268,  268,  262,  262,  256,  256,  250,  250,  244,  
     244,  237,  237,  231,  225,  225,  219,  219,  213,  207,  
     207,  201,  195,  195,  189,  183,  176,  176,  170,  164,  
     158,  152,  146,  140,  134,  122,  115,  103,   91,   73



};
#else
static const uint16_t powerBuf[] = {
      /*
     600,   600, 598,  592,  586,  580,  574,  568,  562,   556,
     550,   544, 538,  532,  526,  520,  514,  508,  502,   496,
     490,   484, 478,  472,  466,  460,  454,  448,  442,   436,
     430,   424, 418,  412,  406,  400,  394,  388,  382,   376,
     370,   364, 358,  352,  346,  340,  334,  328,  322,   316,
     310,   304, 298,  292,  286,  280,  274,  268,  262,   256,
     250,   244, 238,  232,  226,  220,  214,  208,  202,   196,
     180,   174, 168,  162,  156,  150,  144,  138,  132,   126,
     120,   114, 108,  102,  96,   90,   84,   78,   72,     66,
      60,    54,  48,   42,  36,   30,   24,   18,   12,      8*/      
     600,  528,  510,  498,  486,  480,  468,  462,  456,  450,  
     444,  438,  432,  426,  426,  420,  414,  408,  408,  402,  
     396,  396,  390,  384,  384,  378,  378,  372,  366,  366,  
     360,  360,  354,  354,  348,  348,  342,  342,  336,  336,  
     330,  330,  324,  324,  318,  318,  312,  312,  306,  300,  
     300,  300,  294,  288,  288,  282,  282,  276,  276,  270,  
     270,  264,  264,  258,  258,  252,  252,  246,  246,  240,  
     240,  234,  234,  228,  222,  222,  216,  216,  210,  204,  
     204,  198,  192,  192,  186,  180,  174,  174,  168,  162,  
     156,  150,  144,  138,  132,  120,  114,  102,   90,   72,          
};
#endif

#endif
