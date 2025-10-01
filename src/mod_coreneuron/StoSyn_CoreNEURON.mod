COMMENT
/** 
 * Simple uniform random number generator compatible with CoreNEURON
 * @author Generated using the script netstim_inhpoisson.mod from Sonata network in 
 * https://gitlab.ebrains.eu/BlueBrain/efel/-/tree/5.6.17
 * https://gitlab.ebrains.eu/BlueBrain/efel/-/tree/5.6.17/examples/sonata-network/mechanisms
 * @brief Inhibitory poisson generator by the thinning method.
 * @author Eilif Muller
 * @date 2011-03-16
 * @remark Copyright 2005-2023 Blue Brain Project / EPFL
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 
 * @Adopted for Stochastic Synapse by Oliver, IBS, Daejeon, South Korea, July 29, 2025. 
 */
 
ENDCOMMENT

NEURON {
    THREADSAFE
    POINT_PROCESS StochExp2SynGPU 
    RANGE seed1, seed2, seed3, tau1, tau2, e, i, g, release_prob 
    BBCOREPOINTER rng 
    NONSPECIFIC_CURRENT i
}


UNITS {
    (nA) = (nanoamp)
    (mV) = (millivolt)
    (uS) = (microsiemens)
}

PARAMETER {
    seed1 = 1 : First seed for random123
    seed2 = 2 : Second seed for random123  
    seed3 = 3 : Third seed for random123 
    
    : From the the Exp2Syn mod file
    tau1 = 0.1 (ms) <1e-9,1e9>
    tau2 = 10 (ms) <1e-9,1e9>
    e = 0 (mV)
    release_prob = 0.5 
}

VERBATIM
#ifndef CORENEURON_BUILD
    #include "nrnran123.h"
#endif
ENDVERBATIM

ASSIGNED {
    rng
    v (mV)
    i (nA)
    g (uS)
    factor
}


STATE {
    A (uS)
    B (uS)
}

INITIAL { 
    LOCAL tp
    
    VERBATIM
    #ifndef CORENEURON_BUILD
    /* NEURON initialization */
    nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
    if (*pv) {
        nrnran123_deletestream(*pv);
    }
    *pv = nrnran123_newstream3((uint32_t)seed1, (uint32_t)seed2, (uint32_t)seed3);
    nrnran123_setseq(*pv, 0, 0);
    #else
    /* CoreNEURON initialization - RNG already setup from bbcore_read */
    if (_p_rng) {
        nrnran123_State* rng_state = (nrnran123_State*)_p_rng;
        nrnran123_setseq(rng_state, 0, 0);
    }
    #endif
    ENDVERBATIM
    
    : From the the Exp2Syn mod file
    if (tau1 / tau2 > 0.9999) {
        tau1 = 0.9999 * tau2
    }
    if (tau1 / tau2 < 1e-9) {
        tau1 = tau2 * 1e-9
    }
    tp = (tau1 * tau2) / (tau2 - tau1) * log(tau2 / tau1)
    factor = -exp(-tp / tau1) + exp(-tp / tau2)
    factor = 1 / factor

    A = 0
    B = 0 
}


BREAKPOINT {
    SOLVE state METHOD cnexp
    g = B - A
    i = g * (v - e)
}

DERIVATIVE state {
    A' = -A / tau1
    B' = -B / tau2
}
 
 
NET_RECEIVE(weight (uS)) {
    LOCAL rval
    rval = urand()
    if (rval < release_prob) {
        A = A + weight * factor
        B = B + weight * factor
    }
} 
  

: Main function to generate uniform random numbers [0,1)
FUNCTION urand() {
    VERBATIM
    if (_p_rng) {
        nrnran123_State* rng_state = (nrnran123_State*)_p_rng;
        _lurand = nrnran123_dblpick(rng_state);
    } else {
        _lurand = 0.0;
    }
    ENDVERBATIM
}

: Function to get uniform random in range [min, max)
FUNCTION urand_range(min, max) {
    urand_range = min + (max - min) * urand()
}

: Procedure to set new seeds and reinitialize the stream
PROCEDURE setSeeds(new_seed1, new_seed2, new_seed3) {
    seed1 = new_seed1
    seed2 = new_seed2  
    seed3 = new_seed3
    
    VERBATIM
    #ifndef CORENEURON_BUILD
    nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
    if (*pv) {
        nrnran123_deletestream(*pv);
    }
    *pv = nrnran123_newstream3((uint32_t)seed1, (uint32_t)seed2, (uint32_t)seed3);
    nrnran123_setseq(*pv, 0, 0);
    #endif
    ENDVERBATIM
}

VERBATIM
#ifndef CORENEURON_BUILD

/* CoreNEURON serialization support - NEURON side */
static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    if (iArray) {
        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
        nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
        
        if (*pv) {
            nrnran123_getids3(*pv, ia, ia+1, ia+2);
            char which;
            nrnran123_getseq(*pv, ia+3, &which);
            ia[4] = (int)which;
        } else {
            ia[0] = (uint32_t)seed1;
            ia[1] = (uint32_t)seed2;
            ia[2] = (uint32_t)seed3;
            ia[3] = 0;
            ia[4] = 0;
        }
    }
    *ioffset += 5;
}

static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
    nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
    
    if (*pv) {
        nrnran123_deletestream(*pv);
    }
    *pv = nrnran123_newstream3(ia[0], ia[1], ia[2]);
    nrnran123_setseq(*pv, ia[3], (char)ia[4]);
    
    *ioffset += 5;
}

#else

/* CoreNEURON version of serialization */
static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    if (iArray) {
        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
        nrnran123_State* rng_state = (nrnran123_State*)_p_rng;
        
        if (rng_state) {
            nrnran123_getids3(rng_state, ia, ia+1, ia+2);
            char which;
            nrnran123_getseq(rng_state, ia+3, &which);
            ia[4] = (int)which;
        } else {
            ia[0] = (uint32_t)seed1;
            ia[1] = (uint32_t)seed2;
            ia[2] = (uint32_t)seed3;
            ia[3] = 0;
            ia[4] = 0;
        }
    }
    *ioffset += 5;
}

static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
    
    /* CoreNEURON: create new stream with the 4-argument version */
    nrnran123_State* rng_state = nrnran123_newstream3(ia[0], ia[1], ia[2]);
    _p_rng = (double*)rng_state;
    nrnran123_setseq(rng_state, ia[3], (char)ia[4]);
    
    *ioffset += 5;
}

#endif
ENDVERBATIM

CONSTRUCTOR {
    VERBATIM
    #ifndef CORENEURON_BUILD
    /* NEURON: Initialize RNG */
    nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
    *pv = nrnran123_newstream3((uint32_t)seed1, (uint32_t)seed2, (uint32_t)seed3);
    nrnran123_setseq(*pv, 0, 0);
    #endif
    ENDVERBATIM
}

DESTRUCTOR {
    VERBATIM
    #ifndef CORENEURON_BUILD
    /* NEURON: Clean up random stream */
    nrnran123_State** pv = (nrnran123_State**)(&_p_rng);
    if (*pv) {
        nrnran123_deletestream(*pv);
        *pv = NULL;
    }
    #endif
    ENDVERBATIM
}
