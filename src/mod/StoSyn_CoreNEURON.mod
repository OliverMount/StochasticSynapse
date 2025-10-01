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
    ARTIFICIAL_CELL StochExp2SynGPU 
    RANGE seed1, seed2, seed3, tau1, tau2, e, i, g, release_prob 
    BBCOREPOINTER uniform_rng 
    NONSPECIFIC_CURRENT i
}


UNITS {
    (nA) = (nanoamp)
    (mV) = (millivolt)
    (uS) = (microsiemens)
}

VERBATIM
#if defined(NRN_VERSION_GTEQ)
#if NRN_VERSION_GTEQ(9,0,0)
#define NRN_VERSION_GTEQ_9_0_0
#endif
#endif

#ifndef NRN_VERSION_GTEQ_8_2_0
extern int ifarg(int iarg);
#ifndef CORENEURON_BUILD
extern double* vector_vec(void* vv);
extern void* vector_new1(int _i);
extern int vector_capacity(void* vv);
extern void* vector_arg(int iarg);
double nrn_random_pick(void* r);
#endif
void* nrn_random_arg(int argpos);
#define RANDCAST 
#else
#define RANDCAST (Rand*)
#endif
ENDVERBATIM

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
#include "nrnran123.h"
ENDVERBATIM

ASSIGNED {
    uniform_rng
    usingR123
    last_random : Store last generated random number
    
    
    : From the the Exp2Syn mod file
     v (mV)
    i (nA)
    g (uS)
    factor
}


ASSIGNED {
  
}

STATE {
    A (uS)
    B (uS)
}
  

 

INITIAL { 
	LOCAL tp
    usingR123 = 1
    last_random = 0.0
    
    VERBATIM
    /* Initialize uniform_rng pointer to NULL first */
    _p_uniform_rng = (double*)0;
    
    /* Initialize random123 stream with provided seeds */
    nrnran123_State** pv = (nrnran123_State**)(&_p_uniform_rng);
    *pv = nrnran123_newstream3((uint32_t)seed1, (uint32_t)seed2, (uint32_t)seed3);
    
    /* Initialize sequence to 0 for reproducibility */
    if (_p_uniform_rng) {
        nrnran123_setseq((nrnran123_State*)_p_uniform_rng, 0, 0);
    }
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
    rval = urand()  : uniform random number for every net_receive call
    if (rval < release_prob) {
        A = A + weight * factor
        B = B + weight * factor
 		VERBATIM
        // printf("RELEASED SPIKE\n");
        ENDVERBATIM
    } else {
      	VERBATIM
        // printf("FAILED TO RELEASE SPIKE\n");
        ENDVERBATIM
    }
} 
  

: Main function to generate uniform random numbers [0,1)
FUNCTION urand() {
    VERBATIM
    if (_p_uniform_rng) {
        if (usingR123) {
            _lurand = nrnran123_dblpick((nrnran123_State*)_p_uniform_rng);
        } else {
#ifndef CORENEURON_BUILD
            _lurand = nrn_random_pick(RANDCAST _p_uniform_rng);
#endif
        }
    } else {
        _lurand = 0.5; /* fallback value if not initialized */
        hoc_execerror("UniformRNG: random stream not initialized", "call setSeeds() first");
    }
    ENDVERBATIM
}

: Function to get uniform random in range [min, max)
FUNCTION urand_range(min, max) {
    urand_range = min + (max - min) * urand()
}

: Function to get uniform random integer in range [min, max]
FUNCTION urand_int(min, max) {
    urand_int = floor(min + (max - min + 1) * urand())
}

: Procedure to set new seeds and reinitialize the stream
PROCEDURE setSeeds(new_seed1, new_seed2, new_seed3) {
    seed1 = new_seed1
    seed2 = new_seed2  
    seed3 = new_seed3
    
    VERBATIM
    /* Clean up existing stream if any */
    if (_p_uniform_rng) {
        nrnran123_deletestream((nrnran123_State*)_p_uniform_rng);
        _p_uniform_rng = (double*)0;
    }
    
    /* Initialize with new seeds */
    nrnran123_State** pv = (nrnran123_State**)(&_p_uniform_rng);
    *pv = nrnran123_newstream3((uint32_t)seed1, (uint32_t)seed2, (uint32_t)seed3);
    
    if (_p_uniform_rng) {
        nrnran123_setseq((nrnran123_State*)_p_uniform_rng, 0, 0);
        usingR123 = 1;
    }
    ENDVERBATIM
}

: Procedure to advance the random stream by n steps
PROCEDURE advance_stream(n) {
    LOCAL i
    FROM i = 1 TO n {
        last_random = urand()
    }
}

: Get the last generated random number without generating a new one
FUNCTION get_last() {
    get_last = last_random
}

: Generate and store a new random number
FUNCTION next_random() {
    last_random = urand()
    next_random = last_random
}

VERBATIM
/* CoreNEURON serialization support */
static void bbcore_write(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    if (iArray) {
        uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
        nrnran123_State** pv = (nrnran123_State**)(&_p_uniform_rng);
        
        if (*pv) {
            nrnran123_getids3(*pv, ia, ia+1, ia+2);
            
            // Store sequence information
            char which;
            nrnran123_getseq(*pv, ia+3, &which);
            ia[4] = (int)which;
        } else {
            // No stream initialized
            ia[0] = ia[1] = ia[2] = ia[3] = ia[4] = 0;
        }
    }
    *ioffset += 5;
}

static void bbcore_read(double* dArray, int* iArray, int* doffset, int* ioffset, _threadargsproto_) {
    uint32_t* ia = ((uint32_t*)iArray) + *ioffset;
    
    if (ia[0] != 0 || ia[1] != 0 || ia[2] != 0) {
        nrnran123_State** pv = (nrnran123_State**)(&_p_uniform_rng);
#if !NRNBBCORE
        if (*pv) {
            nrnran123_deletestream(*pv);
        }
#endif
        *pv = nrnran123_newstream3(ia[0], ia[1], ia[2]);
        nrnran123_setseq(*pv, ia[3], (char)ia[4]);
        usingR123 = 1;
    }
    
    *ioffset += 5;
}
ENDVERBATIM
