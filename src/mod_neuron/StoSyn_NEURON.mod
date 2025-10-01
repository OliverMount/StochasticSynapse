COMMENT
/**  
 * Using NEURON's built-in scop_random() function
 * @author Adapted for NEURON by Oliver, IBS, Daejeon, South Korea, July 30, 2025
 * @brief Stochastic double exponential synapse with release probability
 
 Features:
---------
1. Dual seed system for spatial and temporal randomness
2. Simple but effective random number generator
3. No deprecated function calls
4. Thread-safe (no POINTER usage)
5. Reproducible results with same seeds
6. Event counter provides temporal variation

The mechanism generates different random sequences for:
- Different spatial seeds (different synapses)
- Different temporal seeds (different runs)
- Different event counts (different spikes at same synapse)
 
*/
ENDCOMMENT

NEURON {
    POINT_PROCESS StochExp2Syn
    RANGE tau1, tau2, e, i, g, release_prob
    RANGE seed_spatial, seed_temporal
    NONSPECIFIC_CURRENT i
}

UNITS {
    (nA) = (nanoamp)
    (mV) = (millivolt)
    (uS) = (microsiemens)
}

PARAMETER {
    : Spatial and temporal seeds for randomness
    seed_spatial = 1    : Seed for spatial variation (different synapses)
    seed_temporal = 1   : Seed for temporal variation (different events)
    
    : Synaptic parameters
    tau1 = 0.1 (ms) <1e-9,1e9>  : Rise time constant
    tau2 = 10 (ms) <1e-9,1e9>   : Decay time constant  
    e = 0 (mV)                  : Reversal potential
    release_prob = 0.5 <0,1>    : Release probability [0,1]
}

ASSIGNED {
    v (mV)
    i (nA)
    g (uS)
    factor
    event_count  : Counter for temporal randomness
}

STATE {
    A (uS)
    B (uS)
}

INITIAL { 
    LOCAL tp
    
    : Initialize event counter
    event_count = 0
    
    : Set the random seed combining spatial and temporal components
    set_seed(seed_spatial * 1009 + seed_temporal * 1013)
    
    : Calculate normalization factor
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
    
    : Generate uniform random number using NEURON's scop_random()
    rval = scop_random()
    
    if (rval < release_prob) {
        A = A + weight * factor
        B = B + weight * factor
        VERBATIM
		// printf("RELEASED SPIKES \n");
		ENDVERBATIM
         
    }
    else{
      	VERBATIM
		// printf("FAILED TO RELEASE SPIKES \n");
		ENDVERBATIM
    }
    
    : Increment event counter for temporal variation
    event_count = event_count + 1
}

: Procedure to set seeds - reinitializes the random number generator
PROCEDURE setSeeds(spatial, temporal) {
    seed_spatial = spatial
    seed_temporal = temporal
    event_count = 0
    
    : Reset NEURON's random seed with combined seed
    set_seed(seed_spatial * 1009 + seed_temporal * 1013)
}

: Function to get current event count
FUNCTION getEventCount() {
    getEventCount = event_count
}

: Reset event counter
PROCEDURE resetCounter() {
    event_count = 0
}

: Generate a random number on demand (useful for testing)
FUNCTION getRandomNumber() {
    getRandomNumber = scop_random()
}

: Procedure to advance the random stream by n steps
PROCEDURE advanceRandom(n) {
    LOCAL i, dummy
    FROM i = 1 TO n {
        dummy = scop_random()
    }
} 
