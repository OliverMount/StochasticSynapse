COMMENT
/** 
 * Stochastic Exp2Syn mechanism for NEURON 9.0
 * Simple version without deprecated functions
 * @author Adapted for NEURON by Oliver, IBS, Daejeon, South Korea, July 30, 2025
 * @brief Stochastic double exponential synapse with release probability
 */
ENDCOMMENT

NEURON {
    POINT_PROCESS StochExp2Syn_temp
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
    rng_state    : Internal random number generator state
}

STATE {
    A (uS)
    B (uS)
}

INITIAL { 
    LOCAL tp
    
    : Initialize event counter and RNG state
    event_count = 0
    rng_state = seed_spatial * 1009 + seed_temporal * 1013
    
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
    
    : Generate random number 
    rval = next_random()
    
    if (rval < release_prob) {
        A = A + weight * factor
        B = B + weight * factor
    }
    
    : Increment event counter for temporal variation
    event_count = event_count + 1
}

: Generate next random number using internal state
FUNCTION next_random() {
    : Update internal state combining all seeds and event count
    rng_state = rng_state * 1103515245 + 12345 + event_count * 1019
    
    : Generate uniform random number [0,1)
    next_random = simple_uniform()
}

: Simple uniform random number generator using internal state
FUNCTION simple_uniform() {
VERBATIM
    /* Use internal rng_state for random number generation */
    unsigned long state = (unsigned long)rng_state;
    state = state * 1103515245 + 12345;
    rng_state = (double)state;
    _lsimple_uniform = ((double)(state & 0x7fffffff)) / 2147483648.0;
ENDVERBATIM
}

: Procedure to set seeds
PROCEDURE setSeeds(spatial, temporal) {
    seed_spatial = spatial
    seed_temporal = temporal
    event_count = 0
    : Re-initialize RNG state
    rng_state = seed_spatial * 1009 + seed_temporal * 1013
}

: Function to get current event count
FUNCTION getEventCount() {
    getEventCount = event_count
}

: Reset event counter
PROCEDURE resetCounter() {
    event_count = 0
}

: Get current random state (for debugging)
FUNCTION getRngState() {
    getRngState = rng_state
}

COMMENT
Usage Examples:

Python:
-------
from neuron import h
import numpy as np

# Create synapse
syn = h.StochExp2Syn(soma(0.5))
syn.tau1 = 0.1      # Rise time (ms)
syn.tau2 = 10       # Decay time (ms) 
syn.e = 0           # Reversal potential (mV)
syn.release_prob = 0.5  # 50% release probability

# Set seeds for reproducible randomness
syn.setSeeds(123, 456)  # spatial_seed, temporal_seed

# Connect to stimulus
stim = h.NetStim()
stim.number = 100
stim.start = 10
stim.interval = 5
nc = h.NetCon(stim, syn, 0, 0, 0.01)  # weight in uS

# Different synapses should use different spatial seeds
syn1.setSeeds(123, 456)  # synapse 1
syn2.setSeeds(124, 456)  # synapse 2 (different spatial seed)
syn3.setSeeds(125, 456)  # synapse 3 (different spatial seed)

# Different runs should use different temporal seeds
syn.setSeeds(123, 456)   # run 1
syn.setSeeds(123, 457)   # run 2 (different temporal seed)

HOC:
----
objref syn, stim, nc

syn = new StochExp2Syn(0.5)
syn.tau1 = 0.1
syn.tau2 = 10
syn.e = 0
syn.release_prob = 0.5
syn.setSeeds(123, 456)

stim = new NetStim()
stim.number = 100
stim.start = 10  
stim.interval = 5
nc = new NetCon(stim, syn, 0, 0, 0.01)

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

ENDCOMMENT
