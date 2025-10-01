# Compiling the mod files

export NMODL_PYLIB=/usr/lib/x86_64-linux-gnu/libpython3.12.so
export NMODLHOME=/home/oli/install/

rm -rf tmp/ x86_64/

/home/oli/install/bin/nrnivmodl -coreneuron mod_coreneuron
#/home/oli/install/bin/nrnivmodl  mod_neuron 

