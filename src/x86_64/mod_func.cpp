#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern "C" void _StoSyn_NEURON_reg(void);

extern "C" void modl_reg() {
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");
    fprintf(stderr, " \"mod/StoSyn_NEURON.mod\"");
    fprintf(stderr, "\n");
  }
  _StoSyn_NEURON_reg();
}
