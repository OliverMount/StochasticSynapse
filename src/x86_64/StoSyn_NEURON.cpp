/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
// clang-format off
#include "md1redef.h"
#include "section_fwd.hpp"
#include "nrniv_mf.h"
#include "md2redef.h"
#include "nrnconf.h"
// clang-format on
#include "neuron/cache/mechanism_range.hpp"
#include <vector>
using std::size_t;
static auto& std_cerr_stream = std::cerr;
static constexpr auto number_of_datum_variables = 2;
static constexpr auto number_of_floating_point_variables = 16;
namespace {
template <typename T>
using _nrn_mechanism_std_vector = std::vector<T>;
using _nrn_model_sorted_token = neuron::model_sorted_token;
using _nrn_mechanism_cache_range = neuron::cache::MechanismRange<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_mechanism_cache_instance = neuron::cache::MechanismInstance<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_non_owning_id_without_container = neuron::container::non_owning_identifier_without_container;
template <typename T>
using _nrn_mechanism_field = neuron::mechanism::field<T>;
template <typename... Args>
void _nrn_mechanism_register_data_fields(Args&&... args) {
  neuron::mechanism::register_data_fields(std::forward<Args>(args)...);
}
}
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#if NRN_ENABLE_ARCH_INDEP_EXP_POW
#undef pow
#define pow hoc_pow
#endif
#endif
 
#define nrn_init _nrn_init__StochExp2Syn
#define _nrn_initial _nrn_initial__StochExp2Syn
#define nrn_cur _nrn_cur__StochExp2Syn
#define _nrn_current _nrn_current__StochExp2Syn
#define nrn_jacob _nrn_jacob__StochExp2Syn
#define nrn_state _nrn_state__StochExp2Syn
#define _net_receive _net_receive__StochExp2Syn 
#define advanceRandom advanceRandom__StochExp2Syn 
#define resetCounter resetCounter__StochExp2Syn 
#define setSeeds setSeeds__StochExp2Syn 
#define state state__StochExp2Syn 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _internalthreadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
#define _internalthreadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define seed_spatial _ml->template fpfield<0>(_iml)
#define seed_spatial_columnindex 0
#define seed_temporal _ml->template fpfield<1>(_iml)
#define seed_temporal_columnindex 1
#define tau1 _ml->template fpfield<2>(_iml)
#define tau1_columnindex 2
#define tau2 _ml->template fpfield<3>(_iml)
#define tau2_columnindex 3
#define e _ml->template fpfield<4>(_iml)
#define e_columnindex 4
#define release_prob _ml->template fpfield<5>(_iml)
#define release_prob_columnindex 5
#define i _ml->template fpfield<6>(_iml)
#define i_columnindex 6
#define g _ml->template fpfield<7>(_iml)
#define g_columnindex 7
#define A _ml->template fpfield<8>(_iml)
#define A_columnindex 8
#define B _ml->template fpfield<9>(_iml)
#define B_columnindex 9
#define factor _ml->template fpfield<10>(_iml)
#define factor_columnindex 10
#define event_count _ml->template fpfield<11>(_iml)
#define event_count_columnindex 11
#define DA _ml->template fpfield<12>(_iml)
#define DA_columnindex 12
#define DB _ml->template fpfield<13>(_iml)
#define DB_columnindex 13
#define _g _ml->template fpfield<14>(_iml)
#define _g_columnindex 14
#define _tsav _ml->template fpfield<15>(_iml)
#define _tsav_columnindex 15
#define _nd_area *_ml->dptr_field<0>(_iml)
 static _nrn_mechanism_cache_instance _ml_real{nullptr};
static _nrn_mechanism_cache_range *_ml{&_ml_real};
static size_t _iml{0};
static Datum *_ppvar;
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_advanceRandom(void*);
 static double _hoc_getRandomNumber(void*);
 static double _hoc_getEventCount(void*);
 static double _hoc_resetCounter(void*);
 static double _hoc_setSeeds(void*);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(Object* _ho) { void* create_point_process(int, Object*);
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt(void*);
 static double _hoc_loc_pnt(void* _vptr) {double loc_point_process(int, void*);
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(void* _vptr) {double has_loc_point(void*);
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(void* _vptr) {
 double get_loc_point_process(void*); return (get_loc_point_process(_vptr));
}
 static void _hoc_setdata(void*);
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {0, 0}
};
 static Member_func _member_func[] = {
 {"loc", _hoc_loc_pnt},
 {"has_loc", _hoc_has_loc},
 {"get_loc", _hoc_get_loc_pnt},
 {"advanceRandom", _hoc_advanceRandom},
 {"getRandomNumber", _hoc_getRandomNumber},
 {"getEventCount", _hoc_getEventCount},
 {"resetCounter", _hoc_resetCounter},
 {"setSeeds", _hoc_setSeeds},
 {0, 0}
};
#define getRandomNumber getRandomNumber_StochExp2Syn
#define getEventCount getEventCount_StochExp2Syn
 extern double getRandomNumber( );
 extern double getEventCount( );
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {"release_prob", 0, 1},
 {"tau2", 1e-09, 1e+09},
 {"tau1", 1e-09, 1e+09},
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"tau1", "ms"},
 {"tau2", "ms"},
 {"e", "mV"},
 {"A", "uS"},
 {"B", "uS"},
 {"i", "nA"},
 {"g", "uS"},
 {0, 0}
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 neuron::legacy::set_globals_from_prop(_prop, _ml_real, _ml, _iml);
_ppvar = _nrn_mechanism_access_dparam(_prop);
 Node * _node = _nrn_mechanism_access_node(_prop);
v = _nrn_mechanism_access_voltage(_node);
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void _hoc_destroy_pnt(void* _vptr) {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[2].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"StochExp2Syn",
 "seed_spatial",
 "seed_temporal",
 "tau1",
 "tau2",
 "e",
 "release_prob",
 0,
 "i",
 "g",
 0,
 "A",
 "B",
 0,
 0};
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     1, /* seed_spatial */
     1, /* seed_temporal */
     0.1, /* tau1 */
     10, /* tau2 */
     0, /* e */
     0.5, /* release_prob */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
  if (nrn_point_prop_) {
    _nrn_mechanism_access_alloc_seq(_prop) = _nrn_mechanism_access_alloc_seq(nrn_point_prop_);
    _ppvar = _nrn_mechanism_access_dparam(nrn_point_prop_);
  } else {
   _ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 16);
 	/*initialize range parameters*/
 	seed_spatial = _parm_default[0]; /* 1 */
 	seed_temporal = _parm_default[1]; /* 1 */
 	tau1 = _parm_default[2]; /* 0.1 */
 	tau2 = _parm_default[3]; /* 10 */
 	e = _parm_default[4]; /* 0 */
 	release_prob = _parm_default[5]; /* 0.5 */
  }
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 16);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _StoSyn_NEURON_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"seed_spatial"} /* 0 */,
                                       _nrn_mechanism_field<double>{"seed_temporal"} /* 1 */,
                                       _nrn_mechanism_field<double>{"tau1"} /* 2 */,
                                       _nrn_mechanism_field<double>{"tau2"} /* 3 */,
                                       _nrn_mechanism_field<double>{"e"} /* 4 */,
                                       _nrn_mechanism_field<double>{"release_prob"} /* 5 */,
                                       _nrn_mechanism_field<double>{"i"} /* 6 */,
                                       _nrn_mechanism_field<double>{"g"} /* 7 */,
                                       _nrn_mechanism_field<double>{"A"} /* 8 */,
                                       _nrn_mechanism_field<double>{"B"} /* 9 */,
                                       _nrn_mechanism_field<double>{"factor"} /* 10 */,
                                       _nrn_mechanism_field<double>{"event_count"} /* 11 */,
                                       _nrn_mechanism_field<double>{"DA"} /* 12 */,
                                       _nrn_mechanism_field<double>{"DB"} /* 13 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 14 */,
                                       _nrn_mechanism_field<double>{"_tsav"} /* 15 */,
                                       _nrn_mechanism_field<double*>{"_nd_area", "area"} /* 0 */,
                                       _nrn_mechanism_field<Point_process*>{"_pntproc", "pntproc"} /* 1 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 2 */);
  hoc_register_prop_size(_mechtype, 16, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 StochExp2Syn /media/oli/Research/Gitrepo/StochasticSynapse/src/mod/StoSyn_NEURON.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int advanceRandom(double);
static int resetCounter();
static int setSeeds(double, double);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[2], _dlist1[2];
 static int state(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / tau1 ;
   DB = - B / tau2 ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( - 1.0 ) / tau1 )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / tau2 )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( - 1.0 ) / tau1)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / tau2)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - B) ;
   }
  return 0;
}
 
static void _net_receive (Point_process* _pnt, double* _args, double _lflag) 
{   neuron::legacy::set_globals_from_prop(_pnt->_prop, _ml_real, _ml, _iml);
    _ppvar = _nrn_mechanism_access_dparam(_pnt->_prop);
  if (_tsav > t){ hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   double _lrval ;
 _lrval = scop_random ( ) ;
   if ( _lrval < release_prob ) {
       if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A;
    double __primary = (A + _args[0] * factor) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - __primary );
    A += __primary;
  } else {
 A = A + _args[0] * factor ;
       }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B;
    double __primary = (B + _args[0] * factor) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - __primary );
    B += __primary;
  } else {
 B = B + _args[0] * factor ;
       }
 
/*VERBATIM*/
		// printf("RELEASED SPIKES \n");
 }
   else {
     
/*VERBATIM*/
		// printf("FAILED TO RELEASE SPIKES \n");
 }
   event_count = event_count + 1.0 ;
   } }
 
static int  setSeeds (  double _lspatial , double _ltemporal ) {
   seed_spatial = _lspatial ;
   seed_temporal = _ltemporal ;
   event_count = 0.0 ;
   set_seed ( seed_spatial * 1009.0 + seed_temporal * 1013.0 ) ;
    return 0; }
 
static double _hoc_setSeeds(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r = 1.;
 setSeeds (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double getEventCount (  ) {
   double _lgetEventCount;
 _lgetEventCount = event_count ;
   
return _lgetEventCount;
 }
 
static double _hoc_getEventCount(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r =  getEventCount (  );
 return(_r);
}
 
static int  resetCounter (  ) {
   event_count = 0.0 ;
    return 0; }
 
static double _hoc_resetCounter(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r = 1.;
 resetCounter (  );
 return(_r);
}
 
double getRandomNumber (  ) {
   double _lgetRandomNumber;
 _lgetRandomNumber = scop_random ( ) ;
   
return _lgetRandomNumber;
 }
 
static double _hoc_getRandomNumber(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r =  getRandomNumber (  );
 return(_r);
}
 
static int  advanceRandom (  double _ln ) {
   double _li , _ldummy ;
 {int  _li ;for ( _li = 1 ; _li <= ((int) _ln ) ; _li ++ ) {
     _ldummy = scop_random ( ) ;
     } }
    return 0; }
 
static double _hoc_advanceRandom(void* _vptr) {
 double _r;
    auto* const _pnt = static_cast<Point_process*>(_vptr);
  auto* const _p = _pnt->_prop;
  if (!_p) {
    hoc_execerror("POINT_PROCESS data instance not valid", NULL);
  }
   _setdata(_p);
 _r = 1.;
 advanceRandom (  *getarg(1) );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 2; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
      Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  B = B0;
 {
   double _ltp ;
 event_count = 0.0 ;
   set_seed ( seed_spatial * 1009.0 + seed_temporal * 1013.0 ) ;
   if ( tau1 / tau2 > 0.9999 ) {
     tau1 = 0.9999 * tau2 ;
     }
   if ( tau1 / tau2 < 1e-9 ) {
     tau1 = tau2 * 1e-9 ;
     }
   _ltp = ( tau1 * tau2 ) / ( tau2 - tau1 ) * log ( tau2 / tau1 ) ;
   factor = - exp ( - _ltp / tau1 ) + exp ( - _ltp / tau2 ) ;
   factor = 1.0 / factor ;
   A = 0.0 ;
   B = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
Node *_nd; double _v; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _tsav = -1e20;
   _v = _vec_v[_ni[_iml]];
 v = _v;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   g = B - A ;
   i = g * ( v - e ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
Node *_nd; int* _ni; double _rhs, _v; int _cntml;
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 auto const _g_local = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g_local - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _cntml;
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
_ml = &_lmr;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
 { error =  state();
 if(error){
  std_cerr_stream << "at line 86 in file StoSyn_NEURON.mod:\nBREAKPOINT {\n";
  std_cerr_stream << _ml << ' ' << _iml << '\n';
  abort_run(error);
}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {A_columnindex, 0};  _dlist1[0] = {DA_columnindex, 0};
 _slist1[1] = {B_columnindex, 0};  _dlist1[1] = {DB_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/media/oli/Research/Gitrepo/StochasticSynapse/src/mod/StoSyn_NEURON.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "/**  \n"
  " * Using NEURON's built-in scop_random() function\n"
  " * @author Adapted for NEURON by Oliver, IBS, Daejeon, South Korea, July 30, 2025\n"
  " * @brief Stochastic double exponential synapse with release probability\n"
  " \n"
  " Features:\n"
  "---------\n"
  "1. Dual seed system for spatial and temporal randomness\n"
  "2. Simple but effective random number generator\n"
  "3. No deprecated function calls\n"
  "4. Thread-safe (no POINTER usage)\n"
  "5. Reproducible results with same seeds\n"
  "6. Event counter provides temporal variation\n"
  "\n"
  "The mechanism generates different random sequences for:\n"
  "- Different spatial seeds (different synapses)\n"
  "- Different temporal seeds (different runs)\n"
  "- Different event counts (different spikes at same synapse)\n"
  " \n"
  "*/\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    POINT_PROCESS StochExp2Syn\n"
  "    RANGE tau1, tau2, e, i, g, release_prob\n"
  "    RANGE seed_spatial, seed_temporal\n"
  "    NONSPECIFIC_CURRENT i\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    (nA) = (nanoamp)\n"
  "    (mV) = (millivolt)\n"
  "    (uS) = (microsiemens)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    : Spatial and temporal seeds for randomness\n"
  "    seed_spatial = 1    : Seed for spatial variation (different synapses)\n"
  "    seed_temporal = 1   : Seed for temporal variation (different events)\n"
  "    \n"
  "    : Synaptic parameters\n"
  "    tau1 = 0.1 (ms) <1e-9,1e9>  : Rise time constant\n"
  "    tau2 = 10 (ms) <1e-9,1e9>   : Decay time constant  \n"
  "    e = 0 (mV)                  : Reversal potential\n"
  "    release_prob = 0.5 <0,1>    : Release probability [0,1]\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    g (uS)\n"
  "    factor\n"
  "    event_count  : Counter for temporal randomness\n"
  "}\n"
  "\n"
  "STATE {\n"
  "    A (uS)\n"
  "    B (uS)\n"
  "}\n"
  "\n"
  "INITIAL { \n"
  "    LOCAL tp\n"
  "    \n"
  "    : Initialize event counter\n"
  "    event_count = 0\n"
  "    \n"
  "    : Set the random seed combining spatial and temporal components\n"
  "    set_seed(seed_spatial * 1009 + seed_temporal * 1013)\n"
  "    \n"
  "    : Calculate normalization factor\n"
  "    if (tau1 / tau2 > 0.9999) {\n"
  "        tau1 = 0.9999 * tau2\n"
  "    }\n"
  "    if (tau1 / tau2 < 1e-9) {\n"
  "        tau1 = tau2 * 1e-9\n"
  "    }\n"
  "    tp = (tau1 * tau2) / (tau2 - tau1) * log(tau2 / tau1)\n"
  "    factor = -exp(-tp / tau1) + exp(-tp / tau2)\n"
  "    factor = 1 / factor\n"
  "\n"
  "    A = 0\n"
  "    B = 0 \n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    SOLVE state METHOD cnexp\n"
  "    g = B - A\n"
  "    i = g * (v - e)\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "    A' = -A / tau1\n"
  "    B' = -B / tau2\n"
  "}\n"
  "\n"
  "NET_RECEIVE(weight (uS)) {\n"
  "    LOCAL rval\n"
  "    \n"
  "    : Generate uniform random number using NEURON's scop_random()\n"
  "    rval = scop_random()\n"
  "    \n"
  "    if (rval < release_prob) {\n"
  "        A = A + weight * factor\n"
  "        B = B + weight * factor\n"
  "        VERBATIM\n"
  "		// printf(\"RELEASED SPIKES \\n\");\n"
  "		ENDVERBATIM\n"
  "         \n"
  "    }\n"
  "    else{\n"
  "      	VERBATIM\n"
  "		// printf(\"FAILED TO RELEASE SPIKES \\n\");\n"
  "		ENDVERBATIM\n"
  "    }\n"
  "    \n"
  "    : Increment event counter for temporal variation\n"
  "    event_count = event_count + 1\n"
  "}\n"
  "\n"
  ": Procedure to set seeds - reinitializes the random number generator\n"
  "PROCEDURE setSeeds(spatial, temporal) {\n"
  "    seed_spatial = spatial\n"
  "    seed_temporal = temporal\n"
  "    event_count = 0\n"
  "    \n"
  "    : Reset NEURON's random seed with combined seed\n"
  "    set_seed(seed_spatial * 1009 + seed_temporal * 1013)\n"
  "}\n"
  "\n"
  ": Function to get current event count\n"
  "FUNCTION getEventCount() {\n"
  "    getEventCount = event_count\n"
  "}\n"
  "\n"
  ": Reset event counter\n"
  "PROCEDURE resetCounter() {\n"
  "    event_count = 0\n"
  "}\n"
  "\n"
  ": Generate a random number on demand (useful for testing)\n"
  "FUNCTION getRandomNumber() {\n"
  "    getRandomNumber = scop_random()\n"
  "}\n"
  "\n"
  ": Procedure to advance the random stream by n steps\n"
  "PROCEDURE advanceRandom(n) {\n"
  "    LOCAL i, dummy\n"
  "    FROM i = 1 TO n {\n"
  "        dummy = scop_random()\n"
  "    }\n"
  "} \n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
