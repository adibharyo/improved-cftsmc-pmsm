#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME SMC2
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 

	ssSetNumDiscStates(S, 3); 
	
	if (!ssSetNumInputPorts(S, 1)) return; 
	
	ssSetInputPortWidth(S, 0, 5); 
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	
	if (!ssSetNumOutputPorts(S, 1)) return; 
	
	ssSetOutputPortWidth(S, 0, 2); 
	ssSetNumSampleTimes(S, 1); 

	ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_DISCRETE_VALUED_OUTPUT));
	
} 

static void mdlInitializeSampleTimes(SimStruct *S){ 

	ssSetSampleTime(S, 0, 1e-4); 
	ssSetOffsetTime(S, 0, 0.0);

} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S){ 

	real_T *X0 = ssGetRealDiscStates(S); 
	int_T nXStates = ssGetNumDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	int_T i; 

	/* initialize the states to 0.0 */ 
	for (i=0; i < nXStates; i++) { 
		X0[i] = 0.0; 
	} 
	
} 

static void mdlOutputs(SimStruct *S, int_T tid){ 

	real_T *Y = ssGetOutputPortRealSignal(S,0); 
	real_T *X = ssGetRealDiscStates(S); 

	Y[0] = X[1]; // q-Current Reference
	Y[1] = X[2]; // Sliding Surface

} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) { 

	real_T *X = ssGetRealDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

	real_T dt = 1e-4;
	
	real_T a, b, c;
	real_T Np  = 4.0;
	real_T psi = 0.121;
	real_T J   = 0.01;
	real_T B   = 0.00;

	real_T error_omega, error_itgr_old, error_itgr, omega_ref, omega;
	real_T sigma, mu, sgn_sigma;
	real_T Isqr;
	
	omega_ref = U(0);
	omega     = U(1);
	
	real_T c_omega = U(2);
	real_T gamma   = U(3);
	real_T phi     = U(4);
	
	a = (Np*Np*psi)/J;
	b = B/J;
	c = Np/J;
	
	error_omega = omega - omega_ref;
	
	error_itgr_old = X[0];
	error_itgr     = error_itgr_old + error_omega*dt;
	X[0]           = error_itgr;
	
	sigma = error_omega + c_omega*error_itgr;
	
	if(sigma > 0){
		sgn_sigma = 1;
	}
	else if(sigma < 0){
		sgn_sigma = -1;
	}
	else{
		sgn_sigma = 0;
	}
	
	mu = (1/a)*(b*fabs(omega) + c*phi + c_omega*fabs(error_omega));
	
	Isqr = -gamma*sigma - mu*sgn_sigma;
	
	X[1] = Isqr;
	X[2] = sigma;
}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 

