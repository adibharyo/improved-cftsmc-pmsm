#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME CFTSMC
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
	ssSetNumDiscStates(S, 6); // 5 DISCRETE STATE USED
	if (!ssSetNumInputPorts(S, 1)) return; 
	ssSetInputPortWidth(S, 0, 12); // 3 INPUT
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	if (!ssSetNumOutputPorts(S, 1)) return; 
	ssSetOutputPortWidth(S, 0, 3); // 3 OUTPUT
	ssSetNumSampleTimes(S, 1); 

	ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE 
	| SS_OPTION_DISCRETE_VALUED_OUTPUT));
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

    real_T Iq_ref, es, error;
    
    Iq_ref = X[3];
    es = X[4];
    error = X[5];
    
	Y[0] = Iq_ref;
    
    Y[1] = es;
    
    Y[2] = error;
} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) {
	real_T *X = ssGetRealDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

	real_T dt = 1e-4;

    // PMSM MODEL'S PARAMETER
    real_T N    = 4;
    real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    real_T B    = 0.08;
    
    // INPUT
    real_T omega_ref, omega_act, lambda_1, lambda_2, alpha_1, alpha_2, alpha_3, k1, k2;
    omega_ref = U(0);
    omega_act = U(1);
    lambda_1 = U(5);
    lambda_2 = U(6);
    alpha_1 = U(7);
    alpha_2 = U(8);
    alpha_3 = U(9);
    k1 = U(10);
    k2 = U(11);
    
    
    real_T pt, m, m_inv;
    pt = (3 * N * psi) / 2;
    m = pt / J;
    m_inv = J / pt;
    
    
	real_T error, error_dot;
    real_T es, sign_s;
    real_T sign_error, sign_error_dot;
    real_T omega_ref_old, omega_act_old;
    real_T X_eq, Xb;
    real_T itgr_Xb_old, itgr_Xb;
    real_T Iq_ref;

	// STATE
    omega_ref_old = X[0];
    omega_act_old = X[1];
    itgr_Xb_old = X[2];
	
    // IP ALGORITHM
	error = omega_ref - omega_act; 
    error_dot = ((omega_ref - omega_ref_old) / dt) - ((omega_act - omega_act_old) / dt); 
    
    if (error_dot > 0) {
        sign_error_dot = 1;
    }
    
    else if (error_dot < 0) { 
        sign_error_dot = -1;
    }
    
    else {
       sign_error_dot = 0; 
    }
    
    if (error > 0) {
        sign_error = 1;
    }
    
    else if (error < 0) {
        sign_error = -1;
    }
    
    else {
        sign_error = 0;
    }
    
    if (es > 0) {
        sign_s = 1;
    }
    
    else if (es < 0) {
        sign_s = -1;
    }
    
    else {
        sign_s = 0;
    }
    
    es = error_dot + lambda_1 * pow(fabs(error_dot), alpha_1) * sign_error_dot + lambda_2 * pow(fabs(error), alpha_2) * sign_error;
    
    itgr_Xb = k1 * es + k2 * pow(fabs(es), alpha_3) * sign_s;
    
    X_eq = ((omega_ref - omega_ref_old) / dt) + lambda_1 * pow(fabs(error_dot), alpha_1) * sign_error_dot + lambda_2 * pow(fabs(error), alpha_2) * sign_error;

    Xb = itgr_Xb_old + itgr_Xb * dt;
    
    Iq_ref = m_inv * (X_eq + Xb);
    
	X[0] = omega_ref;
	X[1] = omega_act;	
    X[2] = itgr_Xb;
    X[3] = Iq_ref;
    X[4] = es;
    X[5] = error;
}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif
