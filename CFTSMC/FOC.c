#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME FOC
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){
	ssSetNumDiscStates(S, 8); // 8 DISCRETE STATE USED
	if (!ssSetNumInputPorts(S, 1)) return; 
	ssSetInputPortWidth(S, 0, 8); // 6 INPUT
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
	
    // OUTPUT
    real_T Va, Vb, Vc;
    
    Va = X[5];
    Vb = X[6];
    Vc = X[7];
    
    Y[0] = Va;
    Y[1] = Vb;
    Y[2] = Vc;

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
    real_T Td   = 0.01;
    
    
    // DECLARE VARIABLE
    real_T Isd_ref, Isq_ref;
    
    real_T Ia, Ib, Ic;
    real_T I_alpha, I_beta;
    real_T Isd_act, Isq_act;
    
    real_T Va, Vb, Vc;
    real_T V_alpha, V_beta;
    real_T Vsd, Vsq;
    
    real_T error_Isd, error_Isq;
	real_T integral_Isd, integral_Isq;
	real_T integral_Isd_old, integral_Isq_old;
    real_T Kpd, Kpq, Kid, Kiq;
    real_T usd, usq;
    real_T omega_m;
    real_T theta_e_act, theta_e_old;
	real_T Isd_new, Isq_new, Isd_old, Isq_old;
    
    real_T pi = 3.141592654;

    real_T K = 0.8164965809; // akar(2/3)
    real_T L = 0.8660254038; // akar(3/2)
	
    
    // INPUT
	// Current feedback
	Ia = U(0);
    Ib = U(1);
    Ic = U(2);
    
    // Current reference
    Isd_ref = U(3);
    Isq_ref = U(4);
    
    // Position feedback
    theta_e_act = U(5);
    
	// TRANSFORM ABC TO ALPHA-BETA
	I_alpha = K * (Ia - 0.5 * Ib - 0.5 * Ic);
    I_beta = K * L * (Ib - Ic);
	
	// TRANSFORM ALPHA-BETA TO DQ
	Isd_act = cos(theta_e_act) * I_alpha + sin(theta_e_act) * I_beta;
    Isq_act = -sin(theta_e_act) * I_alpha + cos(theta_e_act) * I_beta;
    
    // STATE
    integral_Isd_old = X[0];
    integral_Isq_old = X[1];
    theta_e_old = X[2];
    Isd_old = X[3];
    Isq_old = X[4];
    
    error_Isd = Isd_ref - Isd_act;
    error_Isq = Isq_ref - Isq_act;
    
    integral_Isd = integral_Isd_old + error_Isd * dt;
    integral_Isq = integral_Isq_old + error_Isq * dt;
    
    Kpd = U(6);
    Kid = U(7);
    Kpq = U(6);
    Kiq = U(7);
    
    usd = Kpd * error_Isd + Kid * integral_Isd;
    usq = Kpq * error_Isq + Kiq * integral_Isq;
    
    // COUPLING
    omega_m = (theta_e_act - theta_e_old) / (dt * N);
    
    Isd_new = Isd_old + (dt * (Isd_ref - Isd_old)) / Td;
    Isq_new = Isq_old + (dt * (Isq_ref - Isq_old)) / Td;

	Vsd = usd  - (N * omega_m * Lsq * Isq_new);
    Vsq = usq + (N * omega_m * Lsd * Isd_new) + N * omega_m * psi;
    
    
    // TRANSFORM DQ TO ALFA-BETA
    V_alpha = Vsd * cos(theta_e_act) - Vsq * sin(theta_e_act);
    V_beta = Vsd * sin(theta_e_act) + Vsq * cos(theta_e_act);
    
    // TRANSFORM ALFA-BETA TO ABC  
    Va = K * V_alpha;
    Vb = K * (-0.5 * V_alpha) + L * V_beta;
    Vc = K * (-0.5 * V_alpha) - L * V_beta;
    
    
    // UPDATE STATE
    X[0] = integral_Isd;
    X[1] = integral_Isq;
    X[2] = theta_e_act;
    X[3] = Isd_new;
    X[4] = Isq_new;
    X[5] = Va;
    X[6] = Vb;
    X[7] = Vc;
}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif
