#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME PMSM
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 
	ssSetNumContStates(S, 4); // 4 CONTINUOUS STATE
	if (!ssSetNumInputPorts(S, 1)) return; 
	ssSetInputPortWidth(S, 0, 4); // 4 INPUT
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	if (!ssSetNumOutputPorts(S, 1)) return; 
	ssSetOutputPortWidth(S, 0, 7); // 7 OUTPUT
	ssSetNumSampleTimes(S, 1); 
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); 
} 

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); 
	ssSetOffsetTime(S, 0, 0.0);
} 

#define MDL_INITIALIZE_CONDITIONS 
static void mdlInitializeConditions(SimStruct *S) { 

	real_T *X0 = ssGetContStates(S); 
	int_T nStates = ssGetNumContStates(S); 
	int_T i; 

	/* initialize the states to 0.0 */ 
	for (i=0; i < nStates; i++) {
		X0[i] = 0.0;
	}
} 

static void mdlOutputs(SimStruct *S, int_T tid) { 
	real_T *Y = ssGetOutputPortRealSignal(S,0); 
	real_T *X = ssGetContStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	
	// PMSM MODEL'S PARAMETER
     real_T N    = 4;
    real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    real_T B    = 0.08;
    
    real_T Ia, Ib, Ic;
    real_T I_alpha, I_beta;
    real_T Isd, Isq;
    real_T omega_m, theta_m;
    real_T theta_e;
    real_T Te, TL;
    real_T pi = 3.141592654;
    
    real_T K = 0.8164965809; // akar(2/3)
    real_T L = 0.8660254038; // akar(3/2)
    
    
     // STATE VARIABLE
    Isd = X[0];
    Isq = X[1];
    theta_e = X[2];
    omega_m = X[3];
    
    
   // TRANSFORM DQ TO ALPHA-BETA
    I_alpha = Isd * cos(theta_e) - Isq * sin(theta_e);
    I_beta = Isd * sin(theta_e) + Isq * cos(theta_e);
    
    
    // TRANSFORM ALPHA-BETA TO ABC
    Ia = K * I_alpha;
    Ib = K * (-0.5 * I_alpha + L * I_beta);
    Ic = K * (-0.5 * I_alpha - L * I_beta);
    
    
    // OUTPUT
    Te = N * (psi * Isq + (Lsd - Lsq) * Isd * Isq);
    
    theta_m = theta_e / N;
    
    // Ia
    Y[0] = Ia;
    
    // Ib
    Y[1] = Ib;
    
    // Ic
    Y[2] = Ic;
    
    // theta_e
    Y[3] = theta_e;
    
    // theta_m
    Y[4] = theta_m;
    
    // Te
    Y[5] = Te;
    
    // omega_m
    Y[6] = omega_m;

} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
	
	real_T *dX = ssGetdX(S); 
	real_T *X = ssGetContStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	
	// PMSM MODEL'S PARAMETER

    real_T N    = 4;
    real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    real_T B    = 0.08;
    
    real_T Isd_dot, Isq_dot;
    real_T Isd, Isq;
    real_T omega_m, omega_m_dot;
    real_T theta_m, theta_m_dot;
    real_T theta_e, theta_e_dot;  
    
    real_T Te, TL;
    
    real_T Va, Vb, Vc;
    real_T V_alfa, V_beta;
    real_T Vsd, Vsq;
    
    real_T K = 0.8164965809; // akar(2/3)
    real_T L = 0.8660254038; // akar(3/2)
    
    // INPUT
	Va = U(0);
    Vb = U(1);
    Vc = U(2);
    TL = U(3);
    
    
    // STATE VARIABLE
    Isd = X[0];
    Isq = X[1];
    theta_e = X[2];
    omega_m = X[3];
    
    // TRANSFORM
    //ABC TO ALPHA BETA
    V_alfa = K * (Va - 0.5 * Vb - 0.5 * Vc);
    V_beta = K * L * (Vb - Vc);
    
    // Alpha-beta to DQ
    Vsd = cos(theta_e) * V_alfa + sin(theta_e) * V_beta;
    Vsq = -sin(theta_e) * V_alfa + cos(theta_e) * V_beta;
    
    
    // PMSM MODEL
    // State Equation
    Isd_dot = -(Rs / Lsd) * Isd + (N * omega_m * (Lsq / Lsd) * Isq) + (1 / Lsd) * Vsd;
    Isq_dot = (-N * omega_m * (Lsd / Lsq) * Isd) - (N * omega_m * psi / Lsq) - (Rs / Lsq) * Isq + (1 / Lsq) * Vsq;
    
    Te = N * (psi * Isq + (Lsd - Lsq) * Isd * Isq);
    
    omega_m_dot = (Te - B * omega_m - TL) / J;
    
    theta_e_dot = N * omega_m;
    
    // State Derivatives
    dX[0] = Isd_dot;
    dX[1] = Isq_dot;
    dX[2] = theta_e_dot;
    dX[3] = omega_m_dot;
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 