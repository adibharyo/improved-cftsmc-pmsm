#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME PMSM
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 

	ssSetNumContStates(S, 4); 
	
	if (!ssSetNumInputPorts(S, 1)) return; 
	
	ssSetInputPortWidth(S, 0, 4); 
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	
	if (!ssSetNumOutputPorts(S, 1)) return; 
	
	ssSetOutputPortWidth(S, 0, 7); 
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
	
	// PMSM Dynamics
	real_T N	= 4;
	real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    
    real_T Ia, Ib, Ic;
    real_T I_alpha, I_beta;
    real_T Id, Iq;
    real_T omega_m, theta_m;
    real_T theta_e;
    real_T Te, Tl;
    real_T pi = 3.141592654;
    
    real_T c = 0.816497;
    real_T x = 0.866025;
    
    // State Variables
    omega_m = X[0];
    theta_e = X[1];
    Id = X[2];
    Iq = X[3];
    
    // DQ to alpha-beta
    I_alpha = Id*cos(theta_e) - Iq*sin(theta_e);
    I_beta = Id*sin(theta_e) + Iq*cos(theta_e);
    
    // Alpha-beta to 3 phase
    Ia = c*I_alpha;
    Ib = c*(-0.5*I_alpha + x*I_beta);
    Ic = c*(-0.5*I_alpha - x*I_beta);
    
    Te = N*(psi + Id*(Lsd - Lsq))*Iq;
    theta_m = theta_e / N;
    
    Y[0] = Ia;
    Y[1] = Ib;
    Y[2] = Ic;
    Y[3] = Te;
    Y[4] = omega_m;
    Y[5] = theta_m;
    Y[6] = theta_e;

} 

#define MDL_DERIVATIVES 
static void mdlDerivatives(SimStruct *S) { 
	
	real_T *dX = ssGetdX(S); 
	real_T *X = ssGetContStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	
	// PMSM Dynamics
	real_T N	= 4;
	real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    
    real_T Va, Vb, Vc;
    real_T V_alpha, V_beta;
    real_T Vd, Vq;
    real_T Id, Iq, Id_dot, Iq_dot;
    real_T omega_m, omega_m_dot, theta_m;
    real_T theta_e, theta_e_dot;
    real_T Te, Tl;
    
    real_T c = 0.816497; 		// sqrt(2/3) 
    real_T x = 0.866025;		// sqrt(3)/2
    
    // Input
	Va = U(0);
    Vb = U(1);
    Vc = U(2);
    Tl = U(3);
    
    // State Variables
    omega_m = X[0];
    theta_e = X[1];
    Id = X[2];
    Iq = X[3];
    
    theta_e_dot = N * omega_m;
    
    // 3 phase to 2 phase (alpha-beta)
    V_alpha = c*(Va - 0.5*Vb - 0.5*Vc); 
    V_beta = c*(x*Vb - x*Vc);
    
    // Alpha-beta to DQ
    Vd = (V_alpha * cos(theta_e)) + (V_beta * sin(theta_e));
    Vq = (-V_alpha * sin(theta_e)) + (V_beta * cos(theta_e));
    
    // State equation
    Id_dot = (1/Lsd)*(-Rs*Id + N*omega_m*Lsq*Iq + Vd);
    Iq_dot = (1/Lsq)*(-Rs*Iq - N*omega_m*(Lsd*Id + psi) + Vq);
    
    Te = N*(psi + Id*(Lsd - Lsq))*Iq;
    omega_m_dot = (Te - Tl)/J;
    
    // State Derivatives
    dX[0] = omega_m_dot;
    dX[1] = theta_e_dot;
    dX[2] = Id_dot;
    dX[3] = Iq_dot;
    
} 

static void mdlTerminate(SimStruct *S) 
{} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif 
