#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME FOC1
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 

static void mdlInitializeSizes(SimStruct *S){ 

	ssSetNumDiscStates(S, 6); 
	
	if (!ssSetNumInputPorts(S, 1)) return; 
	
	ssSetInputPortWidth(S, 0, 5); 
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	
	if (!ssSetNumOutputPorts(S, 1)) return; 
				
	ssSetOutputPortWidth(S, 0, 3); 
	ssSetNumSampleTimes(S, 1); 

	ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE 
	| SS_OPTION_DISCRETE_VALUED_OUTPUT));
	
} 

static void mdlInitializeSampleTimes(SimStruct *S){ 

	ssSetSampleTime(S, 0, 1e-3); 
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
	
	Y[0] = X[3]; // Va
	Y[1] = X[4]; // Vb
	Y[2] = X[5]; // Vc

} 

#define MDL_UPDATE 
static void mdlUpdate(SimStruct *S, int_T tid) {
 
	real_T *X = ssGetRealDiscStates(S); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 

	real_T dt = 1e-3;
	
	real_T N	= 4;
	real_T psi  = 0.121;
    real_T Lsd  = 16.61e-3;
    real_T Lsq  = 16.22e-3;
    real_T Rs   = 0.55;
    real_T J    = 0.01;
    real_T Td   = 0.01;
    
    real_T Ia, Ib, Ic;
    real_T I_alpha, I_beta;
    real_T Id, Iq;
    real_T error_Id, error_Iq;
	real_T integral_Id, integral_Iq;
	real_T integral_Id_old, integral_Iq_old;
	real_T id_new, iq_new, id_old, iq_old;
    real_T omega_m, theta_m;
    real_T theta_e, theta_e_old;
    real_T ud, uq, Vd, Vq;
    real_T V_alpha, V_beta;
    real_T Va, Vb, Vc;
    real_T Kpq, Kiq, Kpd, Kid;
    real_T pi = 3.141592654;
    
    real_T c = 0.816497;
    real_T x = 0.866025;
	
	// Current feedback
	Ia = U(1);
	Ib = U(2);
	Ic = U(3);
	
	// Position feedback
	theta_e = U(4);
	
	// Current reference
	real_T Iq_ref = U(0);
	real_T Id_ref = 0.0;

	// Three-phase to Alpha-beta
	I_alpha = c*(Ia - 0.5*Ib - 0.5*Ic); 
    I_beta = c*(x*Ib - x*Ic);
	
	// Alpha-beta to DQ
	Id = (I_alpha * cos(theta_e)) + (I_beta * sin(theta_e));
    Iq = (-I_alpha * sin(theta_e)) + (I_beta * cos(theta_e));   
    
    // Current controller
    integral_Iq_old = X[0];
    integral_Id_old = X[1];
    theta_e_old = X[2];
    
    error_Iq = Iq_ref - Iq;
    error_Id = Id_ref - Id;
    
    integral_Iq = integral_Iq_old + error_Iq*dt;
    integral_Id = integral_Id_old + error_Id*dt;
    
    Kpq = 1;
    Kiq = 350;
    Kpd = 1;
    Kid = 350;
    
    Vq = Kpq*error_Iq + Kiq*integral_Iq;
    Vd = Kpd*error_Id + Kid*integral_Id;

    // Decoupling
    omega_m = (theta_e - theta_e_old)/(dt * N);
    
    iq_new = iq_old + (dt*(Iq_ref - iq_old))/Td;
	id_new = id_old + (dt*(Id_ref - id_old))/Td;
	
	Vq = uq + N*omega_m*(Lsd*id_new + psi);
	Vd = ud - N*omega_m*Lsq*iq_new;
    
    // DQ to Alpha-beta
    V_alpha = Vd*cos(theta_e) - Vq*sin(theta_e);
    V_beta = Vd*sin(theta_e) + Vq*cos(theta_e);
    
    // Alpha-beta to Three-phase
    Va = c*V_alpha;
    Vb = c*(-0.5*V_alpha + x*V_beta);
    Vc = c*(-0.5*V_alpha - x*V_beta);
    
    // Update
    X[0] = integral_Iq;
    X[1] = integral_Id;
    X[2] = theta_e;
    X[3] = Va;
    X[4] = Vb;
    X[5] = Vc;
    
}

static void mdlTerminate(SimStruct *S) 
{ } /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /*MEX-file interface mechanism*/ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif
