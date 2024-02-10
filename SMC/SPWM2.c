#define S_FUNCTION_LEVEL 2 
#define S_FUNCTION_NAME SPWM2
#include "simstruc.h" 
#include <math.h> 

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/ 


static void mdlInitializeSizes(SimStruct *S){
 
	if (!ssSetNumInputPorts(S, 1)) return;
	 
	ssSetInputPortWidth(S, 0, 4); 
	ssSetInputPortDirectFeedThrough(S, 0, 1); 
	ssSetInputPortOverWritable(S, 0, 1); 
	
	if (!ssSetNumOutputPorts(S, 1)) return;
	 
	ssSetOutputPortWidth(S, 0, 4); 
	ssSetNumSampleTimes(S, 1); 

	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); 
} 

static void mdlInitializeSampleTimes(SimStruct *S) { 

	ssSetSampleTime(S, 0, 1e-5); 
	ssSetOffsetTime(S, 0, 0.0); 
} 

static void mdlOutputs(SimStruct *S, int_T tid) { 

	real_T *Y = ssGetOutputPortRealSignal(S,0); 
	InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); 
	
	int_T i = 0;
	real_T t = ssGetT(S);
	real_T Tc = 0.001;
	real_T Ac = U(0);
	
	real_T modulator = (1/Tc) * fabs(fmod(t,Tc));
	
	for( i=0; i < 3; i++) {
		
		if(fabs(U(i+1))/Ac >= modulator){
			Y[i+1] = Ac; 
		}
		
		else{
			Y[i+1] = 0.0; 	
		}
		
		if( U(i+1) < 0.0 ){ 
			Y[i+1] = -Y[i+1] ; 
		}
	}
	
	Y[0] = modulator; 
}

static void mdlTerminate(SimStruct *S) {
	
} /*Keep this function empty since no memory is allocated*/ 

#ifdef MATLAB_MEX_FILE 
/* Is this file being compiled as a MEX-file? */ 
#include "simulink.c" /* MEX-file interface mechanism */ 
#else 
#include "cg_sfun.h" /*Code generation registration function*/ 
#endif
