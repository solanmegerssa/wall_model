#include "__cf_new_wall_model.h"
#ifndef RTW_HEADER_new_wall_model_acc_private_h_
#define RTW_HEADER_new_wall_model_acc_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#if !defined(ss_VALIDATE_MEMORY)
#define ss_VALIDATE_MEMORY(S, ptr)   if(!(ptr)) {\
  ssSetErrorStatus(S, RT_MEMORY_ALLOCATION_ERROR);\
  }
#endif
#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((ptr));\
  (ptr) = (NULL);\
  }
#else
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((void *)(ptr));\
  (ptr) = (NULL);\
  }
#endif
#endif
#ifndef rtInterpolate
#define rtInterpolate(v1,v2,f1,f2)   (((v1)==(v2))?((double)(v1)):  (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif
#ifndef rtRound
#define rtRound(v) ( ((v) >= 0) ?   muDoubleScalarFloor((v) + 0.5) :   muDoubleScalarCeil((v) - 0.5) )
#endif
#ifndef __RTW_UTFREE__
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T * bufSzPtr
, int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T tMinusDelay ,
real_T * * tBufPtr , real_T * * uBufPtr , real_T * * xBufPtr , boolean_T
isfixedbuf , boolean_T istransportdelay , int_T * maxNewBufSzPtr ) ; real_T
new_wall_model_acc_rt_VTDelayfindtDInterpolate ( real_T x , real_T * tBuf ,
real_T * uBuf , real_T * xBuf , int_T bufSz , int_T head , int_T tail , int_T
* pLast , real_T t , real_T tStart , boolean_T discrete , boolean_T
minorStepAndTAtLastMajorOutput , real_T initOutput , real_T * appliedDelay )
; void new_wall_model_acc_BINARYSEARCH_real_T ( uint32_T * piLeft , uint32_T
* piRght , real_T u , const real_T * pData , uint32_T iHi ) ; void
new_wall_model_acc_LookUp_real_T_real_T ( real_T * pY , const real_T * pYData
, real_T u , const real_T * pUData , uint32_T iHi ) ; real_T
new_wall_model_acc_rt_TDelayInterpolate ( real_T tMinusDelay , real_T tStart
, real_T * tBuf , real_T * uBuf , int_T bufSz , int_T * lastIdx , int_T
oldestIdx , int_T newIdx , real_T initOutput , boolean_T discrete , boolean_T
minorStepAndTAtLastMajorOutput ) ;
#endif
