#include "__cf_new_wall_model.h"
#include <math.h>
#include "new_wall_model_acc.h"
#include "new_wall_model_acc_private.h"
#include <stdio.h>
#include "slexec_vm_simstruct_bridge.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_lookup_functions.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#include "simtarget/slAccSfcnBridge.h"
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T * bufSzPtr
, int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T tMinusDelay ,
real_T * * tBufPtr , real_T * * uBufPtr , real_T * * xBufPtr , boolean_T
isfixedbuf , boolean_T istransportdelay , int_T * maxNewBufSzPtr ) { int_T
testIdx ; int_T tail = * tailPtr ; int_T bufSz = * bufSzPtr ; real_T * tBuf =
* tBufPtr ; real_T * xBuf = ( NULL ) ; int_T numBuffer = 2 ; if (
istransportdelay ) { numBuffer = 3 ; xBuf = * xBufPtr ; } testIdx = ( tail <
( bufSz - 1 ) ) ? ( tail + 1 ) : 0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] )
&& ! isfixedbuf ) { int_T j ; real_T * tempT ; real_T * tempU ; real_T *
tempX = ( NULL ) ; real_T * uBuf = * uBufPtr ; int_T newBufSz = bufSz + 1024
; if ( newBufSz > * maxNewBufSzPtr ) { * maxNewBufSzPtr = newBufSz ; } tempU
= ( real_T * ) utMalloc ( numBuffer * newBufSz * sizeof ( real_T ) ) ; if (
tempU == ( NULL ) ) { return ( false ) ; } tempT = tempU + newBufSz ; if (
istransportdelay ) tempX = tempT + newBufSz ; for ( j = tail ; j < bufSz ; j
++ ) { tempT [ j - tail ] = tBuf [ j ] ; tempU [ j - tail ] = uBuf [ j ] ; if
( istransportdelay ) tempX [ j - tail ] = xBuf [ j ] ; } for ( j = 0 ; j <
tail ; j ++ ) { tempT [ j + bufSz - tail ] = tBuf [ j ] ; tempU [ j + bufSz -
tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j + bufSz - tail ] =
xBuf [ j ] ; } if ( * lastPtr > tail ) { * lastPtr -= tail ; } else { *
lastPtr += ( bufSz - tail ) ; } * tailPtr = 0 ; * headPtr = bufSz ; utFree (
uBuf ) ; * bufSzPtr = newBufSz ; * tBufPtr = tempT ; * uBufPtr = tempU ; if (
istransportdelay ) * xBufPtr = tempX ; } else { * tailPtr = testIdx ; }
return ( true ) ; } real_T new_wall_model_acc_rt_VTDelayfindtDInterpolate (
real_T x , real_T * tBuf , real_T * uBuf , real_T * xBuf , int_T bufSz ,
int_T head , int_T tail , int_T * pLast , real_T t , real_T tStart ,
boolean_T discrete , boolean_T minorStepAndTAtLastMajorOutput , real_T
initOutput , real_T * appliedDelay ) { int_T n , k ; real_T f ; int_T kp1 ;
real_T tminustD , tL , tR , uD , uL , uR , fU ; if (
minorStepAndTAtLastMajorOutput ) { if ( * pLast == head ) { * pLast = ( *
pLast == 0 ) ? bufSz - 1 : * pLast - 1 ; } head = ( head == 0 ) ? bufSz - 1 :
head - 1 ; } if ( x <= 1 ) { return initOutput ; } k = * pLast ; n = 0 ; for
( ; ; ) { n ++ ; if ( n > bufSz ) break ; if ( x - xBuf [ k ] > 1.0 ) { if (
k == head ) { int_T km1 ; f = ( x - 1.0 - xBuf [ k ] ) / ( x - xBuf [ k ] ) ;
tminustD = ( 1.0 - f ) * tBuf [ k ] + f * t ; km1 = k - 1 ; if ( km1 < 0 )
km1 = bufSz - 1 ; tL = tBuf [ km1 ] ; tR = tBuf [ k ] ; uL = uBuf [ km1 ] ;
uR = uBuf [ k ] ; break ; } kp1 = k + 1 ; if ( kp1 == bufSz ) kp1 = 0 ; if (
x - xBuf [ kp1 ] <= 1.0 ) { f = ( x - 1.0 - xBuf [ k ] ) / ( xBuf [ kp1 ] -
xBuf [ k ] ) ; tL = tBuf [ k ] ; tR = tBuf [ kp1 ] ; uL = uBuf [ k ] ; uR =
uBuf [ kp1 ] ; tminustD = ( 1.0 - f ) * tL + f * tR ; break ; } k = kp1 ; }
else { if ( k == tail ) { f = ( x - 1.0 ) / xBuf [ k ] ; if ( discrete ) {
return ( uBuf [ tail ] ) ; } kp1 = k + 1 ; if ( kp1 == bufSz ) kp1 = 0 ;
tminustD = ( 1 - f ) * tStart + f * tBuf [ k ] ; tL = tBuf [ k ] ; tR = tBuf
[ kp1 ] ; uL = uBuf [ k ] ; uR = uBuf [ kp1 ] ; break ; } k = k - 1 ; if ( k
< 0 ) k = bufSz - 1 ; } } * pLast = k ; if ( tR == tL ) { fU = 1.0 ; } else {
fU = ( tminustD - tL ) / ( tR - tL ) ; } if ( discrete ) { uD = ( fU > ( 1.0
- fU ) ) ? uR : uL ; } else { uD = ( 1.0 - fU ) * uL + fU * uR ; } *
appliedDelay = t - tminustD ; return uD ; } void
new_wall_model_acc_BINARYSEARCH_real_T ( uint32_T * piLeft , uint32_T *
piRght , real_T u , const real_T * pData , uint32_T iHi ) { * piLeft = 0U ; *
piRght = iHi ; if ( u <= pData [ 0 ] ) { * piRght = 0U ; } else if ( u >=
pData [ iHi ] ) { * piLeft = iHi ; } else { uint32_T i ; while ( ( * piRght -
* piLeft ) > 1U ) { i = ( * piLeft + * piRght ) >> 1 ; if ( u < pData [ i ] )
{ * piRght = i ; } else { * piLeft = i ; } } } } void
new_wall_model_acc_LookUp_real_T_real_T ( real_T * pY , const real_T * pYData
, real_T u , const real_T * pUData , uint32_T iHi ) { uint32_T iLeft ;
uint32_T iRght ; new_wall_model_acc_BINARYSEARCH_real_T ( & ( iLeft ) , & (
iRght ) , u , pUData , iHi ) ; { real_T lambda ; if ( pUData [ iRght ] >
pUData [ iLeft ] ) { real_T num ; real_T den ; den = pUData [ iRght ] ; den
-= pUData [ iLeft ] ; num = u ; num -= pUData [ iLeft ] ; lambda = num / den
; } else { lambda = 0.0 ; } { real_T yLeftCast ; real_T yRghtCast ; yLeftCast
= pYData [ iLeft ] ; yRghtCast = pYData [ iRght ] ; yLeftCast += lambda * (
yRghtCast - yLeftCast ) ; ( * pY ) = yLeftCast ; } } } real_T
new_wall_model_acc_rt_TDelayInterpolate ( real_T tMinusDelay , real_T tStart
, real_T * tBuf , real_T * uBuf , int_T bufSz , int_T * lastIdx , int_T
oldestIdx , int_T newIdx , real_T initOutput , boolean_T discrete , boolean_T
minorStepAndTAtLastMajorOutput ) { int_T i ; real_T yout , t1 , t2 , u1 , u2
; if ( ( newIdx == 0 ) && ( oldestIdx == 0 ) && ( tMinusDelay > tStart ) )
return initOutput ; if ( tMinusDelay <= tStart ) return initOutput ; if ( (
tMinusDelay <= tBuf [ oldestIdx ] ) ) { if ( discrete ) { return ( uBuf [
oldestIdx ] ) ; } else { int_T tempIdx = oldestIdx + 1 ; if ( oldestIdx ==
bufSz - 1 ) tempIdx = 0 ; t1 = tBuf [ oldestIdx ] ; t2 = tBuf [ tempIdx ] ;
u1 = uBuf [ oldestIdx ] ; u2 = uBuf [ tempIdx ] ; if ( t2 == t1 ) { if (
tMinusDelay >= t2 ) { yout = u2 ; } else { yout = u1 ; } } else { real_T f1 =
( t2 - tMinusDelay ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; yout = f1 * u1 +
f2 * u2 ; } return yout ; } } if ( minorStepAndTAtLastMajorOutput ) { if (
newIdx != 0 ) { if ( * lastIdx == newIdx ) { ( * lastIdx ) -- ; } newIdx -- ;
} else { if ( * lastIdx == newIdx ) { * lastIdx = bufSz - 1 ; } newIdx =
bufSz - 1 ; } } i = * lastIdx ; if ( tBuf [ i ] < tMinusDelay ) { while (
tBuf [ i ] < tMinusDelay ) { if ( i == newIdx ) break ; i = ( i < ( bufSz - 1
) ) ? ( i + 1 ) : 0 ; } } else { while ( tBuf [ i ] >= tMinusDelay ) { i = (
i > 0 ) ? i - 1 : ( bufSz - 1 ) ; } i = ( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0
; } * lastIdx = i ; if ( discrete ) { double tempEps = ( DBL_EPSILON ) *
128.0 ; double localEps = tempEps * muDoubleScalarAbs ( tBuf [ i ] ) ; if (
tempEps > localEps ) { localEps = tempEps ; } localEps = localEps / 2.0 ; if
( tMinusDelay >= ( tBuf [ i ] - localEps ) ) { yout = uBuf [ i ] ; } else {
if ( i == 0 ) { yout = uBuf [ bufSz - 1 ] ; } else { yout = uBuf [ i - 1 ] ;
} } } else { if ( i == 0 ) { t1 = tBuf [ bufSz - 1 ] ; u1 = uBuf [ bufSz - 1
] ; } else { t1 = tBuf [ i - 1 ] ; u1 = uBuf [ i - 1 ] ; } t2 = tBuf [ i ] ;
u2 = uBuf [ i ] ; if ( t2 == t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; }
else { yout = u1 ; } } else { real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 )
; real_T f2 = 1.0 - f1 ; yout = f1 * u1 + f2 * u2 ; } } return ( yout ) ; }
void rt_ssGetBlockPath ( SimStruct * S , int_T sysIdx , int_T blkIdx , char_T
* * path ) { _ssGetBlockPath ( S , sysIdx , blkIdx , path ) ; } void
rt_ssSet_slErrMsg ( SimStruct * S , void * diag ) { _ssSet_slErrMsg ( S ,
diag ) ; } void rt_ssReportDiagnosticAsWarning ( SimStruct * S , void * diag
) { _ssReportDiagnosticAsWarning ( S , diag ) ; } static void mdlOutputs (
SimStruct * S , int_T tid ) { real_T B_31_277_0 ; real_T B_31_278_0 ; real_T
B_31_328_0 ; real_T B_31_336_0 ; real_T B_31_346_0 ; real_T B_31_354_0 ;
real_T B_31_374_0 ; real_T B_31_382_0 ; real_T B_31_392_0 ; real_T B_31_400_0
; real_T B_31_419_0 ; real_T B_31_427_0 ; real_T B_31_437_0 ; real_T
B_31_445_0 ; real_T B_31_464_0 ; real_T B_31_472_0 ; real_T B_31_482_0 ;
real_T B_31_490_0 ; real_T B_14_12_0 ; real_T B_12_83_0 ; real_T B_7_34_0 ;
boolean_T didZcEventOccur ; real_T rtb_B_31_3_0 ; real_T rtb_B_31_4_0 ;
real_T rtb_B_31_5_0 ; boolean_T rtb_B_31_13_0 ; real_T rtb_B_31_17_0 ; real_T
rtb_B_31_20_0 ; real_T rtb_B_31_21_0 ; real_T rtb_B_31_24_0 ; real_T
rtb_B_31_38_0 ; real_T rtb_B_31_42_0 ; boolean_T rtb_B_31_46_0 ; real_T
rtb_RealImagtoComplex_re ; real_T rtb_RealImagtoComplex_im ; int32_T isHit ;
B_new_wall_model_T * _rtB ; P_new_wall_model_T * _rtP ; X_new_wall_model_T *
_rtX ; PrevZCX_new_wall_model_T * _rtZCE ; DW_new_wall_model_T * _rtDW ;
_rtDW = ( ( DW_new_wall_model_T * ) ssGetRootDWork ( S ) ) ; _rtZCE = ( (
PrevZCX_new_wall_model_T * ) _ssGetPrevZCSigState ( S ) ) ; _rtX = ( (
X_new_wall_model_T * ) ssGetContStates ( S ) ) ; _rtP = ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( ( B_new_wall_model_T
* ) _ssGetModelBlockIO ( S ) ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if (
isHit != 0 ) { rtb_B_31_3_0 = _rtDW -> itinit1_PreviousInput ; rtb_B_31_4_0 =
_rtP -> P_168 * _rtDW -> itinit1_PreviousInput ; rtb_B_31_5_0 = 1.000001 *
rtb_B_31_4_0 * 0.96711798839458663 / 0.9999 ; } isHit = ssIsSampleHit ( S , 2
, 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_6_0 = _rtP -> P_169 * _rtDW ->
Currentfilter_states ; _rtB -> B_31_9_0 = ( _rtB -> B_31_6_0 >
new_wall_model_rtC ( S ) -> B_31_7_0 ) ; } isHit = ssIsSampleHit ( S , 1 , 0
) ; if ( isHit != 0 ) { _rtB -> B_31_10_0 = _rtDW -> itinit_PreviousInput ; }
isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { if ( _rtDW ->
inti_IC_LOADING != 0 ) { _rtDW -> inti_DSTATE = _rtB -> B_31_10_0 ; if (
_rtDW -> inti_DSTATE >= _rtP -> P_175 ) { _rtDW -> inti_DSTATE = _rtP ->
P_175 ; } else { if ( _rtDW -> inti_DSTATE <= _rtP -> P_176 ) { _rtDW ->
inti_DSTATE = _rtP -> P_176 ; } } } if ( ( _rtB -> B_31_9_0 > 0.0 ) && (
_rtDW -> inti_PrevResetState <= 0 ) ) { _rtDW -> inti_DSTATE = _rtB ->
B_31_10_0 ; if ( _rtDW -> inti_DSTATE >= _rtP -> P_175 ) { _rtDW ->
inti_DSTATE = _rtP -> P_175 ; } else { if ( _rtDW -> inti_DSTATE <= _rtP ->
P_176 ) { _rtDW -> inti_DSTATE = _rtP -> P_176 ; } } } if ( _rtDW ->
inti_DSTATE >= _rtP -> P_175 ) { _rtDW -> inti_DSTATE = _rtP -> P_175 ; }
else { if ( _rtDW -> inti_DSTATE <= _rtP -> P_176 ) { _rtDW -> inti_DSTATE =
_rtP -> P_176 ; } } _rtB -> B_31_12_0 = _rtP -> P_177 * _rtDW -> inti_DSTATE
; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { rtb_B_31_13_0 =
( _rtB -> B_31_12_0 > rtb_B_31_4_0 ) ; } isHit = ssIsSampleHit ( S , 2 , 0 )
; if ( isHit != 0 ) { if ( _rtB -> B_31_12_0 < _rtB -> B_31_14_0 ) { _rtB ->
B_31_16_0 = _rtB -> B_31_14_0 ; } else { _rtB -> B_31_16_0 = _rtB ->
B_31_12_0 ; } } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { if
( rtb_B_31_13_0 ) { rtb_B_31_17_0 = rtb_B_31_4_0 ; } else { rtb_B_31_17_0 =
_rtB -> B_31_16_0 ; } if ( rtb_B_31_5_0 <= rtb_B_31_17_0 ) { rtb_B_31_17_0 =
rtb_B_31_4_0 ; } rtb_B_31_20_0 = - 0.037459074024101945 * rtb_B_31_3_0 / (
rtb_B_31_3_0 - rtb_B_31_17_0 ) * rtb_B_31_17_0 ; rtb_B_31_21_0 = - _rtB ->
B_31_9_0 * 0.037459074024101945 * _rtB -> B_31_6_0 * rtb_B_31_3_0 / (
rtb_B_31_3_0 - rtb_B_31_17_0 ) ; rtb_B_31_24_0 = _rtP -> P_181 * rtb_B_31_3_0
; if ( ! ( _rtB -> B_31_12_0 > rtb_B_31_24_0 ) ) { rtb_B_31_4_0 = -
rtb_B_31_24_0 * 0.999 * 0.1 * 0.9999 ; if ( _rtB -> B_31_12_0 < rtb_B_31_4_0
) { rtb_B_31_24_0 = rtb_B_31_4_0 ; } else { rtb_B_31_24_0 = _rtB -> B_31_12_0
; } } } isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_30_0 = ( _rtB -> B_31_6_0 < new_wall_model_rtC ( S ) -> B_31_28_0 ) ; }
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { switch ( ( int32_T
) _rtB -> B_31_22_0 ) { case 1 : rtb_B_31_38_0 = - ( _rtB -> B_31_23_0 * _rtB
-> B_31_30_0 ) * 0.037459074024101945 * ( _rtB -> B_31_23_0 * _rtB ->
B_31_6_0 ) * ( 41.359999999999893 / ( _rtB -> B_31_23_0 * rtb_B_31_24_0 +
4.1359999999999895 ) ) ; break ; case 2 : rtb_B_31_38_0 = _rtB -> B_31_31_0 *
rtb_B_31_3_0 ; rtb_B_31_38_0 = - ( _rtB -> B_31_31_0 * _rtB -> B_31_30_0 ) *
0.037459074024101945 * ( _rtB -> B_31_31_0 * _rtB -> B_31_6_0 ) *
rtb_B_31_38_0 / ( _rtB -> B_31_31_0 * rtb_B_31_24_0 + rtb_B_31_38_0 * 0.1 ) ;
break ; case 3 : rtb_B_31_38_0 = - ( _rtB -> B_31_32_0 * _rtB -> B_31_30_0 )
* 0.037459074024101945 * ( _rtB -> B_31_32_0 * _rtB -> B_31_6_0 ) * (
41.359999999999893 / ( muDoubleScalarAbs ( _rtB -> B_31_32_0 * rtb_B_31_24_0
) + 4.1359999999999895 ) ) ; break ; default : rtb_B_31_38_0 = - ( _rtB ->
B_31_33_0 * _rtB -> B_31_30_0 ) * 0.037459074024101945 * ( _rtB -> B_31_33_0
* _rtB -> B_31_6_0 ) * ( 41.359999999999893 / ( muDoubleScalarAbs ( _rtB ->
B_31_33_0 * rtb_B_31_24_0 ) + 4.1359999999999895 ) ) ; break ; } } isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_40_0 = _rtDW
-> DiscreteTimeIntegrator_DSTATE ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if
( isHit != 0 ) { switch ( ( int32_T ) _rtB -> B_31_39_0 ) { case 1 :
rtb_B_31_42_0 = _rtB -> B_31_40_0 ; break ; case 2 : if ( rtb_B_31_17_0 >
_rtP -> P_0 ) { rtb_B_31_24_0 = _rtP -> P_0 ; } else if ( rtb_B_31_17_0 <
_rtP -> P_1 ) { rtb_B_31_24_0 = _rtP -> P_1 ; } else { rtb_B_31_24_0 =
rtb_B_31_17_0 ; } rtb_B_31_42_0 = muDoubleScalarExp ( - 1.5265486725663715 *
rtb_B_31_24_0 ) * 16.79519261235216 ; break ; case 3 : rtb_B_31_42_0 = _rtB
-> B_31_40_0 ; break ; default : rtb_B_31_42_0 = _rtB -> B_31_40_0 ; break ;
} rtb_B_31_42_0 = ( ( ( ( rtb_B_31_20_0 + rtb_B_31_21_0 ) + rtb_B_31_38_0 ) +
rtb_B_31_42_0 ) + - 0.0 * rtb_B_31_17_0 ) + _rtB -> B_31_2_0 ; rtb_B_31_46_0
= ( rtb_B_31_42_0 > _rtB -> B_31_1_0 ) ; } isHit = ssIsSampleHit ( S , 2 , 0
) ; if ( isHit != 0 ) { _rtB -> B_31_47_0 = _rtDW -> Memory2_PreviousInput ;
} isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { if (
rtb_B_31_46_0 ) { _rtB -> B_31_49_0 = _rtB -> B_31_1_0 ; } else if (
rtb_B_31_42_0 < _rtB -> B_31_47_0 ) { _rtB -> B_31_49_0 = _rtB -> B_31_47_0 ;
} else { _rtB -> B_31_49_0 = rtb_B_31_42_0 ; } } _rtB -> B_31_50_0 = 0.0 ;
_rtB -> B_31_50_0 += _rtP -> P_192 * _rtX -> StateSpace_CSTATE ;
rtb_B_31_38_0 = rt_Lookup ( _rtP -> P_194 , 11 , _rtB -> B_31_50_0 , _rtP ->
P_195 ) ; if ( rtb_B_31_38_0 > _rtP -> P_196 ) { _rtB -> B_31_52_0 = _rtP ->
P_196 ; } else if ( rtb_B_31_38_0 < _rtP -> P_197 ) { _rtB -> B_31_52_0 =
_rtP -> P_197 ; } else { _rtB -> B_31_52_0 = rtb_B_31_38_0 ; } isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { if ( _rtDW -> systemEnable
!= 0 ) { _rtDW -> lastSin = muDoubleScalarSin ( _rtP -> P_200 * ssGetTaskTime
( S , 2 ) ) ; _rtDW -> lastCos = muDoubleScalarCos ( _rtP -> P_200 *
ssGetTaskTime ( S , 2 ) ) ; _rtDW -> systemEnable = 0 ; } _rtB -> B_31_53_0 =
( ( _rtDW -> lastSin * _rtP -> P_204 + _rtDW -> lastCos * _rtP -> P_203 ) *
_rtP -> P_202 + ( _rtDW -> lastCos * _rtP -> P_204 - _rtDW -> lastSin * _rtP
-> P_203 ) * _rtP -> P_201 ) * _rtP -> P_198 + _rtP -> P_199 ; if ( _rtDW ->
systemEnable_g != 0 ) { _rtDW -> lastSin_i = muDoubleScalarSin ( _rtP ->
P_207 * ssGetTaskTime ( S , 2 ) ) ; _rtDW -> lastCos_o = muDoubleScalarCos (
_rtP -> P_207 * ssGetTaskTime ( S , 2 ) ) ; _rtDW -> systemEnable_g = 0 ; }
_rtB -> B_31_54_0 = ( ( _rtDW -> lastSin_i * _rtP -> P_211 + _rtDW ->
lastCos_o * _rtP -> P_210 ) * _rtP -> P_209 + ( _rtDW -> lastCos_o * _rtP ->
P_211 - _rtDW -> lastSin_i * _rtP -> P_210 ) * _rtP -> P_208 ) * _rtP ->
P_205 + _rtP -> P_206 ; if ( _rtDW -> systemEnable_d != 0 ) { _rtDW ->
lastSin_g = muDoubleScalarSin ( _rtP -> P_214 * ssGetTaskTime ( S , 2 ) ) ;
_rtDW -> lastCos_j = muDoubleScalarCos ( _rtP -> P_214 * ssGetTaskTime ( S ,
2 ) ) ; _rtDW -> systemEnable_d = 0 ; } _rtB -> B_31_55_0 = ( ( _rtDW ->
lastSin_g * _rtP -> P_218 + _rtDW -> lastCos_j * _rtP -> P_217 ) * _rtP ->
P_216 + ( _rtDW -> lastCos_j * _rtP -> P_218 - _rtDW -> lastSin_g * _rtP ->
P_217 ) * _rtP -> P_215 ) * _rtP -> P_212 + _rtP -> P_213 ;
ssCallAccelRunBlock ( S , 31 , 59 , SS_CALL_MDL_OUTPUTS ) ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { rtb_B_31_3_0 *= _rtP ->
P_232 ; rtb_B_31_38_0 = ( 1.0 - rtb_B_31_17_0 / rtb_B_31_3_0 ) * 100.0 ; if (
rtb_B_31_38_0 > _rtP -> P_233 ) { _rtB -> B_31_62_0 = _rtP -> P_233 ; } else
if ( rtb_B_31_38_0 < _rtP -> P_234 ) { _rtB -> B_31_62_0 = _rtP -> P_234 ; }
else { _rtB -> B_31_62_0 = rtb_B_31_38_0 ; } } isHit = ssIsSampleHit ( S , 2
, 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_63_0 = _rtP -> P_235 * _rtB ->
B_31_59_0 [ 27 ] ; _rtB -> B_31_64_0 = _rtP -> P_236 * _rtB -> B_31_63_0 ; }
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_65_0 =
_rtB -> B_31_49_0 - _rtB -> B_31_64_0 ; _rtB -> B_31_66_0 = _rtB -> B_31_63_0
* _rtB -> B_31_65_0 ; ssCallAccelRunBlock ( S , 31 , 67 , SS_CALL_MDL_OUTPUTS
) ; } isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_82_0 = ( ( real_T ) ( _rtB -> B_31_63_0 < new_wall_model_rtC ( S ) ->
B_31_76_0 ) * _rtP -> P_239 - _rtB -> B_31_40_0 ) * muDoubleScalarAbs ( _rtB
-> B_31_63_0 ) * _rtP -> P_240 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if (
isHit != 0 ) { _rtB -> B_31_83_0 = _rtP -> P_241 * rtb_B_31_17_0 ; } isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_84_0 = _rtP ->
P_242 * _rtB -> B_31_63_0 ; } _rtB -> B_31_102_0 = 0.0 ; _rtB -> B_31_102_0
+= _rtP -> P_244 * _rtX -> TransferFcn8_CSTATE ; isHit = ssIsSampleHit ( S ,
2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_104_0 = _rtP -> P_246 * _rtB ->
B_31_59_0 [ 24 ] ; } _rtB -> B_31_105_0 = 0.0 ; _rtB -> B_31_105_0 += _rtP ->
P_248 * _rtX -> TransferFcn1_CSTATE ; ssCallAccelRunBlock ( S , 31 , 106 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( ( isHit !=
0 ) && ( ssIsMajorTimeStep ( S ) != 0 ) ) { if ( _rtB -> B_31_106_0 > 0.0 ) {
if ( ! _rtDW -> InverterController_MODE ) { if ( ssGetTaskTime ( S , 1 ) !=
ssGetTStart ( S ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } (
void ) memset ( & ( ( ( XDis_new_wall_model_T * ) ssGetContStateDisabled ( S
) ) -> TransferFcn2_CSTATE_g ) , 0 , 18 * sizeof ( boolean_T ) ) ; _rtX ->
TransferFcn2_CSTATE_g = 0.0 ; _rtX -> Integrator_CSTATE_i = _rtP -> P_59 ;
_rtX -> Filter_CSTATE_k = _rtP -> P_61 ; _rtX -> TransferFcn3_CSTATE_n = 0.0
; _rtX -> TransferFcn1_CSTATE_d = 0.0 ; _rtX -> Filter_CSTATE_i = _rtP ->
P_73 ; _rtX -> Integrator_CSTATE_m = _rtP -> P_76 ; if ( ssIsFirstInitCond (
S ) != 0 ) { _rtX -> Integrator_CSTATE_l = 0.0 ; } _rtDW -> Integrator_IWORK
= 1 ; _rtDW -> Memory1_PreviousInput = _rtP -> P_82 ; _rtX ->
Integrator_CSTATE_f5 = _rtP -> P_84 ; ( ( X_new_wall_model_T * )
ssGetContStates ( S ) ) -> VariableTransportDelay_CSTATE = 0.0 ; _rtX ->
integrator_CSTATE_e2 = _rtP -> P_89 ; _rtDW -> Memory_PreviousInput_c = _rtP
-> P_91 ; _rtX -> TransferFcn_CSTATE_j = 0.0 ; _rtDW -> LastMajorTimeA = (
rtInf ) ; _rtDW -> LastMajorTimeB = ( rtInf ) ; _rtX -> Integrator_x1_CSTATE
= _rtP -> P_105 ; _rtX -> Integrator_x2_CSTATE = _rtP -> P_107 ; _rtDW ->
InverterController_MODE = true ; } } else { if ( _rtDW ->
InverterController_MODE ) { ssSetBlockStateForSolverChangedAtMajorStep ( S )
; ( void ) memset ( & ( ( ( XDis_new_wall_model_T * ) ssGetContStateDisabled
( S ) ) -> TransferFcn2_CSTATE_g ) , 1 , 18 * sizeof ( boolean_T ) ) ; if (
_rtDW -> AutomaticGainControl_MODE ) { _rtDW -> AutomaticGainControl_MODE =
false ; } _rtB -> B_31_111_0 [ 0 ] = _rtP -> P_55 [ 0 ] ; _rtB -> B_31_111_0
[ 1 ] = _rtP -> P_55 [ 1 ] ; _rtB -> B_31_111_0 [ 2 ] = _rtP -> P_55 [ 2 ] ;
_rtB -> B_31_111_0 [ 3 ] = _rtP -> P_55 [ 3 ] ; _rtDW ->
InverterController_MODE = false ; } } } if ( _rtDW -> InverterController_MODE
) { _rtB -> B_12_0_0 = 0.0 ; _rtB -> B_12_0_0 += _rtP -> P_57 * _rtX ->
TransferFcn2_CSTATE_g ; _rtB -> B_12_1_0 = _rtP -> P_58 * _rtB -> B_12_0_0 ;
_rtB -> B_12_2_0 = _rtX -> Integrator_CSTATE_i ; _rtB -> B_12_3_0 = _rtP ->
P_60 * _rtB -> B_12_0_0 ; _rtB -> B_12_4_0 = _rtX -> Filter_CSTATE_k ; _rtB
-> B_12_5_0 = _rtB -> B_12_3_0 - _rtB -> B_12_4_0 ; _rtB -> B_12_6_0 = _rtP
-> P_62 * _rtB -> B_12_5_0 ; _rtB -> B_12_7_0 = ( _rtB -> B_12_1_0 + _rtB ->
B_12_2_0 ) + _rtB -> B_12_6_0 ; if ( _rtB -> B_12_7_0 > _rtP -> P_63 ) { _rtB
-> B_12_8_0 = _rtP -> P_63 ; } else if ( _rtB -> B_12_7_0 < _rtP -> P_64 ) {
_rtB -> B_12_8_0 = _rtP -> P_64 ; } else { _rtB -> B_12_8_0 = _rtB ->
B_12_7_0 ; } _rtB -> B_12_9_0 = _rtP -> P_65 * _rtB -> B_12_8_0 ; _rtB ->
B_12_11_0 = _rtB -> B_12_9_0 + _rtB -> B_12_10_0 ; _rtB -> B_12_12_0 = _rtB
-> B_31_102_0 - _rtB -> B_31_103_0 ; _rtB -> B_12_13_0 = 0.0 ; _rtB ->
B_12_13_0 += _rtP -> P_68 * _rtX -> TransferFcn3_CSTATE_n ; _rtB -> B_12_14_0
= _rtB -> B_31_105_0 - _rtB -> B_12_13_0 ; isHit = ssIsSampleHit ( S , 2 , 0
) ; if ( isHit != 0 ) { _rtB -> B_12_16_0 = _rtB -> B_31_104_0 / _rtB ->
B_12_15_0 ; } _rtB -> B_12_17_0 = 0.0 ; _rtB -> B_12_17_0 += _rtP -> P_71 *
_rtX -> TransferFcn1_CSTATE_d ; _rtB -> B_12_18_0 = _rtP -> P_72 * _rtB ->
B_12_17_0 ; _rtB -> B_12_19_0 = _rtX -> Filter_CSTATE_i ; _rtB -> B_12_20_0 =
_rtB -> B_12_18_0 - _rtB -> B_12_19_0 ; _rtB -> B_12_21_0 = _rtP -> P_74 *
_rtB -> B_12_20_0 ; _rtB -> B_12_22_0 = _rtP -> P_75 * _rtB -> B_12_17_0 ;
_rtB -> B_12_23_0 = _rtX -> Integrator_CSTATE_m ; _rtB -> B_12_24_0 = _rtP ->
P_77 * _rtB -> B_12_17_0 ; _rtB -> B_12_25_0 = ( _rtB -> B_12_24_0 + _rtB ->
B_12_23_0 ) + _rtB -> B_12_21_0 ; _rtB -> B_12_26_0 = _rtP -> P_78 * _rtB ->
B_12_0_0 ; _rtB -> B_12_28_0 = ( _rtX -> Integrator_CSTATE_l > _rtB ->
B_12_27_0 ) ; _rtB -> B_12_29_0 = _rtX -> Integrator_CSTATE_l - _rtB ->
B_12_27_0 ; if ( ( _rtDW -> Initial_FirstOutputTime == ( rtMinusInf ) ) || (
_rtDW -> Initial_FirstOutputTime == ssGetTaskTime ( S , 0 ) ) ) { _rtDW ->
Initial_FirstOutputTime = ssGetTaskTime ( S , 0 ) ; _rtB -> B_12_30_0 = _rtP
-> P_80 ; } else { _rtB -> B_12_30_0 = _rtB -> B_12_29_0 ; } if (
ssIsMajorTimeStep ( S ) != 0 ) { rtb_B_31_13_0 = false ; rtb_B_31_46_0 =
false ; didZcEventOccur = ( _rtB -> B_12_28_0 && ( _rtZCE ->
Integrator_Reset_ZCE != POS_ZCSIG ) ) ; _rtZCE -> Integrator_Reset_ZCE = _rtB
-> B_12_28_0 ; if ( didZcEventOccur || ( _rtDW -> Integrator_IWORK != 0 ) ) {
rtb_B_31_13_0 = true ; _rtX -> Integrator_CSTATE_l = _rtB -> B_12_30_0 ;
rtb_B_31_46_0 = true ; } if ( rtb_B_31_13_0 ) {
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; if ( rtb_B_31_46_0 ) {
ssSetContTimeOutputInconsistentWithStateAtMajorStep ( S ) ; } } } _rtB ->
B_12_31_0 = _rtX -> Integrator_CSTATE_l ; isHit = ssIsSampleHit ( S , 1 , 0 )
; if ( isHit != 0 ) { _rtB -> B_12_33_0 = _rtDW -> Memory1_PreviousInput ; if
( ssIsMajorTimeStep ( S ) != 0 ) { if ( _rtB -> B_12_34_0 > 0.0 ) { if ( !
_rtDW -> AutomaticGainControl_MODE ) { if ( ssGetTaskTime ( S , 1 ) !=
ssGetTStart ( S ) ) { ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } (
void ) memset ( & ( ( ( XDis_new_wall_model_T * ) ssGetContStateDisabled ( S
) ) -> VariableTransportDelay_CSTATE_m ) , 0 , 4 * sizeof ( boolean_T ) ) ;
_rtDW -> AutomaticGainControl_MODE = true ; } } else { if ( _rtDW ->
AutomaticGainControl_MODE ) { ssSetBlockStateForSolverChangedAtMajorStep ( S
) ; ( void ) memset ( & ( ( ( XDis_new_wall_model_T * )
ssGetContStateDisabled ( S ) ) -> VariableTransportDelay_CSTATE_m ) , 1 , 4 *
sizeof ( boolean_T ) ) ; _rtDW -> AutomaticGainControl_MODE = false ; } } } }
if ( _rtDW -> AutomaticGainControl_MODE ) { { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> VariableTransportDelay_PWORK_b . TUbufferPtrs [ 0 ] ; real_T *
* tBuffer = ( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK_b .
TUbufferPtrs [ 1 ] ; real_T * * xBuffer = ( real_T * * ) & _rtDW ->
VariableTransportDelay_PWORK_b . TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT
( S ) ; real_T appliedDelay ; _rtB -> B_10_0_0 =
new_wall_model_acc_rt_VTDelayfindtDInterpolate ( ( ( X_new_wall_model_T * )
ssGetContStates ( S ) ) -> VariableTransportDelay_CSTATE_m , * tBuffer , *
uBuffer , * xBuffer , _rtDW -> VariableTransportDelay_IWORK_o .
CircularBufSize , _rtDW -> VariableTransportDelay_IWORK_o . Head , _rtDW ->
VariableTransportDelay_IWORK_o . Tail , & _rtDW ->
VariableTransportDelay_IWORK_o . Last , simTime , 0.0 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
, _rtP -> P_38 , & appliedDelay ) ; } _rtB -> B_10_1_0 = _rtX ->
integrator_CSTATE_n ; rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_10_3_0 ) ;
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_10_5_0 =
_rtDW -> Memory_PreviousInput_cc ; } if ( rtb_B_31_13_0 ) { _rtB -> B_8_0_0 =
_rtB -> B_10_1_0 - _rtB -> B_10_0_0 ; _rtB -> B_8_1_0 = _rtB -> B_8_0_0 *
_rtB -> B_12_33_0 ; _rtB -> B_10_7_0 = _rtB -> B_8_1_0 ; } else { _rtB ->
B_10_7_0 = _rtB -> B_10_5_0 ; } { real_T * * uBuffer = ( real_T * * ) & _rtDW
-> VariableTransportDelay_PWORK_g . TUbufferPtrs [ 0 ] ; real_T * * tBuffer =
( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK_g . TUbufferPtrs [ 1 ]
; real_T * * xBuffer = ( real_T * * ) & _rtDW ->
VariableTransportDelay_PWORK_g . TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT
( S ) ; real_T appliedDelay ; _rtB -> B_10_8_0 =
new_wall_model_acc_rt_VTDelayfindtDInterpolate ( ( ( X_new_wall_model_T * )
ssGetContStates ( S ) ) -> VariableTransportDelay_CSTATE_c , * tBuffer , *
uBuffer , * xBuffer , _rtDW -> VariableTransportDelay_IWORK_p .
CircularBufSize , _rtDW -> VariableTransportDelay_IWORK_p . Head , _rtDW ->
VariableTransportDelay_IWORK_p . Tail , & _rtDW ->
VariableTransportDelay_IWORK_p . Last , simTime , 0.0 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
, _rtP -> P_43 , & appliedDelay ) ; } _rtB -> B_10_9_0 = _rtX ->
integrator_CSTATE_b4 ; rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_10_11_0 )
; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_10_13_0
= _rtDW -> Memory_PreviousInput_cz ; } if ( rtb_B_31_13_0 ) { _rtB -> B_9_0_0
= _rtB -> B_10_9_0 - _rtB -> B_10_8_0 ; _rtB -> B_9_1_0 = _rtB -> B_9_0_0 *
_rtB -> B_12_33_0 ; _rtB -> B_10_15_0 = _rtB -> B_9_1_0 ; } else { _rtB ->
B_10_15_0 = _rtB -> B_10_13_0 ; } _rtB -> B_10_16_0 . re = _rtB -> B_10_7_0 ;
_rtB -> B_10_16_0 . im = _rtB -> B_10_15_0 ; rtb_B_31_3_0 =
muDoubleScalarHypot ( _rtB -> B_10_16_0 . re , _rtB -> B_10_16_0 . im ) ;
muDoubleScalarSinCos ( _rtB -> B_12_31_0 , & rtb_B_31_17_0 , & rtb_B_31_38_0
) ; _rtB -> B_10_19_0 = _rtP -> P_47 * rtb_B_31_17_0 ; _rtB -> B_10_20_0 =
_rtP -> P_48 * rtb_B_31_38_0 ; if ( _rtB -> B_12_33_0 > _rtP -> P_49 ) { _rtB
-> B_10_21_0 = _rtP -> P_49 ; } else if ( _rtB -> B_12_33_0 < _rtP -> P_50 )
{ _rtB -> B_10_21_0 = _rtP -> P_50 ; } else { _rtB -> B_10_21_0 = _rtB ->
B_12_33_0 ; } _rtB -> B_10_22_0 = 1.0 / _rtB -> B_10_21_0 ; if ( _rtB ->
B_12_33_0 > _rtP -> P_51 ) { _rtB -> B_10_23_0 = _rtP -> P_51 ; } else if (
_rtB -> B_12_33_0 < _rtP -> P_52 ) { _rtB -> B_10_23_0 = _rtP -> P_52 ; }
else { _rtB -> B_10_23_0 = _rtB -> B_12_33_0 ; } _rtB -> B_10_24_0 = 1.0 /
_rtB -> B_10_23_0 ; _rtB -> B_10_25_0 = _rtB -> B_12_16_0 * _rtB -> B_10_19_0
; _rtB -> B_10_26_0 = _rtB -> B_12_16_0 * _rtB -> B_10_20_0 ; if (
rtb_B_31_3_0 > _rtP -> P_53 ) { _rtB -> B_10_27_0 = _rtP -> P_53 ; } else if
( rtb_B_31_3_0 < _rtP -> P_54 ) { _rtB -> B_10_27_0 = _rtP -> P_54 ; } else {
_rtB -> B_10_27_0 = rtb_B_31_3_0 ; } _rtB -> B_10_28_0 = 1.0 / _rtB ->
B_10_27_0 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
AutomaticGainControl_SubsysRanBC ) ; } } if ( _rtX -> Integrator_CSTATE_f5 >=
_rtP -> P_85 ) { if ( _rtX -> Integrator_CSTATE_f5 != _rtP -> P_85 ) { _rtX
-> Integrator_CSTATE_f5 = _rtP -> P_85 ;
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } } else { if ( ( _rtX ->
Integrator_CSTATE_f5 <= _rtP -> P_86 ) && ( _rtX -> Integrator_CSTATE_f5 !=
_rtP -> P_86 ) ) { _rtX -> Integrator_CSTATE_f5 = _rtP -> P_86 ;
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } } _rtB -> B_12_36_0 =
_rtX -> Integrator_CSTATE_f5 ; { real_T * * uBuffer = ( real_T * * ) & _rtDW
-> VariableTransportDelay_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = (
real_T * * ) & _rtDW -> VariableTransportDelay_PWORK . TUbufferPtrs [ 1 ] ;
real_T * * xBuffer = ( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK .
TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT ( S ) ; real_T appliedDelay ;
_rtB -> B_12_37_0 = new_wall_model_acc_rt_VTDelayfindtDInterpolate ( ( (
X_new_wall_model_T * ) ssGetContStates ( S ) ) ->
VariableTransportDelay_CSTATE , * tBuffer , * uBuffer , * xBuffer , _rtDW ->
VariableTransportDelay_IWORK . CircularBufSize , _rtDW ->
VariableTransportDelay_IWORK . Head , _rtDW -> VariableTransportDelay_IWORK .
Tail , & _rtDW -> VariableTransportDelay_IWORK . Last , simTime , 0.0 , 0 , (
boolean_T ) ( ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) ==
ssGetT ( S ) ) ) , _rtP -> P_88 , & appliedDelay ) ; } _rtB -> B_12_38_0 =
_rtX -> integrator_CSTATE_e2 ; rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_12_40_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_12_42_0 = _rtDW -> Memory_PreviousInput_c ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_11_0_0 = _rtB -> B_12_38_0 - _rtB -> B_12_37_0 ; _rtB -> B_11_1_0 =
_rtB -> B_11_0_0 * _rtB -> B_12_33_0 ; _rtB -> B_12_44_0 = _rtB -> B_11_1_0 ;
} else { _rtB -> B_12_44_0 = _rtB -> B_12_42_0 ; } _rtB -> B_12_45_0 = _rtB
-> B_12_44_0 * _rtB -> B_10_28_0 ; _rtB -> B_12_46_0 = _rtP -> P_92 * _rtB ->
B_12_45_0 ; _rtB -> B_12_47_0 = _rtP -> P_93 * _rtB -> B_12_45_0 ; _rtB ->
B_12_48_0 = _rtP -> P_94 * _rtB -> B_12_45_0 ; _rtB -> B_12_49_0 = 0.0 ; _rtB
-> B_12_49_0 += _rtP -> P_96 * _rtX -> TransferFcn_CSTATE_j ; _rtB ->
B_12_49_0 += _rtP -> P_97 * _rtB -> B_12_48_0 ; _rtB -> B_12_50_0 = ( _rtB ->
B_12_46_0 + _rtB -> B_12_36_0 ) + _rtB -> B_12_49_0 ; if ( _rtB -> B_12_50_0
> _rtP -> P_98 ) { _rtB -> B_12_51_0 = _rtP -> P_98 ; } else if ( _rtB ->
B_12_50_0 < _rtP -> P_99 ) { _rtB -> B_12_51_0 = _rtP -> P_99 ; } else { _rtB
-> B_12_51_0 = _rtB -> B_12_50_0 ; } _rtB -> B_12_52_0 = _rtP -> P_100 * _rtB
-> B_12_51_0 ; if ( _rtB -> B_12_33_0 > _rtP -> P_101 ) { _rtB -> B_12_53_0 =
_rtP -> P_101 ; } else if ( _rtB -> B_12_33_0 < _rtP -> P_102 ) { _rtB ->
B_12_53_0 = _rtP -> P_102 ; } else { _rtB -> B_12_53_0 = _rtB -> B_12_33_0 ;
} _rtB -> B_12_54_0 = 1.0 / _rtB -> B_12_53_0 ; _rtB -> B_12_56_0 = _rtB ->
B_12_16_0 * muDoubleScalarCos ( _rtB -> B_12_31_0 ) ; if ( ( _rtDW ->
LastMajorTimeA >= ssGetTaskTime ( S , 0 ) ) && ( _rtDW -> LastMajorTimeB >=
ssGetTaskTime ( S , 0 ) ) ) { _rtB -> B_12_57_0 = _rtB -> B_12_52_0 ; } else
{ if ( ( ( _rtDW -> LastMajorTimeA < _rtDW -> LastMajorTimeB ) && ( _rtDW ->
LastMajorTimeB < ssGetTaskTime ( S , 0 ) ) ) || ( ( _rtDW -> LastMajorTimeA
>= _rtDW -> LastMajorTimeB ) && ( _rtDW -> LastMajorTimeA >= ssGetTaskTime (
S , 0 ) ) ) ) { rtb_B_31_38_0 = ssGetTaskTime ( S , 0 ) - _rtDW ->
LastMajorTimeB ; rtb_B_31_3_0 = _rtDW -> PrevYB ; } else { rtb_B_31_38_0 =
ssGetTaskTime ( S , 0 ) - _rtDW -> LastMajorTimeA ; rtb_B_31_3_0 = _rtDW ->
PrevYA ; } rtb_B_31_20_0 = rtb_B_31_38_0 * _rtP -> P_103 ; rtb_B_31_17_0 =
_rtB -> B_12_52_0 - rtb_B_31_3_0 ; if ( rtb_B_31_17_0 > rtb_B_31_20_0 ) {
_rtB -> B_12_57_0 = rtb_B_31_3_0 + rtb_B_31_20_0 ; } else { rtb_B_31_38_0 *=
_rtP -> P_104 ; if ( rtb_B_31_17_0 < rtb_B_31_38_0 ) { _rtB -> B_12_57_0 =
rtb_B_31_3_0 + rtb_B_31_38_0 ; } else { _rtB -> B_12_57_0 = _rtB -> B_12_52_0
; } } } _rtB -> B_12_58_0 = _rtX -> Integrator_x1_CSTATE ; _rtB -> B_12_59_0
= _rtP -> P_106 * _rtB -> B_12_58_0 ; _rtB -> B_12_60_0 = _rtX ->
Integrator_x2_CSTATE ; _rtB -> B_12_61_0 = _rtP -> P_108 * _rtB -> B_12_60_0
; _rtB -> B_12_62_0 = _rtP -> P_109 * _rtB -> B_12_58_0 ; _rtB -> B_12_63_0 =
_rtP -> P_110 * _rtB -> B_12_60_0 ; _rtB -> B_12_64_0 = _rtB -> B_12_59_0 +
_rtB -> B_12_61_0 ; _rtB -> B_12_65_0 = _rtB -> B_12_62_0 + _rtB -> B_12_63_0
; _rtB -> B_12_66_0 = _rtP -> P_111 * _rtB -> B_12_57_0 ; _rtB -> B_12_67_0 =
_rtB -> B_12_64_0 + _rtB -> B_12_66_0 ; _rtB -> B_12_68_0 = _rtP -> P_112 *
_rtB -> B_12_57_0 ; _rtB -> B_12_69_0 = _rtB -> B_12_65_0 + _rtB -> B_12_68_0
; _rtB -> B_12_74_0 = ( _rtP -> P_113 * _rtB -> B_12_58_0 + _rtP -> P_114 *
_rtB -> B_12_60_0 ) + _rtP -> P_115 * _rtB -> B_12_57_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_12_78_0 = _rtP ->
P_117 ; } _rtB -> B_12_79_0 = ssGetT ( S ) + _rtB -> B_12_78_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_12_80_0 = _rtP ->
P_118 ; } _rtB -> B_12_82_0 = _rtP -> P_119 * muDoubleScalarRem ( _rtB ->
B_12_79_0 , _rtB -> B_12_80_0 ) ; new_wall_model_acc_LookUp_real_T_real_T ( &
( B_12_83_0 ) , _rtP -> P_121 , _rtB -> B_12_82_0 , _rtP -> P_120 , 2U ) ;
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_12_84_0 =
_rtP -> P_122 ; } _rtB -> B_12_85_0 = B_12_83_0 - _rtB -> B_12_84_0 ; _rtB ->
B_12_87_0 = _rtB -> B_12_85_0 * _rtB -> B_12_86_0 ; _rtB -> B_12_88_0 = (
_rtB -> B_12_75_0 [ 0 ] + _rtB -> B_12_87_0 ) + _rtB -> B_12_86_0 ;
rtb_B_31_13_0 = ( _rtB -> B_12_11_0 >= _rtB -> B_12_88_0 ) ; _rtB ->
B_12_90_0 = _rtP -> P_124 * _rtB -> B_12_11_0 ; rtb_B_31_46_0 = ( _rtB ->
B_12_90_0 >= _rtB -> B_12_88_0 ) ; _rtB -> B_31_111_0 [ 0 ] = rtb_B_31_13_0 ;
_rtB -> B_31_111_0 [ 1 ] = ! rtb_B_31_13_0 ; _rtB -> B_31_111_0 [ 2 ] =
rtb_B_31_46_0 ; _rtB -> B_31_111_0 [ 3 ] = ! rtb_B_31_46_0 ; if ( _rtB ->
B_12_25_0 > _rtP -> P_125 ) { _rtB -> B_12_94_0 = _rtP -> P_125 ; } else if (
_rtB -> B_12_25_0 < _rtP -> P_126 ) { _rtB -> B_12_94_0 = _rtP -> P_126 ; }
else { _rtB -> B_12_94_0 = _rtB -> B_12_25_0 ; } _rtB -> B_12_96_0 = _rtB ->
B_12_94_0 * muDoubleScalarSin ( _rtB -> B_12_31_0 ) ; if ( ssIsMajorTimeStep
( S ) != 0 ) { srUpdateBC ( _rtDW -> InverterController_SubsysRanBC ) ; } }
_rtB -> B_31_108_0 = 0.0 ; _rtB -> B_31_108_0 += _rtP -> P_250 * _rtX ->
TransferFcn3_CSTATE ; _rtB -> B_31_109_0 = ! ( _rtB -> B_31_106_0 != 0.0 ) ;
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( ( isHit != 0 ) && (
ssIsMajorTimeStep ( S ) != 0 ) ) { if ( _rtB -> B_31_109_0 ) { if ( ! _rtDW
-> PWMController2_MODE ) { if ( ssGetTaskTime ( S , 1 ) != ssGetTStart ( S )
) { ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } _rtDW ->
systemEnable_o = 1 ; _rtDW -> PWMController2_MODE = true ; } } else { if (
_rtDW -> PWMController2_MODE ) { ssSetBlockStateForSolverChangedAtMajorStep (
S ) ; _rtB -> B_31_111_0 [ 0 ] = _rtP -> P_129 [ 0 ] ; _rtB -> B_31_111_0 [ 1
] = _rtP -> P_129 [ 1 ] ; _rtB -> B_31_111_0 [ 2 ] = _rtP -> P_129 [ 2 ] ;
_rtB -> B_31_111_0 [ 3 ] = _rtP -> P_129 [ 3 ] ; _rtDW -> PWMController2_MODE
= false ; } } } if ( _rtDW -> PWMController2_MODE ) { _rtB -> B_14_3_0 = _rtB
-> B_14_2_0 / ( _rtB -> B_31_108_0 + _rtB -> B_14_0_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_14_7_0 = _rtP ->
P_133 ; } _rtB -> B_14_8_0 = ssGetT ( S ) + _rtB -> B_14_7_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_14_9_0 = _rtP ->
P_134 ; } _rtB -> B_14_11_0 = _rtP -> P_135 * muDoubleScalarRem ( _rtB ->
B_14_8_0 , _rtB -> B_14_9_0 ) ; new_wall_model_acc_LookUp_real_T_real_T ( & (
B_14_12_0 ) , _rtP -> P_137 , _rtB -> B_14_11_0 , _rtP -> P_136 , 2U ) ;
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_14_13_0 =
_rtP -> P_138 ; } _rtB -> B_14_14_0 = B_14_12_0 - _rtB -> B_14_13_0 ; _rtB ->
B_14_16_0 = _rtB -> B_14_14_0 * _rtB -> B_14_15_0 ; _rtB -> B_14_17_0 = (
_rtB -> B_14_4_0 [ 0 ] + _rtB -> B_14_16_0 ) + _rtB -> B_14_15_0 ; isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { if ( _rtDW ->
systemEnable_o != 0 ) { _rtDW -> lastSin_c = muDoubleScalarSin ( _rtP ->
P_142 * ssGetTaskTime ( S , 2 ) ) ; _rtDW -> lastCos_jx = muDoubleScalarCos (
_rtP -> P_142 * ssGetTaskTime ( S , 2 ) ) ; _rtDW -> systemEnable_o = 0 ; }
_rtB -> B_14_19_0 = ( ( ( _rtDW -> lastSin_c * _rtP -> P_146 + _rtDW ->
lastCos_jx * _rtP -> P_145 ) * _rtP -> P_144 + ( _rtDW -> lastCos_jx * _rtP
-> P_146 - _rtDW -> lastSin_c * _rtP -> P_145 ) * _rtP -> P_143 ) * _rtP ->
P_140 + _rtP -> P_141 ) * _rtB -> B_14_3_0 ; } rtb_B_31_13_0 = ( _rtB ->
B_14_19_0 >= _rtB -> B_14_17_0 ) ; _rtB -> B_14_21_0 = _rtP -> P_147 * _rtB
-> B_14_19_0 ; rtb_B_31_46_0 = ( _rtB -> B_14_21_0 >= _rtB -> B_14_17_0 ) ;
_rtB -> B_31_111_0 [ 0 ] = rtb_B_31_13_0 ; _rtB -> B_31_111_0 [ 1 ] = !
rtb_B_31_13_0 ; _rtB -> B_31_111_0 [ 2 ] = rtb_B_31_46_0 ; _rtB -> B_31_111_0
[ 3 ] = ! rtb_B_31_46_0 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC (
_rtDW -> PWMController2_SubsysRanBC ) ; } } _rtB -> B_31_119_0 = _rtB ->
B_31_109_0 ; _rtB -> B_31_123_0 = _rtB -> B_31_109_0 ; _rtB -> B_31_127_0 =
_rtB -> B_31_109_0 ; _rtB -> B_31_134_0 = _rtB -> B_31_109_0 ; isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_138_0 = _rtP
-> P_251 * _rtB -> B_31_59_0 [ 28 ] ; } _rtB -> B_31_148_0 = 0.0 ; _rtB ->
B_31_148_0 += _rtP -> P_253 * _rtX -> TransferFcn_CSTATE ; isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_149_0 = _rtP
-> P_254 * _rtB -> B_31_59_0 [ 23 ] ; } _rtB -> B_31_178_0 = _rtB ->
B_31_109_0 ; _rtB -> B_31_185_0 = _rtB -> B_31_109_0 ; isHit = ssIsSampleHit
( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_204_0 = _rtP -> P_255 * _rtB
-> B_31_59_0 [ 32 ] ; _rtB -> B_31_208_0 = _rtP -> P_256 * _rtB -> B_31_59_0
[ 29 ] ; _rtB -> B_31_212_0 = _rtP -> P_257 * _rtB -> B_31_59_0 [ 31 ] ; _rtB
-> B_31_216_0 = _rtP -> P_258 * _rtB -> B_31_59_0 [ 30 ] ; } _rtB ->
B_31_236_0 = 0.0 ; _rtB -> B_31_236_0 += _rtP -> P_260 * _rtX ->
TransferFcn_CSTATE_p ; _rtB -> B_31_238_0 = 0.0 ; _rtB -> B_31_238_0 += _rtP
-> P_263 * _rtX -> TransferFcn2_CSTATE ; isHit = ssIsSampleHit ( S , 1 , 0 )
; if ( ( isHit != 0 ) && ( ssIsMajorTimeStep ( S ) != 0 ) ) { if ( _rtB ->
B_31_109_0 ) { if ( ! _rtDW -> BatteryController_MODE ) { if ( ssGetTaskTime
( S , 1 ) != ssGetTStart ( S ) ) { ssSetBlockStateForSolverChangedAtMajorStep
( S ) ; } ( void ) memset ( & ( ( ( XDis_new_wall_model_T * )
ssGetContStateDisabled ( S ) ) -> TransferFcn2_CSTATE_a ) , 0 , 7 * sizeof (
boolean_T ) ) ; _rtX -> TransferFcn2_CSTATE_a = 0.0 ; _rtX ->
Integrator_CSTATE_j = _rtP -> P_7 ; _rtX -> Filter_CSTATE_m = _rtP -> P_9 ;
_rtX -> TransferFcn3_CSTATE_g = 0.0 ; _rtX -> TransferFcn1_CSTATE_h = 0.0 ;
_rtX -> Filter_CSTATE_l = _rtP -> P_22 ; _rtX -> Integrator_CSTATE_d = _rtP
-> P_25 ; _rtDW -> BatteryController_MODE = true ; } } else { if ( _rtDW ->
BatteryController_MODE ) { ssSetBlockStateForSolverChangedAtMajorStep ( S ) ;
( void ) memset ( & ( ( ( XDis_new_wall_model_T * ) ssGetContStateDisabled (
S ) ) -> TransferFcn2_CSTATE_a ) , 1 , 7 * sizeof ( boolean_T ) ) ; _rtB ->
B_7_42_0 [ 0 ] = _rtP -> P_2 ; _rtB -> B_7_42_0 [ 1 ] = _rtP -> P_3 ; _rtDW
-> BatteryController_MODE = false ; } } } if ( _rtDW ->
BatteryController_MODE ) { _rtB -> B_7_0_0 = 0.0 ; _rtB -> B_7_0_0 += _rtP ->
P_5 * _rtX -> TransferFcn2_CSTATE_a ; _rtB -> B_7_1_0 = _rtP -> P_6 * _rtB ->
B_7_0_0 ; _rtB -> B_7_2_0 = _rtX -> Integrator_CSTATE_j ; _rtB -> B_7_3_0 =
_rtP -> P_8 * _rtB -> B_7_0_0 ; _rtB -> B_7_4_0 = _rtX -> Filter_CSTATE_m ;
_rtB -> B_7_5_0 = _rtB -> B_7_3_0 - _rtB -> B_7_4_0 ; _rtB -> B_7_6_0 = _rtP
-> P_10 * _rtB -> B_7_5_0 ; _rtB -> B_7_7_0 = ( _rtB -> B_7_1_0 + _rtB ->
B_7_2_0 ) + _rtB -> B_7_6_0 ; if ( _rtB -> B_7_7_0 > _rtP -> P_11 ) { _rtB ->
B_7_8_0 = _rtP -> P_11 ; } else if ( _rtB -> B_7_7_0 < _rtP -> P_12 ) { _rtB
-> B_7_8_0 = _rtP -> P_12 ; } else { _rtB -> B_7_8_0 = _rtB -> B_7_7_0 ; }
_rtB -> B_7_9_0 = _rtP -> P_13 * _rtB -> B_7_8_0 ; _rtB -> B_7_11_0 = _rtB ->
B_7_9_0 + _rtB -> B_7_10_0 ; _rtB -> B_7_13_0 = _rtB -> B_31_236_0 - _rtB ->
B_7_12_0 ; _rtB -> B_7_14_0 = 0.0 ; _rtB -> B_7_14_0 += _rtP -> P_18 * _rtX
-> TransferFcn3_CSTATE_g ; _rtB -> B_7_15_0 = _rtB -> B_31_238_0 - _rtB ->
B_7_14_0 ; _rtB -> B_7_16_0 = 0.0 ; _rtB -> B_7_16_0 += _rtP -> P_20 * _rtX
-> TransferFcn1_CSTATE_h ; _rtB -> B_7_17_0 = _rtP -> P_21 * _rtB -> B_7_16_0
; _rtB -> B_7_18_0 = _rtX -> Filter_CSTATE_l ; _rtB -> B_7_19_0 = _rtB ->
B_7_17_0 - _rtB -> B_7_18_0 ; _rtB -> B_7_20_0 = _rtP -> P_23 * _rtB ->
B_7_19_0 ; _rtB -> B_7_21_0 = _rtP -> P_24 * _rtB -> B_7_16_0 ; _rtB ->
B_7_22_0 = _rtX -> Integrator_CSTATE_d ; _rtB -> B_7_23_0 = _rtP -> P_26 *
_rtB -> B_7_16_0 ; _rtB -> B_7_24_0 = ( _rtB -> B_7_23_0 + _rtB -> B_7_22_0 )
+ _rtB -> B_7_20_0 ; _rtB -> B_7_25_0 = _rtP -> P_27 * _rtB -> B_7_0_0 ;
isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_7_29_0 =
_rtP -> P_29 ; } _rtB -> B_7_30_0 = ssGetT ( S ) + _rtB -> B_7_29_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_7_31_0 = _rtP ->
P_30 ; } _rtB -> B_7_33_0 = _rtP -> P_31 * muDoubleScalarRem ( _rtB ->
B_7_30_0 , _rtB -> B_7_31_0 ) ; new_wall_model_acc_LookUp_real_T_real_T ( & (
B_7_34_0 ) , _rtP -> P_33 , _rtB -> B_7_33_0 , _rtP -> P_32 , 2U ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_7_35_0 = _rtP ->
P_34 ; } _rtB -> B_7_36_0 = B_7_34_0 - _rtB -> B_7_35_0 ; _rtB -> B_7_38_0 =
_rtB -> B_7_36_0 * _rtB -> B_7_37_0 ; _rtB -> B_7_39_0 = ( _rtB -> B_7_26_0 [
0 ] + _rtB -> B_7_38_0 ) + _rtB -> B_7_37_0 ; rtb_B_31_13_0 = ( _rtB ->
B_7_11_0 >= _rtB -> B_7_39_0 ) ; _rtB -> B_7_42_0 [ 0 ] = rtb_B_31_13_0 ;
_rtB -> B_7_42_0 [ 1 ] = ! rtb_B_31_13_0 ; if ( ssIsMajorTimeStep ( S ) != 0
) { srUpdateBC ( _rtDW -> BatteryController_SubsysRanBC ) ; } } _rtB ->
B_31_240_0 = 0.0 ; _rtB -> B_31_240_0 += _rtP -> P_265 * _rtX ->
TransferFcn5_CSTATE ; _rtB -> B_31_241_0 = 0.0 ; _rtB -> B_31_241_0 += _rtP
-> P_267 * _rtX -> TransferFcn6_CSTATE ; isHit = ssIsSampleHit ( S , 4 , 0 )
; if ( isHit != 0 ) { ssCallAccelRunBlock ( S , 13 , 0 , SS_CALL_MDL_OUTPUTS
) ; } _rtB -> B_31_244_0 = 0.0 ; _rtB -> B_31_244_0 += _rtP -> P_270 * _rtX
-> TransferFcn6_CSTATE_a ; _rtB -> B_31_246_0 = _rtX -> Integrator_CSTATE ;
_rtB -> B_31_247_0 = _rtP -> P_273 * _rtB -> B_31_244_0 ; _rtB -> B_31_248_0
= _rtX -> Filter_CSTATE ; _rtB -> B_31_249_0 = _rtB -> B_31_247_0 - _rtB ->
B_31_248_0 ; _rtB -> B_31_250_0 = _rtP -> P_275 * _rtB -> B_31_249_0 ;
rtb_B_31_38_0 = ( _rtP -> P_271 * _rtB -> B_31_244_0 + _rtB -> B_31_246_0 ) +
_rtB -> B_31_250_0 ; if ( rtb_B_31_38_0 > _rtP -> P_276 ) { rtb_B_31_38_0 =
_rtP -> P_276 ; } else { if ( rtb_B_31_38_0 < _rtP -> P_277 ) { rtb_B_31_38_0
= _rtP -> P_277 ; } } rtb_B_31_3_0 = ( _rtB -> B_31_243_0 - rtb_B_31_38_0 ) *
_rtP -> P_278 + _rtB -> B_31_255_0 ; isHit = ssIsSampleHit ( S , 4 , 0 ) ; if
( isHit != 0 ) { if ( _rtB -> B_13_0_1 > _rtP -> P_280 ) { _rtB -> B_31_257_0
= _rtP -> P_280 ; } else if ( _rtB -> B_13_0_1 < _rtP -> P_281 ) { _rtB ->
B_31_257_0 = _rtP -> P_281 ; } else { _rtB -> B_31_257_0 = _rtB -> B_13_0_1 ;
} } _rtB -> B_31_258_0 = _rtB -> B_31_240_0 - _rtB -> B_31_257_0 ; _rtB ->
B_31_259_0 = 0.0 ; _rtB -> B_31_259_0 += _rtP -> P_283 * _rtX ->
TransferFcn5_CSTATE_b ; _rtB -> B_31_260_0 = _rtP -> P_284 * _rtB ->
B_31_259_0 ; _rtB -> B_31_261_0 = _rtX -> Integrator_CSTATE_f ; _rtB ->
B_31_262_0 = _rtP -> P_286 * _rtB -> B_31_259_0 ; _rtB -> B_31_263_0 = _rtX
-> Filter_CSTATE_f ; _rtB -> B_31_264_0 = _rtB -> B_31_262_0 - _rtB ->
B_31_263_0 ; _rtB -> B_31_265_0 = _rtP -> P_288 * _rtB -> B_31_264_0 ; _rtB
-> B_31_266_0 = ( _rtB -> B_31_260_0 + _rtB -> B_31_261_0 ) + _rtB ->
B_31_265_0 ; _rtB -> B_31_267_0 = _rtB -> B_31_241_0 - _rtB -> B_31_266_0 ;
_rtB -> B_31_268_0 = _rtP -> P_289 * _rtB -> B_31_259_0 ; _rtB -> B_31_269_0
= _rtP -> P_290 * _rtB -> B_31_244_0 ; isHit = ssIsSampleHit ( S , 1 , 0 ) ;
if ( isHit != 0 ) { _rtB -> B_31_273_0 = _rtP -> P_292 ; } rtb_B_31_17_0 =
ssGetT ( S ) + _rtB -> B_31_273_0 ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if
( isHit != 0 ) { _rtB -> B_31_275_0 = _rtP -> P_293 ; } B_31_277_0 = _rtP ->
P_294 * muDoubleScalarRem ( rtb_B_31_17_0 , _rtB -> B_31_275_0 ) ;
new_wall_model_acc_LookUp_real_T_real_T ( & ( B_31_278_0 ) , _rtP -> P_296 ,
B_31_277_0 , _rtP -> P_295 , 2U ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if
( isHit != 0 ) { _rtB -> B_31_279_0 = _rtP -> P_297 ; } rtb_B_31_17_0 = ( (
B_31_278_0 - _rtB -> B_31_279_0 ) * _rtB -> B_31_281_0 + _rtB -> B_31_270_0 [
0 ] ) + _rtB -> B_31_281_0 ; rtb_B_31_13_0 = ( rtb_B_31_3_0 >= rtb_B_31_17_0
) ; rtb_B_31_46_0 = ( _rtP -> P_299 * rtb_B_31_3_0 >= rtb_B_31_17_0 ) ; _rtB
-> B_31_288_0 [ 0 ] = rtb_B_31_13_0 ; _rtB -> B_31_288_0 [ 1 ] = !
rtb_B_31_13_0 ; _rtB -> B_31_288_0 [ 2 ] = rtb_B_31_46_0 ; _rtB -> B_31_288_0
[ 3 ] = ! rtb_B_31_46_0 ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit !=
0 ) { _rtB -> B_31_300_0 = _rtP -> P_300 * _rtB -> B_31_59_0 [ 26 ] ; _rtB ->
B_31_304_0 = _rtP -> P_301 * _rtB -> B_31_59_0 [ 22 ] ; _rtB -> B_31_322_0 =
_rtP -> P_302 * _rtB -> B_31_59_0 [ 25 ] ; } ssCallAccelRunBlock ( S , 31 ,
324 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 31 , 325 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 31 , 326 ,
SS_CALL_MDL_OUTPUTS ) ; _rtB -> B_31_327_0 = _rtX -> integrator_CSTATE ; {
real_T * * uBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK .
TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK . TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ;
real_T tMinusDelay = simTime - _rtP -> P_305 ; B_31_328_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK . CircularBufSize , & _rtDW ->
TransportDelay_IWORK . Last , _rtDW -> TransportDelay_IWORK . Tail , _rtDW ->
TransportDelay_IWORK . Head , _rtP -> P_306 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_330_0 = _rtP -> P_307 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_330_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_332_0 = _rtDW -> Memory_PreviousInput ; } if ( rtb_B_31_13_0 ) { _rtB
-> B_31_334_0 = ( _rtB -> B_31_327_0 - B_31_328_0 ) * _rtP -> P_149 ; } else
{ _rtB -> B_31_334_0 = _rtB -> B_31_332_0 ; } _rtB -> B_31_335_0 = _rtX ->
integrator_CSTATE_d ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_a . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_a . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_310 ; B_31_336_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_f . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_f . Last , _rtDW -> TransportDelay_IWORK_f . Tail ,
_rtDW -> TransportDelay_IWORK_f . Head , _rtP -> P_311 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_338_0 = _rtP -> P_312 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_338_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_340_0 = _rtDW -> Memory_PreviousInput_n ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_342_0 = ( _rtB -> B_31_335_0 - B_31_336_0 ) * _rtP -> P_148 ; }
else { _rtB -> B_31_342_0 = _rtB -> B_31_340_0 ; } rtb_RealImagtoComplex_re =
_rtB -> B_31_334_0 ; rtb_RealImagtoComplex_im = _rtB -> B_31_342_0 ; _rtB ->
B_31_345_0 = _rtX -> integrator_CSTATE_b ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_o . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_o . TUbufferPtrs [ 1
] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_315 ; B_31_346_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_d .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_d . Last , _rtDW ->
TransportDelay_IWORK_d . Tail , _rtDW -> TransportDelay_IWORK_d . Head , _rtP
-> P_316 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_348_0 = _rtP -> P_317 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_348_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_350_0 = _rtDW
-> Memory_PreviousInput_p ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_352_0 = (
_rtB -> B_31_345_0 - B_31_346_0 ) * _rtP -> P_151 ; } else { _rtB ->
B_31_352_0 = _rtB -> B_31_350_0 ; } _rtB -> B_31_353_0 = _rtX ->
integrator_CSTATE_de ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_an . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T
* * ) & _rtDW -> TransportDelay_PWORK_an . TUbufferPtrs [ 1 ] ; real_T
simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_320 ;
B_31_354_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , *
tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_f1 . CircularBufSize , &
_rtDW -> TransportDelay_IWORK_f1 . Last , _rtDW -> TransportDelay_IWORK_f1 .
Tail , _rtDW -> TransportDelay_IWORK_f1 . Head , _rtP -> P_321 , 0 , (
boolean_T ) ( ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) ==
ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0
) { _rtB -> B_31_356_0 = _rtP -> P_322 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >=
_rtB -> B_31_356_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0
) { _rtB -> B_31_358_0 = _rtDW -> Memory_PreviousInput_f ; } if (
rtb_B_31_13_0 ) { _rtB -> B_31_360_0 = ( _rtB -> B_31_353_0 - B_31_354_0 ) *
_rtP -> P_150 ; } else { _rtB -> B_31_360_0 = _rtB -> B_31_358_0 ; }
rtb_B_31_3_0 = muDoubleScalarHypot ( rtb_RealImagtoComplex_re ,
rtb_RealImagtoComplex_im ) * muDoubleScalarHypot ( _rtB -> B_31_352_0 , _rtB
-> B_31_360_0 ) * _rtP -> P_324 ; muDoubleScalarSinCos ( _rtP -> P_327 * (
_rtP -> P_325 * muDoubleScalarAtan2 ( rtb_RealImagtoComplex_im ,
rtb_RealImagtoComplex_re ) - _rtP -> P_326 * muDoubleScalarAtan2 ( _rtB ->
B_31_360_0 , _rtB -> B_31_352_0 ) ) , & rtb_B_31_17_0 , & rtb_B_31_38_0 ) ;
_rtB -> B_31_370_0 = rtb_B_31_3_0 * rtb_B_31_38_0 ; _rtB -> B_31_371_0 =
rtb_B_31_3_0 * rtb_B_31_17_0 ; _rtB -> B_31_372_0 = _rtB -> B_31_322_0 * _rtB
-> B_31_52_0 ; _rtB -> B_31_373_0 = _rtX -> integrator_CSTATE_a ; { real_T *
* uBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_og . TUbufferPtrs
[ 0 ] ; real_T * * tBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_og . TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S )
; real_T tMinusDelay = simTime - _rtP -> P_329 ; B_31_374_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_m . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_m . Last , _rtDW -> TransportDelay_IWORK_m . Tail ,
_rtDW -> TransportDelay_IWORK_m . Head , _rtP -> P_330 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_376_0 = _rtP -> P_331 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_376_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_378_0 = _rtDW -> Memory_PreviousInput_e ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_380_0 = ( _rtB -> B_31_373_0 - B_31_374_0 ) * _rtP -> P_157 ; }
else { _rtB -> B_31_380_0 = _rtB -> B_31_378_0 ; } _rtB -> B_31_381_0 = _rtX
-> integrator_CSTATE_f ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_i . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_i . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_334 ; B_31_382_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_o . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_o . Last , _rtDW -> TransportDelay_IWORK_o . Tail ,
_rtDW -> TransportDelay_IWORK_o . Head , _rtP -> P_335 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_384_0 = _rtP -> P_336 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_384_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_386_0 = _rtDW -> Memory_PreviousInput_b ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_388_0 = ( _rtB -> B_31_381_0 - B_31_382_0 ) * _rtP -> P_156 ; }
else { _rtB -> B_31_388_0 = _rtB -> B_31_386_0 ; } rtb_RealImagtoComplex_re =
_rtB -> B_31_380_0 ; rtb_RealImagtoComplex_im = _rtB -> B_31_388_0 ; _rtB ->
B_31_391_0 = _rtX -> integrator_CSTATE_e ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_c . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_c . TUbufferPtrs [ 1
] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_339 ; B_31_392_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_j .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_j . Last , _rtDW ->
TransportDelay_IWORK_j . Tail , _rtDW -> TransportDelay_IWORK_j . Head , _rtP
-> P_340 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_394_0 = _rtP -> P_341 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_394_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_396_0 = _rtDW
-> Memory_PreviousInput_h ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_398_0 = (
_rtB -> B_31_391_0 - B_31_392_0 ) * _rtP -> P_159 ; } else { _rtB ->
B_31_398_0 = _rtB -> B_31_396_0 ; } _rtB -> B_31_399_0 = _rtX ->
integrator_CSTATE_g ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_l . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_l . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_344 ; B_31_400_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_dz . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_dz . Last , _rtDW -> TransportDelay_IWORK_dz . Tail ,
_rtDW -> TransportDelay_IWORK_dz . Head , _rtP -> P_345 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_402_0 = _rtP -> P_346 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_402_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_404_0 = _rtDW -> Memory_PreviousInput_l ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_406_0 = ( _rtB -> B_31_399_0 - B_31_400_0 ) * _rtP -> P_158 ; }
else { _rtB -> B_31_406_0 = _rtB -> B_31_404_0 ; } rtb_B_31_3_0 =
muDoubleScalarHypot ( rtb_RealImagtoComplex_re , rtb_RealImagtoComplex_im ) *
muDoubleScalarHypot ( _rtB -> B_31_398_0 , _rtB -> B_31_406_0 ) * _rtP ->
P_348 ; muDoubleScalarSinCos ( _rtP -> P_351 * ( _rtP -> P_349 *
muDoubleScalarAtan2 ( rtb_RealImagtoComplex_im , rtb_RealImagtoComplex_re ) -
_rtP -> P_350 * muDoubleScalarAtan2 ( _rtB -> B_31_406_0 , _rtB -> B_31_398_0
) ) , & rtb_B_31_17_0 , & rtb_B_31_38_0 ) ; _rtB -> B_31_416_0 = rtb_B_31_3_0
* rtb_B_31_38_0 ; _rtB -> B_31_417_0 = rtb_B_31_3_0 * rtb_B_31_17_0 ; _rtB ->
B_31_418_0 = _rtX -> integrator_CSTATE_eh ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_ax . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_ax . TUbufferPtrs [
1 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_353 ; B_31_419_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_p .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_p . Last , _rtDW ->
TransportDelay_IWORK_p . Tail , _rtDW -> TransportDelay_IWORK_p . Head , _rtP
-> P_354 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_421_0 = _rtP -> P_355 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_421_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_423_0 = _rtDW
-> Memory_PreviousInput_k ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_425_0 = (
_rtB -> B_31_418_0 - B_31_419_0 ) * _rtP -> P_161 ; } else { _rtB ->
B_31_425_0 = _rtB -> B_31_423_0 ; } _rtB -> B_31_426_0 = _rtX ->
integrator_CSTATE_p ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_m . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_m . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_358 ; B_31_427_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_c . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_c . Last , _rtDW -> TransportDelay_IWORK_c . Tail ,
_rtDW -> TransportDelay_IWORK_c . Head , _rtP -> P_359 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_429_0 = _rtP -> P_360 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_429_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_431_0 = _rtDW -> Memory_PreviousInput_j ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_433_0 = ( _rtB -> B_31_426_0 - B_31_427_0 ) * _rtP -> P_160 ; }
else { _rtB -> B_31_433_0 = _rtB -> B_31_431_0 ; } rtb_RealImagtoComplex_re =
_rtB -> B_31_425_0 ; rtb_RealImagtoComplex_im = _rtB -> B_31_433_0 ; _rtB ->
B_31_436_0 = _rtX -> integrator_CSTATE_o ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_is . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_is . TUbufferPtrs [
1 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_363 ; B_31_437_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_e .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_e . Last , _rtDW ->
TransportDelay_IWORK_e . Tail , _rtDW -> TransportDelay_IWORK_e . Head , _rtP
-> P_364 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_439_0 = _rtP -> P_365 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_439_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_441_0 = _rtDW
-> Memory_PreviousInput_ng ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_443_0 = (
_rtB -> B_31_436_0 - B_31_437_0 ) * _rtP -> P_163 ; } else { _rtB ->
B_31_443_0 = _rtB -> B_31_441_0 ; } _rtB -> B_31_444_0 = _rtX ->
integrator_CSTATE_m ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_j . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_j . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_368 ; B_31_445_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_n . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_n . Last , _rtDW -> TransportDelay_IWORK_n . Tail ,
_rtDW -> TransportDelay_IWORK_n . Head , _rtP -> P_369 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_447_0 = _rtP -> P_370 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_447_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_449_0 = _rtDW -> Memory_PreviousInput_g ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_451_0 = ( _rtB -> B_31_444_0 - B_31_445_0 ) * _rtP -> P_162 ; }
else { _rtB -> B_31_451_0 = _rtB -> B_31_449_0 ; } rtb_B_31_3_0 =
muDoubleScalarHypot ( rtb_RealImagtoComplex_re , rtb_RealImagtoComplex_im ) *
muDoubleScalarHypot ( _rtB -> B_31_443_0 , _rtB -> B_31_451_0 ) * _rtP ->
P_372 ; muDoubleScalarSinCos ( _rtP -> P_375 * ( _rtP -> P_373 *
muDoubleScalarAtan2 ( rtb_RealImagtoComplex_im , rtb_RealImagtoComplex_re ) -
_rtP -> P_374 * muDoubleScalarAtan2 ( _rtB -> B_31_451_0 , _rtB -> B_31_443_0
) ) , & rtb_B_31_17_0 , & rtb_B_31_38_0 ) ; _rtB -> B_31_461_0 = rtb_B_31_3_0
* rtb_B_31_38_0 ; _rtB -> B_31_462_0 = rtb_B_31_3_0 * rtb_B_31_17_0 ; _rtB ->
B_31_463_0 = _rtX -> integrator_CSTATE_k ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_jp . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_jp . TUbufferPtrs [
1 ] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_377 ; B_31_464_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_g .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_g . Last , _rtDW ->
TransportDelay_IWORK_g . Tail , _rtDW -> TransportDelay_IWORK_g . Head , _rtP
-> P_378 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_466_0 = _rtP -> P_379 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_466_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_468_0 = _rtDW
-> Memory_PreviousInput_a ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_470_0 = (
_rtB -> B_31_463_0 - B_31_464_0 ) * _rtP -> P_153 ; } else { _rtB ->
B_31_470_0 = _rtB -> B_31_468_0 ; } _rtB -> B_31_471_0 = _rtX ->
integrator_CSTATE_bu ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_e . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_e . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_382 ; B_31_472_0 =
new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * tBuffer , *
uBuffer , _rtDW -> TransportDelay_IWORK_cn . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_cn . Last , _rtDW -> TransportDelay_IWORK_cn . Tail ,
_rtDW -> TransportDelay_IWORK_cn . Head , _rtP -> P_383 , 0 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB ->
B_31_474_0 = _rtP -> P_384 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB ->
B_31_474_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB
-> B_31_476_0 = _rtDW -> Memory_PreviousInput_gn ; } if ( rtb_B_31_13_0 ) {
_rtB -> B_31_478_0 = ( _rtB -> B_31_471_0 - B_31_472_0 ) * _rtP -> P_152 ; }
else { _rtB -> B_31_478_0 = _rtB -> B_31_476_0 ; } rtb_RealImagtoComplex_re =
_rtB -> B_31_470_0 ; rtb_RealImagtoComplex_im = _rtB -> B_31_478_0 ; _rtB ->
B_31_481_0 = _rtX -> integrator_CSTATE_h ; { real_T * * uBuffer = ( real_T *
* ) & _rtDW -> TransportDelay_PWORK_b . TUbufferPtrs [ 0 ] ; real_T * *
tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_b . TUbufferPtrs [ 1
] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_387 ; B_31_482_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_i .
CircularBufSize , & _rtDW -> TransportDelay_IWORK_i . Last , _rtDW ->
TransportDelay_IWORK_i . Tail , _rtDW -> TransportDelay_IWORK_i . Head , _rtP
-> P_388 , 0 , ( boolean_T ) ( ssIsMinorTimeStep ( S ) && (
ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit (
S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_484_0 = _rtP -> P_389 ; }
rtb_B_31_13_0 = ( ssGetT ( S ) >= _rtB -> B_31_484_0 ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_31_486_0 = _rtDW
-> Memory_PreviousInput_i ; } if ( rtb_B_31_13_0 ) { _rtB -> B_31_488_0 = (
_rtB -> B_31_481_0 - B_31_482_0 ) * _rtP -> P_155 ; } else { _rtB ->
B_31_488_0 = _rtB -> B_31_486_0 ; } _rtB -> B_31_489_0 = _rtX ->
integrator_CSTATE_k4 ; { real_T * * uBuffer = ( real_T * * ) & _rtDW ->
TransportDelay_PWORK_by . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T
* * ) & _rtDW -> TransportDelay_PWORK_by . TUbufferPtrs [ 1 ] ; real_T
simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP -> P_392 ;
B_31_490_0 = new_wall_model_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , *
tBuffer , * uBuffer , _rtDW -> TransportDelay_IWORK_ex . CircularBufSize , &
_rtDW -> TransportDelay_IWORK_ex . Last , _rtDW -> TransportDelay_IWORK_ex .
Tail , _rtDW -> TransportDelay_IWORK_ex . Head , _rtP -> P_393 , 0 , (
boolean_T ) ( ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) ==
ssGetT ( S ) ) ) ) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0
) { _rtB -> B_31_492_0 = _rtP -> P_394 ; } rtb_B_31_13_0 = ( ssGetT ( S ) >=
_rtB -> B_31_492_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0
) { _rtB -> B_31_494_0 = _rtDW -> Memory_PreviousInput_li ; } if (
rtb_B_31_13_0 ) { _rtB -> B_31_496_0 = ( _rtB -> B_31_489_0 - B_31_490_0 ) *
_rtP -> P_154 ; } else { _rtB -> B_31_496_0 = _rtB -> B_31_494_0 ; }
rtb_B_31_3_0 = muDoubleScalarHypot ( rtb_RealImagtoComplex_re ,
rtb_RealImagtoComplex_im ) * muDoubleScalarHypot ( _rtB -> B_31_488_0 , _rtB
-> B_31_496_0 ) * _rtP -> P_396 ; muDoubleScalarSinCos ( _rtP -> P_399 * (
_rtP -> P_397 * muDoubleScalarAtan2 ( rtb_RealImagtoComplex_im ,
rtb_RealImagtoComplex_re ) - _rtP -> P_398 * muDoubleScalarAtan2 ( _rtB ->
B_31_496_0 , _rtB -> B_31_488_0 ) ) , & rtb_B_31_17_0 , & rtb_B_31_38_0 ) ;
_rtB -> B_31_506_0 = rtb_B_31_3_0 * rtb_B_31_38_0 ; _rtB -> B_31_507_0 =
rtb_B_31_3_0 * rtb_B_31_17_0 ; ssCallAccelRunBlock ( S , 31 , 508 ,
SS_CALL_MDL_OUTPUTS ) ; _rtB -> B_31_510_0 = ( muDoubleScalarSin ( _rtP ->
P_402 * ssGetTaskTime ( S , 0 ) + _rtP -> P_403 ) * _rtP -> P_400 + _rtP ->
P_401 ) * _rtB -> B_31_104_0 ; _rtB -> B_31_512_0 = ( muDoubleScalarSin (
_rtP -> P_406 * ssGetTaskTime ( S , 0 ) + _rtP -> P_407 ) * _rtP -> P_404 +
_rtP -> P_405 ) * _rtB -> B_31_104_0 ; _rtB -> B_31_514_0 = (
muDoubleScalarSin ( _rtP -> P_410 * ssGetTaskTime ( S , 0 ) + _rtP -> P_411 )
* _rtP -> P_408 + _rtP -> P_409 ) * _rtB -> B_31_208_0 ; _rtB -> B_31_516_0 =
( muDoubleScalarSin ( _rtP -> P_414 * ssGetTaskTime ( S , 0 ) + _rtP -> P_415
) * _rtP -> P_412 + _rtP -> P_413 ) * _rtB -> B_31_208_0 ; _rtB -> B_31_518_0
= ( muDoubleScalarSin ( _rtP -> P_418 * ssGetTaskTime ( S , 0 ) + _rtP ->
P_419 ) * _rtP -> P_416 + _rtP -> P_417 ) * _rtB -> B_31_104_0 ; _rtB ->
B_31_520_0 = ( muDoubleScalarSin ( _rtP -> P_422 * ssGetTaskTime ( S , 0 ) +
_rtP -> P_423 ) * _rtP -> P_420 + _rtP -> P_421 ) * _rtB -> B_31_104_0 ; _rtB
-> B_31_522_0 = ( muDoubleScalarSin ( _rtP -> P_426 * ssGetTaskTime ( S , 0 )
+ _rtP -> P_427 ) * _rtP -> P_424 + _rtP -> P_425 ) * _rtB -> B_31_204_0 ;
_rtB -> B_31_524_0 = ( muDoubleScalarSin ( _rtP -> P_430 * ssGetTaskTime ( S
, 0 ) + _rtP -> P_431 ) * _rtP -> P_428 + _rtP -> P_429 ) * _rtB ->
B_31_204_0 ; _rtB -> B_31_526_0 = ( muDoubleScalarSin ( _rtP -> P_434 *
ssGetTaskTime ( S , 0 ) + _rtP -> P_435 ) * _rtP -> P_432 + _rtP -> P_433 ) *
_rtB -> B_31_104_0 ; _rtB -> B_31_528_0 = ( muDoubleScalarSin ( _rtP -> P_438
* ssGetTaskTime ( S , 0 ) + _rtP -> P_439 ) * _rtP -> P_436 + _rtP -> P_437 )
* _rtB -> B_31_104_0 ; _rtB -> B_31_530_0 = ( muDoubleScalarSin ( _rtP ->
P_442 * ssGetTaskTime ( S , 0 ) + _rtP -> P_443 ) * _rtP -> P_440 + _rtP ->
P_441 ) * _rtB -> B_31_212_0 ; _rtB -> B_31_532_0 = ( muDoubleScalarSin (
_rtP -> P_446 * ssGetTaskTime ( S , 0 ) + _rtP -> P_447 ) * _rtP -> P_444 +
_rtP -> P_445 ) * _rtB -> B_31_212_0 ; _rtB -> B_31_534_0 = (
muDoubleScalarSin ( _rtP -> P_450 * ssGetTaskTime ( S , 0 ) + _rtP -> P_451 )
* _rtP -> P_448 + _rtP -> P_449 ) * _rtB -> B_31_104_0 ; _rtB -> B_31_536_0 =
( muDoubleScalarSin ( _rtP -> P_454 * ssGetTaskTime ( S , 0 ) + _rtP -> P_455
) * _rtP -> P_452 + _rtP -> P_453 ) * _rtB -> B_31_104_0 ; _rtB -> B_31_538_0
= ( muDoubleScalarSin ( _rtP -> P_458 * ssGetTaskTime ( S , 0 ) + _rtP ->
P_459 ) * _rtP -> P_456 + _rtP -> P_457 ) * _rtB -> B_31_216_0 ; _rtB ->
B_31_540_0 = ( muDoubleScalarSin ( _rtP -> P_462 * ssGetTaskTime ( S , 0 ) +
_rtP -> P_463 ) * _rtP -> P_460 + _rtP -> P_461 ) * _rtB -> B_31_216_0 ;
isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock
( S , 31 , 541 , SS_CALL_MDL_OUTPUTS ) ; if ( _rtB -> B_31_322_0 > _rtP ->
P_464 ) { _rtB -> B_31_545_0 = _rtP -> P_464 ; } else if ( _rtB -> B_31_322_0
< _rtP -> P_465 ) { _rtB -> B_31_545_0 = _rtP -> P_465 ; } else { _rtB ->
B_31_545_0 = _rtB -> B_31_322_0 ; } } UNUSED_PARAMETER ( tid ) ; } static
void mdlOutputsTID5 ( SimStruct * S , int_T tid ) { B_new_wall_model_T * _rtB
; P_new_wall_model_T * _rtP ; DW_new_wall_model_T * _rtDW ; _rtDW = ( (
DW_new_wall_model_T * ) ssGetRootDWork ( S ) ) ; _rtP = ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( ( B_new_wall_model_T
* ) _ssGetModelBlockIO ( S ) ) ; memcpy ( & _rtB -> B_31_0_0 [ 0 ] , & _rtP
-> P_164 [ 0 ] , 21U * sizeof ( real_T ) ) ; _rtB -> B_31_1_0 = _rtP -> P_165
; _rtB -> B_31_2_0 = _rtP -> P_166 ; _rtB -> B_31_14_0 = _rtP -> P_178 ; _rtB
-> B_31_22_0 = _rtP -> P_179 ; _rtB -> B_31_23_0 = _rtP -> P_180 ; _rtB ->
B_31_31_0 = _rtP -> P_183 ; _rtB -> B_31_32_0 = _rtP -> P_184 ; _rtB ->
B_31_33_0 = _rtP -> P_185 ; _rtB -> B_31_39_0 = _rtP -> P_186 ; _rtB ->
B_31_56_0 = _rtP -> P_219 ; _rtB -> B_31_57_0 = _rtP -> P_220 ; _rtB ->
B_31_58_0 = _rtP -> P_221 ; _rtB -> B_31_74_0 = _rtP -> P_237 ; _rtB ->
B_31_103_0 = _rtP -> P_245 ; _rtB -> B_12_10_0 = _rtP -> P_66 ; _rtB ->
B_12_15_0 = _rtP -> P_69 ; _rtB -> B_12_27_0 = _rtP -> P_79 ; _rtB ->
B_12_34_0 = _rtP -> P_83 ; _rtB -> B_10_3_0 = _rtP -> P_40 ; _rtB ->
B_10_11_0 = _rtP -> P_45 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC (
_rtDW -> AutomaticGainControl_SubsysRanBC ) ; } _rtB -> B_12_40_0 = _rtP ->
P_90 ; _rtB -> B_12_75_0 [ 0 ] = _rtP -> P_116 [ 0 ] ; _rtB -> B_12_75_0 [ 1
] = _rtP -> P_116 [ 1 ] ; _rtB -> B_12_86_0 = ( _rtB -> B_12_75_0 [ 1 ] -
_rtB -> B_12_75_0 [ 0 ] ) * _rtP -> P_123 ; if ( ssIsMajorTimeStep ( S ) != 0
) { srUpdateBC ( _rtDW -> InverterController_SubsysRanBC ) ; } _rtB ->
B_14_0_0 = _rtP -> P_130 ; _rtB -> B_14_2_0 = _rtP -> P_131 ; _rtB ->
B_14_4_0 [ 0 ] = _rtP -> P_132 [ 0 ] ; _rtB -> B_14_4_0 [ 1 ] = _rtP -> P_132
[ 1 ] ; _rtB -> B_14_15_0 = ( _rtB -> B_14_4_0 [ 1 ] - _rtB -> B_14_4_0 [ 0 ]
) * _rtP -> P_139 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW
-> PWMController2_SubsysRanBC ) ; } _rtB -> B_7_10_0 = _rtP -> P_14 ; if (
_rtP -> P_261 > _rtP -> P_15 ) { _rtB -> B_7_12_0 = _rtP -> P_15 ; } else if
( _rtP -> P_261 < _rtP -> P_16 ) { _rtB -> B_7_12_0 = _rtP -> P_16 ; } else {
_rtB -> B_7_12_0 = _rtP -> P_261 ; } _rtB -> B_7_26_0 [ 0 ] = _rtP -> P_28 [
0 ] ; _rtB -> B_7_26_0 [ 1 ] = _rtP -> P_28 [ 1 ] ; _rtB -> B_7_37_0 = ( _rtB
-> B_7_26_0 [ 1 ] - _rtB -> B_7_26_0 [ 0 ] ) * _rtP -> P_35 ; if (
ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
BatteryController_SubsysRanBC ) ; } _rtB -> B_31_243_0 = _rtP -> P_268 ; _rtB
-> B_31_255_0 = _rtP -> P_279 ; _rtB -> B_31_270_0 [ 0 ] = _rtP -> P_291 [ 0
] ; _rtB -> B_31_270_0 [ 1 ] = _rtP -> P_291 [ 1 ] ; _rtB -> B_31_281_0 = (
_rtB -> B_31_270_0 [ 1 ] - _rtB -> B_31_270_0 [ 0 ] ) * _rtP -> P_298 ; _rtB
-> B_31_323_0 = _rtP -> P_303 ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { real_T HoldSine ;
int32_T isHit ; B_new_wall_model_T * _rtB ; P_new_wall_model_T * _rtP ;
DW_new_wall_model_T * _rtDW ; _rtDW = ( ( DW_new_wall_model_T * )
ssGetRootDWork ( S ) ) ; _rtP = ( ( P_new_wall_model_T * ) ssGetModelRtp ( S
) ) ; _rtB = ( ( B_new_wall_model_T * ) _ssGetModelBlockIO ( S ) ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
itinit1_PreviousInput = _rtB -> B_31_74_0 ; } isHit = ssIsSampleHit ( S , 2 ,
0 ) ; if ( isHit != 0 ) { _rtDW -> Currentfilter_states = ( _rtB -> B_31_63_0
- _rtP -> P_170 [ 1 ] * _rtDW -> Currentfilter_states ) / _rtP -> P_170 [ 0 ]
; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
itinit_PreviousInput = _rtB -> B_31_83_0 ; } isHit = ssIsSampleHit ( S , 2 ,
0 ) ; if ( isHit != 0 ) { _rtDW -> inti_IC_LOADING = 0U ; _rtDW ->
inti_DSTATE += _rtP -> P_174 * _rtB -> B_31_63_0 ; if ( _rtDW -> inti_DSTATE
>= _rtP -> P_175 ) { _rtDW -> inti_DSTATE = _rtP -> P_175 ; } else { if (
_rtDW -> inti_DSTATE <= _rtP -> P_176 ) { _rtDW -> inti_DSTATE = _rtP ->
P_176 ; } } if ( _rtB -> B_31_9_0 > 0.0 ) { _rtDW -> inti_PrevResetState = 1
; } else if ( _rtB -> B_31_9_0 < 0.0 ) { _rtDW -> inti_PrevResetState = - 1 ;
} else if ( _rtB -> B_31_9_0 == 0.0 ) { _rtDW -> inti_PrevResetState = 0 ; }
else { _rtDW -> inti_PrevResetState = 2 ; } _rtDW ->
DiscreteTimeIntegrator_DSTATE += _rtP -> P_187 * _rtB -> B_31_82_0 ; _rtDW ->
Memory2_PreviousInput = _rtB -> B_31_84_0 ; HoldSine = _rtDW -> lastSin ;
_rtDW -> lastSin = _rtDW -> lastSin * _rtP -> P_202 + _rtDW -> lastCos * _rtP
-> P_201 ; _rtDW -> lastCos = _rtDW -> lastCos * _rtP -> P_202 - HoldSine *
_rtP -> P_201 ; HoldSine = _rtDW -> lastSin_i ; _rtDW -> lastSin_i = _rtDW ->
lastSin_i * _rtP -> P_209 + _rtDW -> lastCos_o * _rtP -> P_208 ; _rtDW ->
lastCos_o = _rtDW -> lastCos_o * _rtP -> P_209 - HoldSine * _rtP -> P_208 ;
HoldSine = _rtDW -> lastSin_g ; _rtDW -> lastSin_g = _rtDW -> lastSin_g *
_rtP -> P_216 + _rtDW -> lastCos_j * _rtP -> P_215 ; _rtDW -> lastCos_j =
_rtDW -> lastCos_j * _rtP -> P_216 - HoldSine * _rtP -> P_215 ;
ssCallAccelRunBlock ( S , 31 , 59 , SS_CALL_MDL_UPDATE ) ; } if ( _rtDW ->
InverterController_MODE ) { _rtDW -> Integrator_IWORK = 0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory1_PreviousInput = _rtB -> B_12_74_0 ; } if ( _rtDW ->
AutomaticGainControl_MODE ) { { real_T * * uBuffer = ( real_T * * ) & _rtDW
-> VariableTransportDelay_PWORK_b . TUbufferPtrs [ 0 ] ; real_T * * tBuffer =
( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK_b . TUbufferPtrs [ 1 ]
; real_T * * xBuffer = ( real_T * * ) & _rtDW ->
VariableTransportDelay_PWORK_b . TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT
( S ) ; _rtDW -> VariableTransportDelay_IWORK_o . Head = ( ( _rtDW ->
VariableTransportDelay_IWORK_o . Head < ( _rtDW ->
VariableTransportDelay_IWORK_o . CircularBufSize - 1 ) ) ? ( _rtDW ->
VariableTransportDelay_IWORK_o . Head + 1 ) : 0 ) ; if ( _rtDW ->
VariableTransportDelay_IWORK_o . Head == _rtDW ->
VariableTransportDelay_IWORK_o . Tail ) { if ( !
new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
VariableTransportDelay_IWORK_o . CircularBufSize , & _rtDW ->
VariableTransportDelay_IWORK_o . Tail , & _rtDW ->
VariableTransportDelay_IWORK_o . Head , & _rtDW ->
VariableTransportDelay_IWORK_o . Last , simTime - _rtP -> P_37 , tBuffer ,
uBuffer , xBuffer , ( boolean_T ) 0 , ( boolean_T ) 1 , & _rtDW ->
VariableTransportDelay_IWORK_o . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"vtdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
VariableTransportDelay_IWORK_o . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
VariableTransportDelay_IWORK_o . Head ] = _rtB -> B_10_1_0 ; ( * xBuffer ) [
_rtDW -> VariableTransportDelay_IWORK_o . Head ] = ( ( X_new_wall_model_T * )
ssGetContStates ( S ) ) -> VariableTransportDelay_CSTATE_m ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_cc = _rtB -> B_10_7_0 ; } { real_T * * uBuffer = (
real_T * * ) & _rtDW -> VariableTransportDelay_PWORK_g . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK_g
. TUbufferPtrs [ 1 ] ; real_T * * xBuffer = ( real_T * * ) & _rtDW ->
VariableTransportDelay_PWORK_g . TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT
( S ) ; _rtDW -> VariableTransportDelay_IWORK_p . Head = ( ( _rtDW ->
VariableTransportDelay_IWORK_p . Head < ( _rtDW ->
VariableTransportDelay_IWORK_p . CircularBufSize - 1 ) ) ? ( _rtDW ->
VariableTransportDelay_IWORK_p . Head + 1 ) : 0 ) ; if ( _rtDW ->
VariableTransportDelay_IWORK_p . Head == _rtDW ->
VariableTransportDelay_IWORK_p . Tail ) { if ( !
new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
VariableTransportDelay_IWORK_p . CircularBufSize , & _rtDW ->
VariableTransportDelay_IWORK_p . Tail , & _rtDW ->
VariableTransportDelay_IWORK_p . Head , & _rtDW ->
VariableTransportDelay_IWORK_p . Last , simTime - _rtP -> P_42 , tBuffer ,
uBuffer , xBuffer , ( boolean_T ) 0 , ( boolean_T ) 1 , & _rtDW ->
VariableTransportDelay_IWORK_p . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"vtdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
VariableTransportDelay_IWORK_p . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
VariableTransportDelay_IWORK_p . Head ] = _rtB -> B_10_9_0 ; ( * xBuffer ) [
_rtDW -> VariableTransportDelay_IWORK_p . Head ] = ( ( X_new_wall_model_T * )
ssGetContStates ( S ) ) -> VariableTransportDelay_CSTATE_c ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_cz = _rtB -> B_10_15_0 ; } } { real_T * * uBuffer = (
real_T * * ) & _rtDW -> VariableTransportDelay_PWORK . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> VariableTransportDelay_PWORK .
TUbufferPtrs [ 1 ] ; real_T * * xBuffer = ( real_T * * ) & _rtDW ->
VariableTransportDelay_PWORK . TUbufferPtrs [ 2 ] ; real_T simTime = ssGetT (
S ) ; _rtDW -> VariableTransportDelay_IWORK . Head = ( ( _rtDW ->
VariableTransportDelay_IWORK . Head < ( _rtDW -> VariableTransportDelay_IWORK
. CircularBufSize - 1 ) ) ? ( _rtDW -> VariableTransportDelay_IWORK . Head +
1 ) : 0 ) ; if ( _rtDW -> VariableTransportDelay_IWORK . Head == _rtDW ->
VariableTransportDelay_IWORK . Tail ) { if ( !
new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
VariableTransportDelay_IWORK . CircularBufSize , & _rtDW ->
VariableTransportDelay_IWORK . Tail , & _rtDW -> VariableTransportDelay_IWORK
. Head , & _rtDW -> VariableTransportDelay_IWORK . Last , simTime - _rtP ->
P_87 , tBuffer , uBuffer , xBuffer , ( boolean_T ) 0 , ( boolean_T ) 1 , &
_rtDW -> VariableTransportDelay_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus
( S , "vtdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [
_rtDW -> VariableTransportDelay_IWORK . Head ] = simTime ; ( * uBuffer ) [
_rtDW -> VariableTransportDelay_IWORK . Head ] = _rtB -> B_12_38_0 ; ( *
xBuffer ) [ _rtDW -> VariableTransportDelay_IWORK . Head ] = ( (
X_new_wall_model_T * ) ssGetContStates ( S ) ) ->
VariableTransportDelay_CSTATE ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if (
isHit != 0 ) { _rtDW -> Memory_PreviousInput_c = _rtB -> B_12_44_0 ; } if (
_rtDW -> LastMajorTimeA == ( rtInf ) ) { _rtDW -> LastMajorTimeA =
ssGetTaskTime ( S , 0 ) ; _rtDW -> PrevYA = _rtB -> B_12_57_0 ; } else if (
_rtDW -> LastMajorTimeB == ( rtInf ) ) { _rtDW -> LastMajorTimeB =
ssGetTaskTime ( S , 0 ) ; _rtDW -> PrevYB = _rtB -> B_12_57_0 ; } else if (
_rtDW -> LastMajorTimeA < _rtDW -> LastMajorTimeB ) { _rtDW -> LastMajorTimeA
= ssGetTaskTime ( S , 0 ) ; _rtDW -> PrevYA = _rtB -> B_12_57_0 ; } else {
_rtDW -> LastMajorTimeB = ssGetTaskTime ( S , 0 ) ; _rtDW -> PrevYB = _rtB ->
B_12_57_0 ; } } if ( _rtDW -> PWMController2_MODE ) { isHit = ssIsSampleHit (
S , 2 , 0 ) ; if ( isHit != 0 ) { HoldSine = _rtDW -> lastSin_c ; _rtDW ->
lastSin_c = _rtDW -> lastSin_c * _rtP -> P_144 + _rtDW -> lastCos_jx * _rtP
-> P_143 ; _rtDW -> lastCos_jx = _rtDW -> lastCos_jx * _rtP -> P_144 -
HoldSine * _rtP -> P_143 ; } } { real_T * * uBuffer = ( real_T * * ) & _rtDW
-> TransportDelay_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = ( real_T
* * ) & _rtDW -> TransportDelay_PWORK . TUbufferPtrs [ 1 ] ; real_T simTime =
ssGetT ( S ) ; _rtDW -> TransportDelay_IWORK . Head = ( ( _rtDW ->
TransportDelay_IWORK . Head < ( _rtDW -> TransportDelay_IWORK .
CircularBufSize - 1 ) ) ? ( _rtDW -> TransportDelay_IWORK . Head + 1 ) : 0 )
; if ( _rtDW -> TransportDelay_IWORK . Head == _rtDW -> TransportDelay_IWORK
. Tail ) { if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW
-> TransportDelay_IWORK . CircularBufSize , & _rtDW -> TransportDelay_IWORK .
Tail , & _rtDW -> TransportDelay_IWORK . Head , & _rtDW ->
TransportDelay_IWORK . Last , simTime - _rtP -> P_305 , tBuffer , uBuffer , (
NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK . Head ] = _rtB ->
B_31_327_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput = _rtB -> B_31_334_0 ; } { real_T * * uBuffer =
( real_T * * ) & _rtDW -> TransportDelay_PWORK_a . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_a .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_f . Head = ( ( _rtDW -> TransportDelay_IWORK_f . Head <
( _rtDW -> TransportDelay_IWORK_f . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_f . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_f . Head == _rtDW -> TransportDelay_IWORK_f . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_f . CircularBufSize , & _rtDW -> TransportDelay_IWORK_f
. Tail , & _rtDW -> TransportDelay_IWORK_f . Head , & _rtDW ->
TransportDelay_IWORK_f . Last , simTime - _rtP -> P_310 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_f .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_f . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_f . Head ] = _rtB ->
B_31_335_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_n = _rtB -> B_31_342_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_o . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_o .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_d . Head = ( ( _rtDW -> TransportDelay_IWORK_d . Head <
( _rtDW -> TransportDelay_IWORK_d . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_d . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_d . Head == _rtDW -> TransportDelay_IWORK_d . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_d . CircularBufSize , & _rtDW -> TransportDelay_IWORK_d
. Tail , & _rtDW -> TransportDelay_IWORK_d . Head , & _rtDW ->
TransportDelay_IWORK_d . Last , simTime - _rtP -> P_315 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_d .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_d . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_d . Head ] = _rtB ->
B_31_345_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_p = _rtB -> B_31_352_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_an . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_an .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_f1 . Head = ( ( _rtDW -> TransportDelay_IWORK_f1 . Head
< ( _rtDW -> TransportDelay_IWORK_f1 . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_f1 . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_f1 . Head == _rtDW -> TransportDelay_IWORK_f1 . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_f1 . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_f1 . Tail , & _rtDW -> TransportDelay_IWORK_f1 . Head ,
& _rtDW -> TransportDelay_IWORK_f1 . Last , simTime - _rtP -> P_320 , tBuffer
, uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay_IWORK_f1 . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
TransportDelay_IWORK_f1 . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay_IWORK_f1 . Head ] = _rtB -> B_31_353_0 ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_f = _rtB -> B_31_360_0 ; } { real_T * * uBuffer = (
real_T * * ) & _rtDW -> TransportDelay_PWORK_og . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_og .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_m . Head = ( ( _rtDW -> TransportDelay_IWORK_m . Head <
( _rtDW -> TransportDelay_IWORK_m . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_m . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_m . Head == _rtDW -> TransportDelay_IWORK_m . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_m . CircularBufSize , & _rtDW -> TransportDelay_IWORK_m
. Tail , & _rtDW -> TransportDelay_IWORK_m . Head , & _rtDW ->
TransportDelay_IWORK_m . Last , simTime - _rtP -> P_329 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_m .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_m . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_m . Head ] = _rtB ->
B_31_373_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_e = _rtB -> B_31_380_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_i . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_i .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_o . Head = ( ( _rtDW -> TransportDelay_IWORK_o . Head <
( _rtDW -> TransportDelay_IWORK_o . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_o . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_o . Head == _rtDW -> TransportDelay_IWORK_o . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_o . CircularBufSize , & _rtDW -> TransportDelay_IWORK_o
. Tail , & _rtDW -> TransportDelay_IWORK_o . Head , & _rtDW ->
TransportDelay_IWORK_o . Last , simTime - _rtP -> P_334 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_o .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_o . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_o . Head ] = _rtB ->
B_31_381_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_b = _rtB -> B_31_388_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_c . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_c .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_j . Head = ( ( _rtDW -> TransportDelay_IWORK_j . Head <
( _rtDW -> TransportDelay_IWORK_j . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_j . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_j . Head == _rtDW -> TransportDelay_IWORK_j . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_j . CircularBufSize , & _rtDW -> TransportDelay_IWORK_j
. Tail , & _rtDW -> TransportDelay_IWORK_j . Head , & _rtDW ->
TransportDelay_IWORK_j . Last , simTime - _rtP -> P_339 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_j .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_j . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_j . Head ] = _rtB ->
B_31_391_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_h = _rtB -> B_31_398_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_l . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_l .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_dz . Head = ( ( _rtDW -> TransportDelay_IWORK_dz . Head
< ( _rtDW -> TransportDelay_IWORK_dz . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_dz . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_dz . Head == _rtDW -> TransportDelay_IWORK_dz . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_dz . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_dz . Tail , & _rtDW -> TransportDelay_IWORK_dz . Head ,
& _rtDW -> TransportDelay_IWORK_dz . Last , simTime - _rtP -> P_344 , tBuffer
, uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay_IWORK_dz . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
TransportDelay_IWORK_dz . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay_IWORK_dz . Head ] = _rtB -> B_31_399_0 ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_l = _rtB -> B_31_406_0 ; } { real_T * * uBuffer = (
real_T * * ) & _rtDW -> TransportDelay_PWORK_ax . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_ax .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_p . Head = ( ( _rtDW -> TransportDelay_IWORK_p . Head <
( _rtDW -> TransportDelay_IWORK_p . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_p . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_p . Head == _rtDW -> TransportDelay_IWORK_p . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_p . CircularBufSize , & _rtDW -> TransportDelay_IWORK_p
. Tail , & _rtDW -> TransportDelay_IWORK_p . Head , & _rtDW ->
TransportDelay_IWORK_p . Last , simTime - _rtP -> P_353 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_p .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_p . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_p . Head ] = _rtB ->
B_31_418_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_k = _rtB -> B_31_425_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_m . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_m .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_c . Head = ( ( _rtDW -> TransportDelay_IWORK_c . Head <
( _rtDW -> TransportDelay_IWORK_c . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_c . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_c . Head == _rtDW -> TransportDelay_IWORK_c . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_c . CircularBufSize , & _rtDW -> TransportDelay_IWORK_c
. Tail , & _rtDW -> TransportDelay_IWORK_c . Head , & _rtDW ->
TransportDelay_IWORK_c . Last , simTime - _rtP -> P_358 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_c .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_c . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_c . Head ] = _rtB ->
B_31_426_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_j = _rtB -> B_31_433_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_is . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_is .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_e . Head = ( ( _rtDW -> TransportDelay_IWORK_e . Head <
( _rtDW -> TransportDelay_IWORK_e . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_e . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_e . Head == _rtDW -> TransportDelay_IWORK_e . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_e . CircularBufSize , & _rtDW -> TransportDelay_IWORK_e
. Tail , & _rtDW -> TransportDelay_IWORK_e . Head , & _rtDW ->
TransportDelay_IWORK_e . Last , simTime - _rtP -> P_363 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_e .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_e . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_e . Head ] = _rtB ->
B_31_436_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_ng = _rtB -> B_31_443_0 ; } { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_j . TUbufferPtrs [ 0
] ; real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_j .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_n . Head = ( ( _rtDW -> TransportDelay_IWORK_n . Head <
( _rtDW -> TransportDelay_IWORK_n . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_n . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_n . Head == _rtDW -> TransportDelay_IWORK_n . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_n . CircularBufSize , & _rtDW -> TransportDelay_IWORK_n
. Tail , & _rtDW -> TransportDelay_IWORK_n . Head , & _rtDW ->
TransportDelay_IWORK_n . Last , simTime - _rtP -> P_368 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_n .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_n . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_n . Head ] = _rtB ->
B_31_444_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_g = _rtB -> B_31_451_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_jp . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_jp .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_g . Head = ( ( _rtDW -> TransportDelay_IWORK_g . Head <
( _rtDW -> TransportDelay_IWORK_g . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_g . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_g . Head == _rtDW -> TransportDelay_IWORK_g . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_g . CircularBufSize , & _rtDW -> TransportDelay_IWORK_g
. Tail , & _rtDW -> TransportDelay_IWORK_g . Head , & _rtDW ->
TransportDelay_IWORK_g . Last , simTime - _rtP -> P_377 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_g .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_g . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_g . Head ] = _rtB ->
B_31_463_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_a = _rtB -> B_31_470_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_e . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_e .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_cn . Head = ( ( _rtDW -> TransportDelay_IWORK_cn . Head
< ( _rtDW -> TransportDelay_IWORK_cn . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_cn . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_cn . Head == _rtDW -> TransportDelay_IWORK_cn . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_cn . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_cn . Tail , & _rtDW -> TransportDelay_IWORK_cn . Head ,
& _rtDW -> TransportDelay_IWORK_cn . Last , simTime - _rtP -> P_382 , tBuffer
, uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay_IWORK_cn . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
TransportDelay_IWORK_cn . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay_IWORK_cn . Head ] = _rtB -> B_31_471_0 ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_gn = _rtB -> B_31_478_0 ; } { real_T * * uBuffer = (
real_T * * ) & _rtDW -> TransportDelay_PWORK_b . TUbufferPtrs [ 0 ] ; real_T
* * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_b . TUbufferPtrs
[ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW -> TransportDelay_IWORK_i .
Head = ( ( _rtDW -> TransportDelay_IWORK_i . Head < ( _rtDW ->
TransportDelay_IWORK_i . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_i . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_i . Head == _rtDW -> TransportDelay_IWORK_i . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_i . CircularBufSize , & _rtDW -> TransportDelay_IWORK_i
. Tail , & _rtDW -> TransportDelay_IWORK_i . Head , & _rtDW ->
TransportDelay_IWORK_i . Last , simTime - _rtP -> P_387 , tBuffer , uBuffer ,
( NULL ) , ( boolean_T ) 0 , false , & _rtDW -> TransportDelay_IWORK_i .
MaxNewBufSize ) ) { ssSetErrorStatus ( S , "tdelay memory allocation error" )
; return ; } } ( * tBuffer ) [ _rtDW -> TransportDelay_IWORK_i . Head ] =
simTime ; ( * uBuffer ) [ _rtDW -> TransportDelay_IWORK_i . Head ] = _rtB ->
B_31_481_0 ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) {
_rtDW -> Memory_PreviousInput_i = _rtB -> B_31_488_0 ; } { real_T * * uBuffer
= ( real_T * * ) & _rtDW -> TransportDelay_PWORK_by . TUbufferPtrs [ 0 ] ;
real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK_by .
TUbufferPtrs [ 1 ] ; real_T simTime = ssGetT ( S ) ; _rtDW ->
TransportDelay_IWORK_ex . Head = ( ( _rtDW -> TransportDelay_IWORK_ex . Head
< ( _rtDW -> TransportDelay_IWORK_ex . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK_ex . Head + 1 ) : 0 ) ; if ( _rtDW ->
TransportDelay_IWORK_ex . Head == _rtDW -> TransportDelay_IWORK_ex . Tail ) {
if ( ! new_wall_model_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK_ex . CircularBufSize , & _rtDW ->
TransportDelay_IWORK_ex . Tail , & _rtDW -> TransportDelay_IWORK_ex . Head ,
& _rtDW -> TransportDelay_IWORK_ex . Last , simTime - _rtP -> P_392 , tBuffer
, uBuffer , ( NULL ) , ( boolean_T ) 0 , false , & _rtDW ->
TransportDelay_IWORK_ex . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ) [ _rtDW ->
TransportDelay_IWORK_ex . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay_IWORK_ex . Head ] = _rtB -> B_31_489_0 ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { _rtDW ->
Memory_PreviousInput_li = _rtB -> B_31_496_0 ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdateTID5 ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER (
tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { boolean_T lsat ; boolean_T
usat ; B_new_wall_model_T * _rtB ; P_new_wall_model_T * _rtP ;
X_new_wall_model_T * _rtX ; XDot_new_wall_model_T * _rtXdot ;
DW_new_wall_model_T * _rtDW ; _rtDW = ( ( DW_new_wall_model_T * )
ssGetRootDWork ( S ) ) ; _rtXdot = ( ( XDot_new_wall_model_T * ) ssGetdX ( S
) ) ; _rtX = ( ( X_new_wall_model_T * ) ssGetContStates ( S ) ) ; _rtP = ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( ( B_new_wall_model_T
* ) _ssGetModelBlockIO ( S ) ) ; _rtXdot -> StateSpace_CSTATE = 0.0 ; _rtXdot
-> StateSpace_CSTATE += _rtP -> P_190 * _rtX -> StateSpace_CSTATE ; _rtXdot
-> StateSpace_CSTATE += _rtP -> P_191 * _rtB -> B_31_545_0 ; _rtXdot ->
TransferFcn8_CSTATE = 0.0 ; _rtXdot -> TransferFcn8_CSTATE += _rtP -> P_243 *
_rtX -> TransferFcn8_CSTATE ; _rtXdot -> TransferFcn8_CSTATE += _rtB ->
B_31_148_0 ; _rtXdot -> TransferFcn1_CSTATE = 0.0 ; _rtXdot ->
TransferFcn1_CSTATE += _rtP -> P_247 * _rtX -> TransferFcn1_CSTATE ; _rtXdot
-> TransferFcn1_CSTATE += _rtB -> B_31_208_0 ; if ( _rtDW ->
InverterController_MODE ) { _rtXdot -> TransferFcn2_CSTATE_g = 0.0 ; _rtXdot
-> TransferFcn2_CSTATE_g += _rtP -> P_56 * _rtX -> TransferFcn2_CSTATE_g ;
_rtXdot -> TransferFcn2_CSTATE_g += _rtB -> B_12_14_0 ; _rtXdot ->
Integrator_CSTATE_i = _rtB -> B_12_26_0 ; _rtXdot -> Filter_CSTATE_k = _rtB
-> B_12_6_0 ; _rtXdot -> TransferFcn3_CSTATE_n = 0.0 ; _rtXdot ->
TransferFcn3_CSTATE_n += _rtP -> P_67 * _rtX -> TransferFcn3_CSTATE_n ;
_rtXdot -> TransferFcn3_CSTATE_n += _rtB -> B_12_96_0 ; _rtXdot ->
TransferFcn1_CSTATE_d = 0.0 ; _rtXdot -> TransferFcn1_CSTATE_d += _rtP ->
P_70 * _rtX -> TransferFcn1_CSTATE_d ; _rtXdot -> TransferFcn1_CSTATE_d +=
_rtB -> B_12_12_0 ; _rtXdot -> Filter_CSTATE_i = _rtB -> B_12_21_0 ; _rtXdot
-> Integrator_CSTATE_m = _rtB -> B_12_22_0 ; _rtXdot -> Integrator_CSTATE_l =
_rtB -> B_12_51_0 ; if ( _rtDW -> AutomaticGainControl_MODE ) { { real_T
instantDelay ; instantDelay = _rtB -> B_10_22_0 ; if ( instantDelay > _rtP ->
P_37 ) { instantDelay = _rtP -> P_37 ; } if ( instantDelay < 0.0 ) { ( (
XDot_new_wall_model_T * ) ssGetdX ( S ) ) -> VariableTransportDelay_CSTATE_m
= 0 ; } else { ( ( XDot_new_wall_model_T * ) ssGetdX ( S ) ) ->
VariableTransportDelay_CSTATE_m = 1.0 / instantDelay ; } } _rtXdot ->
integrator_CSTATE_n = _rtB -> B_10_25_0 ; { real_T instantDelay ;
instantDelay = _rtB -> B_10_24_0 ; if ( instantDelay > _rtP -> P_42 ) {
instantDelay = _rtP -> P_42 ; } if ( instantDelay < 0.0 ) { ( (
XDot_new_wall_model_T * ) ssGetdX ( S ) ) -> VariableTransportDelay_CSTATE_c
= 0 ; } else { ( ( XDot_new_wall_model_T * ) ssGetdX ( S ) ) ->
VariableTransportDelay_CSTATE_c = 1.0 / instantDelay ; } } _rtXdot ->
integrator_CSTATE_b4 = _rtB -> B_10_26_0 ; } else { { real_T * dx ; int_T i ;
dx = & ( ( ( XDot_new_wall_model_T * ) ssGetdX ( S ) ) ->
VariableTransportDelay_CSTATE_m ) ; for ( i = 0 ; i < 4 ; i ++ ) { dx [ i ] =
0.0 ; } } } lsat = ( _rtX -> Integrator_CSTATE_f5 <= _rtP -> P_86 ) ; usat =
( _rtX -> Integrator_CSTATE_f5 >= _rtP -> P_85 ) ; if ( ( ( ! lsat ) && ( !
usat ) ) || ( lsat && ( _rtB -> B_12_47_0 > 0.0 ) ) || ( usat && ( _rtB ->
B_12_47_0 < 0.0 ) ) ) { _rtXdot -> Integrator_CSTATE_f5 = _rtB -> B_12_47_0 ;
} else { _rtXdot -> Integrator_CSTATE_f5 = 0.0 ; } { real_T instantDelay ;
instantDelay = _rtB -> B_12_54_0 ; if ( instantDelay > _rtP -> P_87 ) {
instantDelay = _rtP -> P_87 ; } if ( instantDelay < 0.0 ) { ( (
XDot_new_wall_model_T * ) ssGetdX ( S ) ) -> VariableTransportDelay_CSTATE =
0 ; } else { ( ( XDot_new_wall_model_T * ) ssGetdX ( S ) ) ->
VariableTransportDelay_CSTATE = 1.0 / instantDelay ; } } _rtXdot ->
integrator_CSTATE_e2 = _rtB -> B_12_56_0 ; _rtXdot -> TransferFcn_CSTATE_j =
0.0 ; _rtXdot -> TransferFcn_CSTATE_j += _rtP -> P_95 * _rtX ->
TransferFcn_CSTATE_j ; _rtXdot -> TransferFcn_CSTATE_j += _rtB -> B_12_48_0 ;
_rtXdot -> Integrator_x1_CSTATE = _rtB -> B_12_67_0 ; _rtXdot ->
Integrator_x2_CSTATE = _rtB -> B_12_69_0 ; } else { { real_T * dx ; int_T i ;
dx = & ( ( ( XDot_new_wall_model_T * ) ssGetdX ( S ) ) ->
TransferFcn2_CSTATE_g ) ; for ( i = 0 ; i < 18 ; i ++ ) { dx [ i ] = 0.0 ; }
} } _rtXdot -> TransferFcn3_CSTATE = 0.0 ; _rtXdot -> TransferFcn3_CSTATE +=
_rtP -> P_249 * _rtX -> TransferFcn3_CSTATE ; _rtXdot -> TransferFcn3_CSTATE
+= _rtB -> B_31_148_0 ; _rtXdot -> TransferFcn_CSTATE = 0.0 ; _rtXdot ->
TransferFcn_CSTATE += _rtP -> P_252 * _rtX -> TransferFcn_CSTATE ; _rtXdot ->
TransferFcn_CSTATE += _rtB -> B_31_149_0 ; _rtXdot -> TransferFcn_CSTATE_p =
0.0 ; _rtXdot -> TransferFcn_CSTATE_p += _rtP -> P_259 * _rtX ->
TransferFcn_CSTATE_p ; _rtXdot -> TransferFcn_CSTATE_p += _rtB -> B_31_148_0
; _rtXdot -> TransferFcn2_CSTATE = 0.0 ; _rtXdot -> TransferFcn2_CSTATE +=
_rtP -> P_262 * _rtX -> TransferFcn2_CSTATE ; _rtXdot -> TransferFcn2_CSTATE
+= _rtB -> B_31_138_0 ; if ( _rtDW -> BatteryController_MODE ) { _rtXdot ->
TransferFcn2_CSTATE_a = 0.0 ; _rtXdot -> TransferFcn2_CSTATE_a += _rtP -> P_4
* _rtX -> TransferFcn2_CSTATE_a ; _rtXdot -> TransferFcn2_CSTATE_a += _rtB ->
B_7_15_0 ; _rtXdot -> Integrator_CSTATE_j = _rtB -> B_7_25_0 ; _rtXdot ->
Filter_CSTATE_m = _rtB -> B_7_6_0 ; _rtXdot -> TransferFcn3_CSTATE_g = 0.0 ;
_rtXdot -> TransferFcn3_CSTATE_g += _rtP -> P_17 * _rtX ->
TransferFcn3_CSTATE_g ; _rtXdot -> TransferFcn3_CSTATE_g += _rtB -> B_7_24_0
; _rtXdot -> TransferFcn1_CSTATE_h = 0.0 ; _rtXdot -> TransferFcn1_CSTATE_h
+= _rtP -> P_19 * _rtX -> TransferFcn1_CSTATE_h ; _rtXdot ->
TransferFcn1_CSTATE_h += _rtB -> B_7_13_0 ; _rtXdot -> Filter_CSTATE_l = _rtB
-> B_7_20_0 ; _rtXdot -> Integrator_CSTATE_d = _rtB -> B_7_21_0 ; } else { {
real_T * dx ; int_T i ; dx = & ( ( ( XDot_new_wall_model_T * ) ssGetdX ( S )
) -> TransferFcn2_CSTATE_a ) ; for ( i = 0 ; i < 7 ; i ++ ) { dx [ i ] = 0.0
; } } } _rtXdot -> TransferFcn5_CSTATE = 0.0 ; _rtXdot -> TransferFcn5_CSTATE
+= _rtP -> P_264 * _rtX -> TransferFcn5_CSTATE ; _rtXdot ->
TransferFcn5_CSTATE += _rtB -> B_31_322_0 ; _rtXdot -> TransferFcn6_CSTATE =
0.0 ; _rtXdot -> TransferFcn6_CSTATE += _rtP -> P_266 * _rtX ->
TransferFcn6_CSTATE ; _rtXdot -> TransferFcn6_CSTATE += _rtB -> B_31_52_0 ;
_rtXdot -> TransferFcn6_CSTATE_a = 0.0 ; _rtXdot -> TransferFcn6_CSTATE_a +=
_rtP -> P_269 * _rtX -> TransferFcn6_CSTATE_a ; _rtXdot ->
TransferFcn6_CSTATE_a += _rtB -> B_31_267_0 ; _rtXdot -> Integrator_CSTATE =
_rtB -> B_31_269_0 ; _rtXdot -> Filter_CSTATE = _rtB -> B_31_250_0 ; _rtXdot
-> TransferFcn5_CSTATE_b = 0.0 ; _rtXdot -> TransferFcn5_CSTATE_b += _rtP ->
P_282 * _rtX -> TransferFcn5_CSTATE_b ; _rtXdot -> TransferFcn5_CSTATE_b +=
_rtB -> B_31_258_0 ; _rtXdot -> Integrator_CSTATE_f = _rtB -> B_31_268_0 ;
_rtXdot -> Filter_CSTATE_f = _rtB -> B_31_265_0 ; _rtXdot ->
integrator_CSTATE = _rtB -> B_31_510_0 ; _rtXdot -> integrator_CSTATE_d =
_rtB -> B_31_512_0 ; _rtXdot -> integrator_CSTATE_b = _rtB -> B_31_514_0 ;
_rtXdot -> integrator_CSTATE_de = _rtB -> B_31_516_0 ; _rtXdot ->
integrator_CSTATE_a = _rtB -> B_31_526_0 ; _rtXdot -> integrator_CSTATE_f =
_rtB -> B_31_528_0 ; _rtXdot -> integrator_CSTATE_e = _rtB -> B_31_530_0 ;
_rtXdot -> integrator_CSTATE_g = _rtB -> B_31_532_0 ; _rtXdot ->
integrator_CSTATE_eh = _rtB -> B_31_534_0 ; _rtXdot -> integrator_CSTATE_p =
_rtB -> B_31_536_0 ; _rtXdot -> integrator_CSTATE_o = _rtB -> B_31_538_0 ;
_rtXdot -> integrator_CSTATE_m = _rtB -> B_31_540_0 ; _rtXdot ->
integrator_CSTATE_k = _rtB -> B_31_518_0 ; _rtXdot -> integrator_CSTATE_bu =
_rtB -> B_31_520_0 ; _rtXdot -> integrator_CSTATE_h = _rtB -> B_31_522_0 ;
_rtXdot -> integrator_CSTATE_k4 = _rtB -> B_31_524_0 ; } static void
mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 , 3834780148U
) ; ssSetChecksumVal ( S , 1 , 700643381U ) ; ssSetChecksumVal ( S , 2 ,
1108402200U ) ; ssSetChecksumVal ( S , 3 , 348679064U ) ; { mxArray *
slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" ) ;
char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat , 1
, & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat = mxGetField
( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) { status = 1 ; }
else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; } }
mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
status == 1 ) || ( strcmp ( slVerChar , "9.0" ) != 0 ) ) { return ; } }
ssSetOptions ( S , SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork (
S ) != sizeof ( DW_new_wall_model_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( B_new_wall_model_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
P_new_wall_model_T ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetModelRtp ( S , ( real_T *
) & new_wall_model_rtDefaultP ) ; _ssSetConstBlockIO ( S , &
new_wall_model_rtInvariant ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ; ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) -> P_1 = rtMinusInf ; ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) -> P_53 = rtInf ; ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) -> P_85 = rtInf ; ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) -> P_98 = rtInf ; ( (
P_new_wall_model_T * ) ssGetModelRtp ( S ) ) -> P_176 = rtMinusInf ; } static
void mdlInitializeSampleTimes ( SimStruct * S ) { { SimStruct * childS ;
SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S , 0 ) ; callSysFcns
= ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; } slAccRegPrmChangeFcn ( S , mdlOutputsTID5 ) ; }
static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
