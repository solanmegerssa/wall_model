#include "__cf_new_wall_model.h"
#ifndef RTW_HEADER_new_wall_model_acc_h_
#define RTW_HEADER_new_wall_model_acc_h_
#include <stddef.h>
#include <float.h>
#include <string.h>
#ifndef new_wall_model_acc_COMMON_INCLUDES_
#define new_wall_model_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#endif
#include "new_wall_model_acc_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_look.h"
#include "rt_look1d.h"
#include "rt_defines.h"
typedef struct { creal_T B_10_16_0 ; real_T B_31_0_0 [ 21 ] ; real_T B_31_1_0
; real_T B_31_2_0 ; real_T B_31_6_0 ; real_T B_31_9_0 ; real_T B_31_10_0 ;
real_T B_31_12_0 ; real_T B_31_14_0 ; real_T B_31_16_0 ; real_T B_31_22_0 ;
real_T B_31_23_0 ; real_T B_31_30_0 ; real_T B_31_31_0 ; real_T B_31_32_0 ;
real_T B_31_33_0 ; real_T B_31_39_0 ; real_T B_31_40_0 ; real_T B_31_47_0 ;
real_T B_31_49_0 ; real_T B_31_50_0 ; real_T B_31_52_0 ; real_T B_31_53_0 ;
real_T B_31_54_0 ; real_T B_31_55_0 ; real_T B_31_56_0 ; real_T B_31_57_0 ;
real_T B_31_58_0 ; real_T B_31_59_0 [ 33 ] ; real_T B_31_59_1 [ 22 ] ; real_T
B_31_62_0 ; real_T B_31_63_0 ; real_T B_31_64_0 ; real_T B_31_65_0 ; real_T
B_31_66_0 ; real_T B_31_74_0 ; real_T B_31_82_0 ; real_T B_31_83_0 ; real_T
B_31_84_0 ; real_T B_31_102_0 ; real_T B_31_103_0 ; real_T B_31_104_0 ;
real_T B_31_105_0 ; real_T B_31_106_0 ; real_T B_31_108_0 ; real_T B_31_111_0
[ 4 ] ; real_T B_31_119_0 ; real_T B_31_123_0 ; real_T B_31_127_0 ; real_T
B_31_134_0 ; real_T B_31_138_0 ; real_T B_31_148_0 ; real_T B_31_149_0 ;
real_T B_31_178_0 ; real_T B_31_185_0 ; real_T B_31_204_0 ; real_T B_31_208_0
; real_T B_31_212_0 ; real_T B_31_216_0 ; real_T B_31_236_0 ; real_T
B_31_238_0 ; real_T B_31_240_0 ; real_T B_31_241_0 ; real_T B_31_243_0 ;
real_T B_31_244_0 ; real_T B_31_246_0 ; real_T B_31_247_0 ; real_T B_31_248_0
; real_T B_31_249_0 ; real_T B_31_250_0 ; real_T B_31_255_0 ; real_T
B_31_257_0 ; real_T B_31_258_0 ; real_T B_31_259_0 ; real_T B_31_260_0 ;
real_T B_31_261_0 ; real_T B_31_262_0 ; real_T B_31_263_0 ; real_T B_31_264_0
; real_T B_31_265_0 ; real_T B_31_266_0 ; real_T B_31_267_0 ; real_T
B_31_268_0 ; real_T B_31_269_0 ; real_T B_31_270_0 [ 2 ] ; real_T B_31_273_0
; real_T B_31_275_0 ; real_T B_31_279_0 ; real_T B_31_281_0 ; real_T
B_31_288_0 [ 4 ] ; real_T B_31_300_0 ; real_T B_31_304_0 ; real_T B_31_322_0
; real_T B_31_323_0 ; real_T B_31_325_0 [ 3 ] ; real_T B_31_327_0 ; real_T
B_31_330_0 ; real_T B_31_332_0 ; real_T B_31_334_0 ; real_T B_31_335_0 ;
real_T B_31_338_0 ; real_T B_31_340_0 ; real_T B_31_342_0 ; real_T B_31_345_0
; real_T B_31_348_0 ; real_T B_31_350_0 ; real_T B_31_352_0 ; real_T
B_31_353_0 ; real_T B_31_356_0 ; real_T B_31_358_0 ; real_T B_31_360_0 ;
real_T B_31_370_0 ; real_T B_31_371_0 ; real_T B_31_372_0 ; real_T B_31_373_0
; real_T B_31_376_0 ; real_T B_31_378_0 ; real_T B_31_380_0 ; real_T
B_31_381_0 ; real_T B_31_384_0 ; real_T B_31_386_0 ; real_T B_31_388_0 ;
real_T B_31_391_0 ; real_T B_31_394_0 ; real_T B_31_396_0 ; real_T B_31_398_0
; real_T B_31_399_0 ; real_T B_31_402_0 ; real_T B_31_404_0 ; real_T
B_31_406_0 ; real_T B_31_416_0 ; real_T B_31_417_0 ; real_T B_31_418_0 ;
real_T B_31_421_0 ; real_T B_31_423_0 ; real_T B_31_425_0 ; real_T B_31_426_0
; real_T B_31_429_0 ; real_T B_31_431_0 ; real_T B_31_433_0 ; real_T
B_31_436_0 ; real_T B_31_439_0 ; real_T B_31_441_0 ; real_T B_31_443_0 ;
real_T B_31_444_0 ; real_T B_31_447_0 ; real_T B_31_449_0 ; real_T B_31_451_0
; real_T B_31_461_0 ; real_T B_31_462_0 ; real_T B_31_463_0 ; real_T
B_31_466_0 ; real_T B_31_468_0 ; real_T B_31_470_0 ; real_T B_31_471_0 ;
real_T B_31_474_0 ; real_T B_31_476_0 ; real_T B_31_478_0 ; real_T B_31_481_0
; real_T B_31_484_0 ; real_T B_31_486_0 ; real_T B_31_488_0 ; real_T
B_31_489_0 ; real_T B_31_492_0 ; real_T B_31_494_0 ; real_T B_31_496_0 ;
real_T B_31_506_0 ; real_T B_31_507_0 ; real_T B_31_510_0 ; real_T B_31_512_0
; real_T B_31_514_0 ; real_T B_31_516_0 ; real_T B_31_518_0 ; real_T
B_31_520_0 ; real_T B_31_522_0 ; real_T B_31_524_0 ; real_T B_31_526_0 ;
real_T B_31_528_0 ; real_T B_31_530_0 ; real_T B_31_532_0 ; real_T B_31_534_0
; real_T B_31_536_0 ; real_T B_31_538_0 ; real_T B_31_540_0 ; real_T
B_31_545_0 ; real_T B_14_0_0 ; real_T B_14_2_0 ; real_T B_14_3_0 ; real_T
B_14_4_0 [ 2 ] ; real_T B_14_7_0 ; real_T B_14_8_0 ; real_T B_14_9_0 ; real_T
B_14_11_0 ; real_T B_14_13_0 ; real_T B_14_14_0 ; real_T B_14_15_0 ; real_T
B_14_16_0 ; real_T B_14_17_0 ; real_T B_14_19_0 ; real_T B_14_21_0 ; real_T
B_13_0_1 ; real_T B_12_0_0 ; real_T B_12_1_0 ; real_T B_12_2_0 ; real_T
B_12_3_0 ; real_T B_12_4_0 ; real_T B_12_5_0 ; real_T B_12_6_0 ; real_T
B_12_7_0 ; real_T B_12_8_0 ; real_T B_12_9_0 ; real_T B_12_10_0 ; real_T
B_12_11_0 ; real_T B_12_12_0 ; real_T B_12_13_0 ; real_T B_12_14_0 ; real_T
B_12_15_0 ; real_T B_12_16_0 ; real_T B_12_17_0 ; real_T B_12_18_0 ; real_T
B_12_19_0 ; real_T B_12_20_0 ; real_T B_12_21_0 ; real_T B_12_22_0 ; real_T
B_12_23_0 ; real_T B_12_24_0 ; real_T B_12_25_0 ; real_T B_12_26_0 ; real_T
B_12_27_0 ; real_T B_12_29_0 ; real_T B_12_30_0 ; real_T B_12_31_0 ; real_T
B_12_33_0 ; real_T B_12_34_0 ; real_T B_12_36_0 ; real_T B_12_37_0 ; real_T
B_12_38_0 ; real_T B_12_40_0 ; real_T B_12_42_0 ; real_T B_12_44_0 ; real_T
B_12_45_0 ; real_T B_12_46_0 ; real_T B_12_47_0 ; real_T B_12_48_0 ; real_T
B_12_49_0 ; real_T B_12_50_0 ; real_T B_12_51_0 ; real_T B_12_52_0 ; real_T
B_12_53_0 ; real_T B_12_54_0 ; real_T B_12_56_0 ; real_T B_12_57_0 ; real_T
B_12_58_0 ; real_T B_12_59_0 ; real_T B_12_60_0 ; real_T B_12_61_0 ; real_T
B_12_62_0 ; real_T B_12_63_0 ; real_T B_12_64_0 ; real_T B_12_65_0 ; real_T
B_12_66_0 ; real_T B_12_67_0 ; real_T B_12_68_0 ; real_T B_12_69_0 ; real_T
B_12_74_0 ; real_T B_12_75_0 [ 2 ] ; real_T B_12_78_0 ; real_T B_12_79_0 ;
real_T B_12_80_0 ; real_T B_12_82_0 ; real_T B_12_84_0 ; real_T B_12_85_0 ;
real_T B_12_86_0 ; real_T B_12_87_0 ; real_T B_12_88_0 ; real_T B_12_90_0 ;
real_T B_12_94_0 ; real_T B_12_96_0 ; real_T B_11_0_0 ; real_T B_11_1_0 ;
real_T B_10_0_0 ; real_T B_10_1_0 ; real_T B_10_3_0 ; real_T B_10_5_0 ;
real_T B_10_7_0 ; real_T B_10_8_0 ; real_T B_10_9_0 ; real_T B_10_11_0 ;
real_T B_10_13_0 ; real_T B_10_15_0 ; real_T B_10_19_0 ; real_T B_10_20_0 ;
real_T B_10_21_0 ; real_T B_10_22_0 ; real_T B_10_23_0 ; real_T B_10_24_0 ;
real_T B_10_25_0 ; real_T B_10_26_0 ; real_T B_10_27_0 ; real_T B_10_28_0 ;
real_T B_9_0_0 ; real_T B_9_1_0 ; real_T B_8_0_0 ; real_T B_8_1_0 ; real_T
B_7_0_0 ; real_T B_7_1_0 ; real_T B_7_2_0 ; real_T B_7_3_0 ; real_T B_7_4_0 ;
real_T B_7_5_0 ; real_T B_7_6_0 ; real_T B_7_7_0 ; real_T B_7_8_0 ; real_T
B_7_9_0 ; real_T B_7_10_0 ; real_T B_7_11_0 ; real_T B_7_12_0 ; real_T
B_7_13_0 ; real_T B_7_14_0 ; real_T B_7_15_0 ; real_T B_7_16_0 ; real_T
B_7_17_0 ; real_T B_7_18_0 ; real_T B_7_19_0 ; real_T B_7_20_0 ; real_T
B_7_21_0 ; real_T B_7_22_0 ; real_T B_7_23_0 ; real_T B_7_24_0 ; real_T
B_7_25_0 ; real_T B_7_26_0 [ 2 ] ; real_T B_7_29_0 ; real_T B_7_30_0 ; real_T
B_7_31_0 ; real_T B_7_33_0 ; real_T B_7_35_0 ; real_T B_7_36_0 ; real_T
B_7_37_0 ; real_T B_7_38_0 ; real_T B_7_39_0 ; real_T B_7_42_0 [ 2 ] ;
boolean_T B_31_109_0 ; boolean_T B_12_28_0 ; char_T pad_B_12_28_0 [ 6 ] ; }
B_new_wall_model_T ; typedef struct { real_T Currentfilter_states ; real_T
inti_DSTATE ; real_T DiscreteTimeIntegrator_DSTATE ; real_T StateSpace_DSTATE
[ 38 ] ; real_T itinit1_PreviousInput ; real_T itinit_PreviousInput ; real_T
Memory2_PreviousInput ; real_T lastSin ; real_T lastCos ; real_T lastSin_i ;
real_T lastCos_o ; real_T lastSin_g ; real_T lastCos_j ; real_T
Memory_PreviousInput ; real_T Memory_PreviousInput_n ; real_T
Memory_PreviousInput_p ; real_T Memory_PreviousInput_f ; real_T
Memory_PreviousInput_e ; real_T Memory_PreviousInput_b ; real_T
Memory_PreviousInput_h ; real_T Memory_PreviousInput_l ; real_T
Memory_PreviousInput_k ; real_T Memory_PreviousInput_j ; real_T
Memory_PreviousInput_ng ; real_T Memory_PreviousInput_g ; real_T
Memory_PreviousInput_a ; real_T Memory_PreviousInput_gn ; real_T
Memory_PreviousInput_i ; real_T Memory_PreviousInput_li ; real_T lastSin_c ;
real_T lastCos_jx ; real_T Initial_FirstOutputTime ; real_T
Memory1_PreviousInput ; real_T Memory_PreviousInput_c ; real_T PrevYA ;
real_T PrevYB ; real_T LastMajorTimeA ; real_T LastMajorTimeB ; real_T
Memory_PreviousInput_cc ; real_T Memory_PreviousInput_cz ; struct { real_T
modelTStart ; } TransportDelay_RWORK ; struct { real_T modelTStart ; }
TransportDelay_RWORK_o ; struct { real_T modelTStart ; }
TransportDelay_RWORK_c ; struct { real_T modelTStart ; }
TransportDelay_RWORK_l ; struct { real_T modelTStart ; }
TransportDelay_RWORK_j ; struct { real_T modelTStart ; }
TransportDelay_RWORK_ov ; struct { real_T modelTStart ; }
TransportDelay_RWORK_d ; struct { real_T modelTStart ; }
TransportDelay_RWORK_i ; struct { real_T modelTStart ; }
TransportDelay_RWORK_b ; struct { real_T modelTStart ; }
TransportDelay_RWORK_oe ; struct { real_T modelTStart ; }
TransportDelay_RWORK_jp ; struct { real_T modelTStart ; }
TransportDelay_RWORK_e ; struct { real_T modelTStart ; }
TransportDelay_RWORK_g ; struct { real_T modelTStart ; }
TransportDelay_RWORK_en ; struct { real_T modelTStart ; }
TransportDelay_RWORK_dm ; struct { real_T modelTStart ; }
TransportDelay_RWORK_h ; struct { real_T modelTStart ; }
VariableTransportDelay_RWORK ; struct { real_T modelTStart ; }
VariableTransportDelay_RWORK_c ; struct { real_T modelTStart ; }
VariableTransportDelay_RWORK_cj ; struct { void * AS ; void * BS ; void * CS
; void * DS ; void * DX_COL ; void * BD_COL ; void * TMP1 ; void * TMP2 ;
void * XTMP ; void * SWITCH_STATUS ; void * SWITCH_STATUS_INIT ; void *
SW_CHG ; void * G_STATE ; void * USWLAST ; void * XKM12 ; void * XKP12 ; void
* XLAST ; void * ULAST ; void * IDX_SW_CHG ; void * Y_SWITCH ; void *
SWITCH_TYPES ; void * IDX_OUT_SW ; void * SWITCH_TOPO_SAVED_IDX ; void *
SWITCH_MAP ; } StateSpace_PWORK ; void * BatteryStatus_PWORK [ 4 ] ; void *
FromWs_PWORK [ 3 ] ; void * PVPanel_PWORK [ 3 ] ; void * FromWs_PWORK_h [ 3 ]
; void * Events_PWORK [ 5 ] ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_a ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_o ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_an ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_og ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_i ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_c ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_l ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_ax ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_m ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_is ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_j ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_jp ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_e ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_b ; struct { void * TUbufferPtrs [ 2 ] ; }
TransportDelay_PWORK_by ; void * POWERS_PWORK [ 6 ] ; void * GRID_PWORK [ 4 ]
; struct { void * TUbufferPtrs [ 3 ] ; } VariableTransportDelay_PWORK ;
struct { void * TUbufferPtrs [ 3 ] ; } VariableTransportDelay_PWORK_b ;
struct { void * TUbufferPtrs [ 3 ] ; } VariableTransportDelay_PWORK_g ;
int32_T systemEnable ; int32_T systemEnable_g ; int32_T systemEnable_d ;
int32_T TmpAtomicSubsysAtSwitchInport1_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_d ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_m ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_c ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_e ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_p ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_f ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_fs ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_fm ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_ms ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_ca ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_fv ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_k ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_l ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_o ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_j ; int32_T
PWMController2_sysIdxToRun ; int32_T systemEnable_o ; int32_T
MPPTController_sysIdxToRun ; int32_T InverterController_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_a ; int32_T
AutomaticGainControl_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_i ; int32_T
TmpAtomicSubsysAtSwitchInport1_sysIdxToRun_cq ; int32_T
BatteryController_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitch2Inport3_sysIdxToRun ; int32_T
TmpAtomicSubsysAtSwitch2Inport3_sysIdxToRun_k ; int32_T
TmpAtomicSubsysAtMultiportSwitch1Inport3_sysIdxToRun ; int32_T
TmpAtomicSubsysAtMultiportSwitch1Inport2_sysIdxToRun ; int32_T
TmpAtomicSubsysAtMultiportSwitch1Inport3_sysIdxToRun_l ; int32_T
TmpAtomicSubsysAtMultiportSwitch1Inport4_sysIdxToRun ; int32_T
TmpAtomicSubsysAtMultiportSwitch1Inport5_sysIdxToRun ; int_T StateSpace_IWORK
[ 11 ] ; int_T FromWs_IWORK ; int_T FromWs_IWORK_g ; struct { int_T Tail ;
int_T Head ; int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK ; struct { int_T Tail ; int_T Head ; int_T Last ; int_T
CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_f ; struct {
int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_d ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_f1 ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_m ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_o ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_j ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_dz ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_p ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_c ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_e ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_n ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_g ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay_IWORK_cn ;
struct { int_T Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T
MaxNewBufSize ; } TransportDelay_IWORK_i ; struct { int_T Tail ; int_T Head ;
int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
TransportDelay_IWORK_ex ; int_T Integrator_IWORK ; struct { int_T Tail ;
int_T Head ; int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
VariableTransportDelay_IWORK ; struct { int_T Tail ; int_T Head ; int_T Last
; int_T CircularBufSize ; int_T MaxNewBufSize ; }
VariableTransportDelay_IWORK_o ; struct { int_T Tail ; int_T Head ; int_T
Last ; int_T CircularBufSize ; int_T MaxNewBufSize ; }
VariableTransportDelay_IWORK_p ; int8_T inti_PrevResetState ; int8_T
PWMController2_SubsysRanBC ; int8_T InverterController_SubsysRanBC ; int8_T
AutomaticGainControl_SubsysRanBC ; int8_T BatteryController_SubsysRanBC ;
uint8_T inti_IC_LOADING ; boolean_T PWMController2_MODE ; boolean_T
InverterController_MODE ; boolean_T AutomaticGainControl_MODE ; boolean_T
BatteryController_MODE ; char_T pad_BatteryController_MODE [ 6 ] ; }
DW_new_wall_model_T ; typedef struct { real_T StateSpace_CSTATE ; real_T
TransferFcn8_CSTATE ; real_T TransferFcn1_CSTATE ; real_T TransferFcn3_CSTATE
; real_T TransferFcn_CSTATE ; real_T TransferFcn_CSTATE_p ; real_T
TransferFcn2_CSTATE ; real_T TransferFcn5_CSTATE ; real_T TransferFcn6_CSTATE
; real_T TransferFcn6_CSTATE_a ; real_T Integrator_CSTATE ; real_T
Filter_CSTATE ; real_T TransferFcn5_CSTATE_b ; real_T Integrator_CSTATE_f ;
real_T Filter_CSTATE_f ; real_T integrator_CSTATE ; real_T
integrator_CSTATE_d ; real_T integrator_CSTATE_b ; real_T
integrator_CSTATE_de ; real_T integrator_CSTATE_a ; real_T
integrator_CSTATE_f ; real_T integrator_CSTATE_e ; real_T integrator_CSTATE_g
; real_T integrator_CSTATE_eh ; real_T integrator_CSTATE_p ; real_T
integrator_CSTATE_o ; real_T integrator_CSTATE_m ; real_T integrator_CSTATE_k
; real_T integrator_CSTATE_bu ; real_T integrator_CSTATE_h ; real_T
integrator_CSTATE_k4 ; real_T TransferFcn2_CSTATE_g ; real_T
Integrator_CSTATE_i ; real_T Filter_CSTATE_k ; real_T TransferFcn3_CSTATE_n ;
real_T TransferFcn1_CSTATE_d ; real_T Filter_CSTATE_i ; real_T
Integrator_CSTATE_m ; real_T Integrator_CSTATE_l ; real_T
Integrator_CSTATE_f5 ; real_T VariableTransportDelay_CSTATE ; real_T
integrator_CSTATE_e2 ; real_T TransferFcn_CSTATE_j ; real_T
Integrator_x1_CSTATE ; real_T Integrator_x2_CSTATE ; real_T
VariableTransportDelay_CSTATE_m ; real_T integrator_CSTATE_n ; real_T
VariableTransportDelay_CSTATE_c ; real_T integrator_CSTATE_b4 ; real_T
TransferFcn2_CSTATE_a ; real_T Integrator_CSTATE_j ; real_T Filter_CSTATE_m ;
real_T TransferFcn3_CSTATE_g ; real_T TransferFcn1_CSTATE_h ; real_T
Filter_CSTATE_l ; real_T Integrator_CSTATE_d ; } X_new_wall_model_T ; typedef
struct { real_T StateSpace_CSTATE ; real_T TransferFcn8_CSTATE ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn3_CSTATE ; real_T TransferFcn_CSTATE
; real_T TransferFcn_CSTATE_p ; real_T TransferFcn2_CSTATE ; real_T
TransferFcn5_CSTATE ; real_T TransferFcn6_CSTATE ; real_T
TransferFcn6_CSTATE_a ; real_T Integrator_CSTATE ; real_T Filter_CSTATE ;
real_T TransferFcn5_CSTATE_b ; real_T Integrator_CSTATE_f ; real_T
Filter_CSTATE_f ; real_T integrator_CSTATE ; real_T integrator_CSTATE_d ;
real_T integrator_CSTATE_b ; real_T integrator_CSTATE_de ; real_T
integrator_CSTATE_a ; real_T integrator_CSTATE_f ; real_T integrator_CSTATE_e
; real_T integrator_CSTATE_g ; real_T integrator_CSTATE_eh ; real_T
integrator_CSTATE_p ; real_T integrator_CSTATE_o ; real_T integrator_CSTATE_m
; real_T integrator_CSTATE_k ; real_T integrator_CSTATE_bu ; real_T
integrator_CSTATE_h ; real_T integrator_CSTATE_k4 ; real_T
TransferFcn2_CSTATE_g ; real_T Integrator_CSTATE_i ; real_T Filter_CSTATE_k ;
real_T TransferFcn3_CSTATE_n ; real_T TransferFcn1_CSTATE_d ; real_T
Filter_CSTATE_i ; real_T Integrator_CSTATE_m ; real_T Integrator_CSTATE_l ;
real_T Integrator_CSTATE_f5 ; real_T VariableTransportDelay_CSTATE ; real_T
integrator_CSTATE_e2 ; real_T TransferFcn_CSTATE_j ; real_T
Integrator_x1_CSTATE ; real_T Integrator_x2_CSTATE ; real_T
VariableTransportDelay_CSTATE_m ; real_T integrator_CSTATE_n ; real_T
VariableTransportDelay_CSTATE_c ; real_T integrator_CSTATE_b4 ; real_T
TransferFcn2_CSTATE_a ; real_T Integrator_CSTATE_j ; real_T Filter_CSTATE_m ;
real_T TransferFcn3_CSTATE_g ; real_T TransferFcn1_CSTATE_h ; real_T
Filter_CSTATE_l ; real_T Integrator_CSTATE_d ; } XDot_new_wall_model_T ;
typedef struct { boolean_T StateSpace_CSTATE ; boolean_T TransferFcn8_CSTATE
; boolean_T TransferFcn1_CSTATE ; boolean_T TransferFcn3_CSTATE ; boolean_T
TransferFcn_CSTATE ; boolean_T TransferFcn_CSTATE_p ; boolean_T
TransferFcn2_CSTATE ; boolean_T TransferFcn5_CSTATE ; boolean_T
TransferFcn6_CSTATE ; boolean_T TransferFcn6_CSTATE_a ; boolean_T
Integrator_CSTATE ; boolean_T Filter_CSTATE ; boolean_T TransferFcn5_CSTATE_b
; boolean_T Integrator_CSTATE_f ; boolean_T Filter_CSTATE_f ; boolean_T
integrator_CSTATE ; boolean_T integrator_CSTATE_d ; boolean_T
integrator_CSTATE_b ; boolean_T integrator_CSTATE_de ; boolean_T
integrator_CSTATE_a ; boolean_T integrator_CSTATE_f ; boolean_T
integrator_CSTATE_e ; boolean_T integrator_CSTATE_g ; boolean_T
integrator_CSTATE_eh ; boolean_T integrator_CSTATE_p ; boolean_T
integrator_CSTATE_o ; boolean_T integrator_CSTATE_m ; boolean_T
integrator_CSTATE_k ; boolean_T integrator_CSTATE_bu ; boolean_T
integrator_CSTATE_h ; boolean_T integrator_CSTATE_k4 ; boolean_T
TransferFcn2_CSTATE_g ; boolean_T Integrator_CSTATE_i ; boolean_T
Filter_CSTATE_k ; boolean_T TransferFcn3_CSTATE_n ; boolean_T
TransferFcn1_CSTATE_d ; boolean_T Filter_CSTATE_i ; boolean_T
Integrator_CSTATE_m ; boolean_T Integrator_CSTATE_l ; boolean_T
Integrator_CSTATE_f5 ; boolean_T VariableTransportDelay_CSTATE ; boolean_T
integrator_CSTATE_e2 ; boolean_T TransferFcn_CSTATE_j ; boolean_T
Integrator_x1_CSTATE ; boolean_T Integrator_x2_CSTATE ; boolean_T
VariableTransportDelay_CSTATE_m ; boolean_T integrator_CSTATE_n ; boolean_T
VariableTransportDelay_CSTATE_c ; boolean_T integrator_CSTATE_b4 ; boolean_T
TransferFcn2_CSTATE_a ; boolean_T Integrator_CSTATE_j ; boolean_T
Filter_CSTATE_m ; boolean_T TransferFcn3_CSTATE_g ; boolean_T
TransferFcn1_CSTATE_h ; boolean_T Filter_CSTATE_l ; boolean_T
Integrator_CSTATE_d ; } XDis_new_wall_model_T ; typedef struct { real_T
StateSpace_CSTATE ; real_T TransferFcn8_CSTATE ; real_T TransferFcn1_CSTATE ;
real_T TransferFcn3_CSTATE ; real_T TransferFcn_CSTATE ; real_T
TransferFcn_CSTATE_p ; real_T TransferFcn2_CSTATE ; real_T
TransferFcn5_CSTATE ; real_T TransferFcn6_CSTATE ; real_T
TransferFcn6_CSTATE_a ; real_T Integrator_CSTATE ; real_T Filter_CSTATE ;
real_T TransferFcn5_CSTATE_b ; real_T Integrator_CSTATE_f ; real_T
Filter_CSTATE_f ; real_T integrator_CSTATE ; real_T integrator_CSTATE_d ;
real_T integrator_CSTATE_b ; real_T integrator_CSTATE_de ; real_T
integrator_CSTATE_a ; real_T integrator_CSTATE_f ; real_T integrator_CSTATE_e
; real_T integrator_CSTATE_g ; real_T integrator_CSTATE_eh ; real_T
integrator_CSTATE_p ; real_T integrator_CSTATE_o ; real_T integrator_CSTATE_m
; real_T integrator_CSTATE_k ; real_T integrator_CSTATE_bu ; real_T
integrator_CSTATE_h ; real_T integrator_CSTATE_k4 ; real_T
TransferFcn2_CSTATE_g ; real_T Integrator_CSTATE_i ; real_T Filter_CSTATE_k ;
real_T TransferFcn3_CSTATE_n ; real_T TransferFcn1_CSTATE_d ; real_T
Filter_CSTATE_i ; real_T Integrator_CSTATE_m ; real_T Integrator_CSTATE_l ;
real_T Integrator_CSTATE_f5 ; real_T VariableTransportDelay_CSTATE ; real_T
integrator_CSTATE_e2 ; real_T TransferFcn_CSTATE_j ; real_T
Integrator_x1_CSTATE ; real_T Integrator_x2_CSTATE ; real_T
VariableTransportDelay_CSTATE_m ; real_T integrator_CSTATE_n ; real_T
VariableTransportDelay_CSTATE_c ; real_T integrator_CSTATE_b4 ; real_T
TransferFcn2_CSTATE_a ; real_T Integrator_CSTATE_j ; real_T Filter_CSTATE_m ;
real_T TransferFcn3_CSTATE_g ; real_T TransferFcn1_CSTATE_h ; real_T
Filter_CSTATE_l ; real_T Integrator_CSTATE_d ; }
CStateAbsTol_new_wall_model_T ; typedef struct { ZCSigState
Integrator_Reset_ZCE ; } PrevZCX_new_wall_model_T ; typedef struct { const
real_T B_31_7_0 ; const real_T B_31_28_0 ; const real_T B_31_76_0 ; }
ConstB_new_wall_model_T ;
#define new_wall_model_rtC(S) ((ConstB_new_wall_model_T *) _ssGetConstBlockIO(S))
struct P_new_wall_model_T_ { real_T P_0 ; real_T P_1 ; real_T P_2 ; real_T
P_3 ; real_T P_4 ; real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T
P_9 ; real_T P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T P_14 ;
real_T P_15 ; real_T P_16 ; real_T P_17 ; real_T P_18 ; real_T P_19 ; real_T
P_20 ; real_T P_21 ; real_T P_22 ; real_T P_23 ; real_T P_24 ; real_T P_25 ;
real_T P_26 ; real_T P_27 ; real_T P_28 [ 2 ] ; real_T P_29 ; real_T P_30 ;
real_T P_31 ; real_T P_32 [ 3 ] ; real_T P_33 [ 3 ] ; real_T P_34 ; real_T
P_35 ; real_T P_36 ; real_T P_37 ; real_T P_38 ; real_T P_39 ; real_T P_40 ;
real_T P_41 ; real_T P_42 ; real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T
P_46 ; real_T P_47 ; real_T P_48 ; real_T P_49 ; real_T P_50 ; real_T P_51 ;
real_T P_52 ; real_T P_53 ; real_T P_54 ; real_T P_55 [ 4 ] ; real_T P_56 ;
real_T P_57 ; real_T P_58 ; real_T P_59 ; real_T P_60 ; real_T P_61 ; real_T
P_62 ; real_T P_63 ; real_T P_64 ; real_T P_65 ; real_T P_66 ; real_T P_67 ;
real_T P_68 ; real_T P_69 ; real_T P_70 ; real_T P_71 ; real_T P_72 ; real_T
P_73 ; real_T P_74 ; real_T P_75 ; real_T P_76 ; real_T P_77 ; real_T P_78 ;
real_T P_79 ; real_T P_80 ; real_T P_81 ; real_T P_82 ; real_T P_83 ; real_T
P_84 ; real_T P_85 ; real_T P_86 ; real_T P_87 ; real_T P_88 ; real_T P_89 ;
real_T P_90 ; real_T P_91 ; real_T P_92 ; real_T P_93 ; real_T P_94 ; real_T
P_95 ; real_T P_96 ; real_T P_97 ; real_T P_98 ; real_T P_99 ; real_T P_100 ;
real_T P_101 ; real_T P_102 ; real_T P_103 ; real_T P_104 ; real_T P_105 ;
real_T P_106 ; real_T P_107 ; real_T P_108 ; real_T P_109 ; real_T P_110 ;
real_T P_111 ; real_T P_112 ; real_T P_113 ; real_T P_114 ; real_T P_115 ;
real_T P_116 [ 2 ] ; real_T P_117 ; real_T P_118 ; real_T P_119 ; real_T
P_120 [ 3 ] ; real_T P_121 [ 3 ] ; real_T P_122 ; real_T P_123 ; real_T P_124
; real_T P_125 ; real_T P_126 ; real_T P_127 [ 2 ] ; real_T P_128 ; real_T
P_129 [ 4 ] ; real_T P_130 ; real_T P_131 ; real_T P_132 [ 2 ] ; real_T P_133
; real_T P_134 ; real_T P_135 ; real_T P_136 [ 3 ] ; real_T P_137 [ 3 ] ;
real_T P_138 ; real_T P_139 ; real_T P_140 ; real_T P_141 ; real_T P_142 ;
real_T P_143 ; real_T P_144 ; real_T P_145 ; real_T P_146 ; real_T P_147 ;
real_T P_148 ; real_T P_149 ; real_T P_150 ; real_T P_151 ; real_T P_152 ;
real_T P_153 ; real_T P_154 ; real_T P_155 ; real_T P_156 ; real_T P_157 ;
real_T P_158 ; real_T P_159 ; real_T P_160 ; real_T P_161 ; real_T P_162 ;
real_T P_163 ; real_T P_164 [ 21 ] ; real_T P_165 ; real_T P_166 ; real_T
P_167 ; real_T P_168 ; real_T P_169 ; real_T P_170 [ 2 ] ; real_T P_171 ;
real_T P_172 ; real_T P_173 ; real_T P_174 ; real_T P_175 ; real_T P_176 ;
real_T P_177 ; real_T P_178 ; real_T P_179 ; real_T P_180 ; real_T P_181 ;
real_T P_182 ; real_T P_183 ; real_T P_184 ; real_T P_185 ; real_T P_186 ;
real_T P_187 ; real_T P_188 ; real_T P_189 ; real_T P_190 ; real_T P_191 ;
real_T P_192 ; real_T P_193 ; real_T P_194 [ 11 ] ; real_T P_195 [ 11 ] ;
real_T P_196 ; real_T P_197 ; real_T P_198 ; real_T P_199 ; real_T P_200 ;
real_T P_201 ; real_T P_202 ; real_T P_203 ; real_T P_204 ; real_T P_205 ;
real_T P_206 ; real_T P_207 ; real_T P_208 ; real_T P_209 ; real_T P_210 ;
real_T P_211 ; real_T P_212 ; real_T P_213 ; real_T P_214 ; real_T P_215 ;
real_T P_216 ; real_T P_217 ; real_T P_218 ; real_T P_219 ; real_T P_220 ;
real_T P_221 ; real_T P_222 [ 2 ] ; real_T P_223 [ 1444 ] ; real_T P_224 [ 2
] ; real_T P_225 [ 1140 ] ; real_T P_226 [ 2 ] ; real_T P_227 [ 1254 ] ;
real_T P_228 [ 2 ] ; real_T P_229 [ 990 ] ; real_T P_230 [ 2 ] ; real_T P_231
[ 38 ] ; real_T P_232 ; real_T P_233 ; real_T P_234 ; real_T P_235 ; real_T
P_236 ; real_T P_237 ; real_T P_238 ; real_T P_239 ; real_T P_240 ; real_T
P_241 ; real_T P_242 ; real_T P_243 ; real_T P_244 ; real_T P_245 ; real_T
P_246 ; real_T P_247 ; real_T P_248 ; real_T P_249 ; real_T P_250 ; real_T
P_251 ; real_T P_252 ; real_T P_253 ; real_T P_254 ; real_T P_255 ; real_T
P_256 ; real_T P_257 ; real_T P_258 ; real_T P_259 ; real_T P_260 ; real_T
P_261 ; real_T P_262 ; real_T P_263 ; real_T P_264 ; real_T P_265 ; real_T
P_266 ; real_T P_267 ; real_T P_268 ; real_T P_269 ; real_T P_270 ; real_T
P_271 ; real_T P_272 ; real_T P_273 ; real_T P_274 ; real_T P_275 ; real_T
P_276 ; real_T P_277 ; real_T P_278 ; real_T P_279 ; real_T P_280 ; real_T
P_281 ; real_T P_282 ; real_T P_283 ; real_T P_284 ; real_T P_285 ; real_T
P_286 ; real_T P_287 ; real_T P_288 ; real_T P_289 ; real_T P_290 ; real_T
P_291 [ 2 ] ; real_T P_292 ; real_T P_293 ; real_T P_294 ; real_T P_295 [ 3 ]
; real_T P_296 [ 3 ] ; real_T P_297 ; real_T P_298 ; real_T P_299 ; real_T
P_300 ; real_T P_301 ; real_T P_302 ; real_T P_303 ; real_T P_304 ; real_T
P_305 ; real_T P_306 ; real_T P_307 ; real_T P_308 ; real_T P_309 ; real_T
P_310 ; real_T P_311 ; real_T P_312 ; real_T P_313 ; real_T P_314 ; real_T
P_315 ; real_T P_316 ; real_T P_317 ; real_T P_318 ; real_T P_319 ; real_T
P_320 ; real_T P_321 ; real_T P_322 ; real_T P_323 ; real_T P_324 ; real_T
P_325 ; real_T P_326 ; real_T P_327 ; real_T P_328 ; real_T P_329 ; real_T
P_330 ; real_T P_331 ; real_T P_332 ; real_T P_333 ; real_T P_334 ; real_T
P_335 ; real_T P_336 ; real_T P_337 ; real_T P_338 ; real_T P_339 ; real_T
P_340 ; real_T P_341 ; real_T P_342 ; real_T P_343 ; real_T P_344 ; real_T
P_345 ; real_T P_346 ; real_T P_347 ; real_T P_348 ; real_T P_349 ; real_T
P_350 ; real_T P_351 ; real_T P_352 ; real_T P_353 ; real_T P_354 ; real_T
P_355 ; real_T P_356 ; real_T P_357 ; real_T P_358 ; real_T P_359 ; real_T
P_360 ; real_T P_361 ; real_T P_362 ; real_T P_363 ; real_T P_364 ; real_T
P_365 ; real_T P_366 ; real_T P_367 ; real_T P_368 ; real_T P_369 ; real_T
P_370 ; real_T P_371 ; real_T P_372 ; real_T P_373 ; real_T P_374 ; real_T
P_375 ; real_T P_376 ; real_T P_377 ; real_T P_378 ; real_T P_379 ; real_T
P_380 ; real_T P_381 ; real_T P_382 ; real_T P_383 ; real_T P_384 ; real_T
P_385 ; real_T P_386 ; real_T P_387 ; real_T P_388 ; real_T P_389 ; real_T
P_390 ; real_T P_391 ; real_T P_392 ; real_T P_393 ; real_T P_394 ; real_T
P_395 ; real_T P_396 ; real_T P_397 ; real_T P_398 ; real_T P_399 ; real_T
P_400 ; real_T P_401 ; real_T P_402 ; real_T P_403 ; real_T P_404 ; real_T
P_405 ; real_T P_406 ; real_T P_407 ; real_T P_408 ; real_T P_409 ; real_T
P_410 ; real_T P_411 ; real_T P_412 ; real_T P_413 ; real_T P_414 ; real_T
P_415 ; real_T P_416 ; real_T P_417 ; real_T P_418 ; real_T P_419 ; real_T
P_420 ; real_T P_421 ; real_T P_422 ; real_T P_423 ; real_T P_424 ; real_T
P_425 ; real_T P_426 ; real_T P_427 ; real_T P_428 ; real_T P_429 ; real_T
P_430 ; real_T P_431 ; real_T P_432 ; real_T P_433 ; real_T P_434 ; real_T
P_435 ; real_T P_436 ; real_T P_437 ; real_T P_438 ; real_T P_439 ; real_T
P_440 ; real_T P_441 ; real_T P_442 ; real_T P_443 ; real_T P_444 ; real_T
P_445 ; real_T P_446 ; real_T P_447 ; real_T P_448 ; real_T P_449 ; real_T
P_450 ; real_T P_451 ; real_T P_452 ; real_T P_453 ; real_T P_454 ; real_T
P_455 ; real_T P_456 ; real_T P_457 ; real_T P_458 ; real_T P_459 ; real_T
P_460 ; real_T P_461 ; real_T P_462 ; real_T P_463 ; real_T P_464 ; real_T
P_465 ; } ; extern P_new_wall_model_T new_wall_model_rtDefaultP ; extern
const ConstB_new_wall_model_T new_wall_model_rtInvariant ;
#endif
