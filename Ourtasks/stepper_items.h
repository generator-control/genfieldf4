/******************************************************************************
* File Name          : stepper_items.h
* Date First Issued  : 07/31/2020
* Description        : Stepper motor associated items
*******************************************************************************/
/*
09/10/2020 realfaux branch 


08/29/2020 fauxencoder

*/


#ifndef __STEPPER_ITEMS
#define __STEPPER_ITEMS

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stepper_items.h"
#include "common_can.h"
#include "CanTask.h"
#include "stepper_switches.h"

/* Port and pin numbers for stepper controller. */
#define PU_port  GPIOA      // Pulse
#define PU_pin   GPIO_PIN_5 // Pulse
#define DR_port  GPIOB      // Direction
#define DR_pin   GPIO_PIN_0 // Direction
#define EN_port  GPIOB      // Enable
#define EN_pin   GPIO_PIN_1 // Enable
#define LMIN_port  GPIOE       // Limit switch inner
#define LMIN_pin   GPIO_PIN_5  // Limit switch inner
#define LMOUT_port GPIOE       // Limit switch outside
#define LMOUT_pin  GPIO_PIN_10 // Limit switch outside

/* CAN msg: cid_drum_tst_stepcmd: payload[0] bit definitions. */
/* CAN msg: cid_drum_tst_stepcmd: payload[0] bit definitions. */
#define DRBIT 0x01 // (1) Bit mask Direction output pin: 0 = low; 1 = high
#define ENBIT 0x02 // (2) Bit mask Enable output pin: 0 = low; 1 = high
#define LMBIT 0x04 // (3) Bit mask Limit switch simulation
#define IXBIT 0x08 // (4) Bit mask Indexing command
#define ZTBIT 0x10 // (5) Bit mask PB State: Zero Tension
#define ZOBIT 0x20 // (6) Bit mask PB State: Zero Odometer
#define ARBIT 0x40 // (7) Bit mask PB State: ARM
#define PRBIT 0x80 // (8) Bit Mask PB State: PREP
/* Notes of above bit usage--
(1) CP PB processed: Zero Odometer TOGGLES direction minus sign on LCD
(2) CP SAFE/ACTIVE: Bit sets when in CP goes into ARM state
(3) CP PB: Zero Tension PB state simulates limit switch
(4) CP PB: ARM PB state simulates CP begin indexing command
(5) CP PB state: Zero Tension (CP toggles direction)
(6) CP PB state: Zero Odometer
(7) CP PB state: ARM
(8) CP PB state: Prep (CP toggles freeze of CL setting)
*/

// Super-state definitions. Lower nibble reserved for sub-states
#define LW_OFF    0 * 16
#define LW_INDEX  1 * 16
#define LW_MOVE   2 * 16
#define LW_SWEEP  3 * 16
#define LW_TRACK  4 * 16 
#define LW_LOS    5 * 16 

#define NUMCANMSGSSTEPPER 1  // Number of CAN msgs stepper sends

struct STEPPERDBGBUF
{
   uint32_t intcntr;    // interrupt counter
   int32_t  dbg1;       // Debug 1
   int32_t  dbg2;       // Debug 2
   int32_t  dbg3;       // Debug 3
   uint32_t tim5cnt;    // Encoder count
};

/* Parameters stepper instance (LC = Local Copy) */
struct STEPPERLC
{
   float  clfactor;   // Constant to compute oc duration at CL = 100.0
   uint32_t cltimemax;  // Max timer count for shutdown
   int32_t  Lplus;      //
   int32_t  Lminus;     //
   uint32_t hbct;       // Number of ticks between hb msgs
   int32_t  Ka;      // reversal rate
   int32_t  Nr;      // ratio of reversal rate to sweep rate      
   int32_t  Ks;      // sweep rate

   // stepper function sends: others receive the following CAN msgs
   uint32_t cid_hb_stepper;        // CANID_HB_STEPPER: U8_U32','STEPPER: U8: Status, U32: stepper position accum
};

union PAYFLT
{
   float f;
   uint8_t u8[4];
   uint16_t u16[4];
   uint32_t u32;
   int32_t  s32;
   int16_t  s16[2];
}pf;

struct STEPPERSTUFF
{
   struct   STEPPERLC lc; // Parameters for stepper
   struct   CANTXQMSG canmsg[NUMCANMSGSSTEPPER]; // CAN msgs sent
   union    PAYFLT   pf; // For extracting float from payload
   union    PAYFLT   posaccum;  // Stepper position accumulator
   union    PAYFLT   velaccum;  // Stepper velocity accumulator
   int32_t  Lplus32;    // 32-bit extended Lplus
   int32_t  Lminus32;   // 32-bit extended Lminus
   float    speedcmdf;  // Speed command (float)
   float    focdur;     // Temp for computer inverse of CL position
   float    clpos;      // CL position extracted from CAN msg
   uint32_t ledctr1;    // Counter for throttling green LED
   uint32_t ledctr2;    // Counter for throttling orangeLED
   uint32_t ledbit1;    // Bit for toggling green led
   uint32_t ledbit2;    // Bit for toggling orange led
   uint32_t cltimectr;  // Counter for loss of CL msgs
   uint32_t speedcmdi;   // Commanded speed (integer)
   uint32_t ocinc;      // OC register increment for CL faux encoder  
   uint32_t ocidx;      // OC register increment for indexing
   uint32_t hbctr;      // Count ticks for sending heartbeat CAN msg
   uint32_t drflag;     // BSRR pin set/reset bit position: direction
   uint32_t enflag;     // BSRR pin set/reset bit position: enable
   uint32_t iobits;     // Bits from CL CAN msg positioned for PB0
   int16_t  posaccum_prev;  // Previous posaccum
   uint8_t  stepperstatus;  // Reserved for CAN msg
   uint8_t  pay0;       // canmsg.cd.uc[0] saved
   uint8_t  drbit;      // Drum direction bit (0, forward|1, reverse)
   uint8_t  drbit_prev; // Previous Direction bit

   uint8_t  lw_state;      // level-wind states. defined above
   uint8_t  ocicbit;      //
   uint8_t  ocicbit_prev; //

   struct STEPPERSWCONTACT ctk[6]; // Measured switch contact open/close posaccum
   struct EXTISWITCHSTATUS sw[6]; // Limit & overrun switches
   uint16_t swbits;     // Port E switch bits (10:15)

   // debug and characterization, potentially removable for operational code
   uint32_t dtwentry;   // DTW timer upon ISR entry
   int32_t dtwdiff;    // DTW timer minus entry upon ISR exit
   int32_t dtwmax;     // DTW difference max
   int32_t dtwmin;     // DTW difference min
   uint32_t intcntr;    // interrupt counter

   struct STEPPERDBGBUF*  pdbgbegin;
   struct STEPPERDBGBUF*  pdbgadd;
   struct STEPPERDBGBUF*  pdbgtake;
   struct STEPPERDBGBUF*  pdbgend;

};


/* *************************************************************************/
 void stepper_items_init(void);
 /* @brief   : Initialization
 * *************************************************************************/
  void stepper_items_clupdate(struct CANRCVBUF* pcan);
/* @param   : pcan = pointer to CAN msg struct
 * @brief   : Initialization of channel increment
 * *************************************************************************/
  void stepper_items_timeout(void);
/* @brief   : Check for loss of CL CAN msgs
 * *************************************************************************/
  void stepper_items_CANsend(void);
/* @brief   : Send CAN msgs for stepper
 * *************************************************************************/
  /* *************************************************************************/
 void stepper_items_index_init(void);
 /* @brief   : Initialization for indexing
 * *************************************************************************/
 struct STEPPERDBGBUF* stepper_items_getdbg(void);
/* @brief   : Get pointer to debug buffer
 * @return  : NULL = no new data; otherwise ptr struct with data
 * *************************************************************************/

 extern struct STEPPERSTUFF stepperstuff;

#endif
