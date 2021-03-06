#ifndef __SCT_FSM_H__
#define __SCT_FSM_H__

/* Generated by fzmparser version 2.2 --- DO NOT EDIT! */

#include "sct_user.h"

extern void sct_fsm_init (void);

/* macros for defining the mapping between IRQ and events */
#define SCT_IRQ_EVENT_signal1_no_signal (0)
#define SCT_IRQ_EVENT_signal1_width (1)
#define SCT_IRQ_EVENT_signal2_no_signal (2)
#define SCT_IRQ_EVENT_signal2_width (3)

/* Input assignments */
#define SCT_INPUT_signal1_input (0)
#define SCT_INPUT_signal2_input (1)

/* Capture registers */
#define SCT_CAPTURE_cap_signal1_width LPC_SCT->CAP[4].L
#define SCT_CAPTURE_cap_signal2_width LPC_SCT->CAP[4].H

/* Match register reload macro definitions */
#define reload_match_no_input(value) { LPC_SCT->MATCHREL[2].L = value; LPC_SCT->MATCHREL[2].L = value; }

#endif
