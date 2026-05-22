/*
 * comms_scr.h — SCR metric transmission
 *
 * Thin extension to the L4 comms layer (comms.h from tapestry-csm-sim).
 * Adds one function that serializes the SCR state snapshot and sends it
 * to the orchestrator as a SIM_MSG_SCR_METRIC datagram.
 */

#ifndef TAPESTRY_COMMS_SCR_H
#define TAPESTRY_COMMS_SCR_H

#include <stdint.h>
#include "comms.h"           /* comms_t — resolves to csm-sim comms.h */
#include <tapestry/scr.h>    /* scr_state_t                            */
#include "sim_protocol.h"    /* SIM_LOOPBACK_ADDR                      */
#include <tapestry/wire.h>   /* tapestry_scr_metric_frame_t, TAPESTRY_MSG_SCR_METRIC */

/*
 * comms_send_scr_metric — Serialize scr_state and election_count as a
 * TAPESTRY_MSG_SCR_METRIC datagram and send to the orchestrator.
 */
void comms_send_scr_metric(const comms_t *c,
                           const scr_state_t *scr,
                           uint32_t election_count);

#endif /* TAPESTRY_COMMS_SCR_H */
