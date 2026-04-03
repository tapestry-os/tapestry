/*
 * tapestry/csm.h — Tapestry Collective State Manager public API
 *
 * This is the stable framework boundary.  Application code, simulation
 * harnesses, and platform adaptation layers include this header.
 *
 * The internal subsys/csm/ headers are implementation details and must
 * not be included directly by consumers.
 *
 * No OS-specific or Zephyr types appear anywhere in this interface.
 * The CSM is portable to any platform that provides a C99 toolchain
 * and a libm (for sqrtf in state.h).  Platform adaptation is limited
 * to the files that call wm_init / wm_tick (the main loop) and the
 * transport layer (gossip send/receive) — not the CSM logic itself.
 */

#ifndef TAPESTRY_CSM_H
#define TAPESTRY_CSM_H

#include "../../subsys/csm/state.h"
#include "../../subsys/csm/world_model.h"

#endif /* TAPESTRY_CSM_H */
