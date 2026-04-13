/*
 * tapestry/scr.h — Public API boundary for L5 Swarm Coordination Runtime.
 *
 * This is the only header consumers of L5 should include:
 *
 *   #include <tapestry/scr.h>
 *
 * It transitively includes <tapestry/csm.h> (L4), so including this
 * header gives access to the complete L4–L5 public surface.
 *
 * Design contract:
 *   - No OS-specific types cross this boundary.
 *   - Compiles cleanly against any C99 toolchain with libm.
 *   - The adaptation layer (platform main loop, transport) lives in the
 *     consuming application; this header and the files it includes are
 *     platform-independent.
 */

#ifndef TAPESTRY_SCR_PUBLIC_H
#define TAPESTRY_SCR_PUBLIC_H

#include "csm.h"                      /* L4 public boundary (transitively)   */
#include "../../subsys/scr/scr.h"     /* L5 types and API                    */

#endif /* TAPESTRY_SCR_PUBLIC_H */
