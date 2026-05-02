#ifndef NAV_AUTO_H
#define NAV_AUTO_H

#include "local.h"

/*
 * Runtime Flood-Fill Waypoint Auto-Generation
 *
 * Replaces manually authored .chn/.chf chain files with automatic
 * navigation graph generation at map load time. Uses the Q2 engine's
 * gi.trace / gi.pointcontents to flood-fill walkable areas from known
 * seed positions (spawns, items, teleporters), then builds a
 * connectivity graph that fills the existing Route[] / route_t array.
 *
 * Backward compatible: if a chain file exists it is still loaded;
 * auto-generation only runs when no chain file is found.
 */

/* Tuning constants */
#define NAV_GRID_STEP       96      /* sampling interval in map units */
#define NAV_MAX_SEEDS       512     /* max entity seed positions */
#define NAV_MAX_CANDIDATES  10000   /* temp node storage (== MAXNODES) */
#define NAV_LINK_DIST       250.0f  /* max link distance between nodes */
#define NAV_LINK_DIST_SQ    (NAV_LINK_DIST * NAV_LINK_DIST)
#define NAV_DEDUP_DIST      48.0f   /* min distance between nodes */
#define NAV_DEDUP_DIST_SQ   (NAV_DEDUP_DIST * NAV_DEDUP_DIST)
#define NAV_GROUND_DROP     1024.0f /* max drop-to-ground distance */
#define NAV_MIN_NORMAL_Z    0.7f    /* min ground normal for walkable */
#define NAV_PLAYER_HALF     16      /* player half-width */
#define NAV_PLAYER_HEIGHT   32      /* standing bbox top */

/* Generate navigation nodes and fill Route[]/CurrentIndex.
 * Call after all entities are spawned (replaces chain-file load). */
void Nav_AutoGenerate(void);

#endif /* NAV_AUTO_H */
