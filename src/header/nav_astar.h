#ifndef NAV_ASTAR_H
#define NAV_ASTAR_H

#include "local.h"

/*
 * A* Pathfinding with Tactical Cost Weights
 *
 * Provides dynamic, non-deterministic pathfinding for bots on the
 * Route[] navigation graph.  Instead of linear routeindex++ traversal,
 * bots run A* from their current position to a chosen goal, producing
 * varied paths influenced by:
 *   - Danger zones (recent death locations)
 *   - Item attraction (health/ammo/weapon the bot needs)
 *   - Enemy proximity (avoid or approach, per personality)
 *   - Random noise (prevent identical paths)
 *   - Bot personality (BOP_OFFENCE)
 */

/* Per-bot path buffer */
#define NAV_PATH_MAX    512     /* max nodes in a planned path */
#define NAV_DEATHS_MAX  8       /* remembered death locations */
#define NAV_REPLAN_SEC  4.0f    /* seconds between automatic replans */
#define NAV_DANGER_RADIUS 300.0f
#define NAV_DANGER_DECAY  60.0f /* seconds for danger to fully decay */

/* Per-bot navigation state (stored alongside zgcl_t via client index) */
typedef struct {
	int     path[NAV_PATH_MAX]; /* Route[] indices */
	int     pathlen;            /* number of entries in path[] */
	int     pathpos;            /* current position in path[] */
	int     goal_node;          /* target Route[] index */
	float   replan_time;        /* level.time of next replan */

	/* Danger zones: recent death positions */
	vec3_t  deaths[NAV_DEATHS_MAX];
	float   death_times[NAV_DEATHS_MAX];
	int     death_count;
	int     death_next;         /* ring-buffer write index */
} nav_state_t;

/* Initialize all per-bot nav state. Call on map load. */
void Nav_InitStates(void);

/* Plan an A* path from 'start_node' to 'goal_node' for the given bot.
 * Fills the bot's nav_state path buffer.  Returns path length or 0
 * on failure. */
int Nav_PlanPath(edict_t *bot, int start_node, int goal_node);

/* Get the next Route[] index this bot should move toward.
 * Returns -1 if no path is active. */
int Nav_NextNode(edict_t *bot);

/* Advance the bot's path position after reaching the current node. */
void Nav_AdvancePath(edict_t *bot);

/* Select a goal node for the bot based on needs and game state. */
int Nav_SelectGoal(edict_t *bot);

/* Record a bot death for danger-zone tracking. */
void Nav_RecordDeath(edict_t *bot);

/* Find the nearest Route[] node to a world position. */
int Nav_NearestNode(vec3_t pos);

/* Access the nav state for a given client. */
nav_state_t *Nav_GetState(edict_t *ent);

/* Check if a bot has an active A* path. */
qboolean Nav_HasPath(edict_t *bot);

#endif /* NAV_ASTAR_H */
