/*
 * nav_auto.c - Runtime Flood-Fill Waypoint Auto-Generation
 *
 * Automatically generates navigation waypoints at map load time by
 * flood-filling walkable surfaces from known entity positions.
 * Produces Route[] entries compatible with the existing bot movement
 * code so that no changes to Bots_Move_NORM or combat AI are needed.
 */

#include "../header/bot.h"
#include "../header/ctf.h"
#include "../header/nav_auto.h"

/* ------------------------------------------------------------------ */
/* Temporary storage used only during generation                      */
/* ------------------------------------------------------------------ */

typedef struct {
	vec3_t  pos;        /* ground position */
	int     visited;    /* BFS visited flag */
	int     order;      /* final ordering index (-1 = unordered) */
} nav_node_t;

static nav_node_t nav_nodes[NAV_MAX_CANDIDATES];
static int nav_count;

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

/* Drop a point straight down to find the ground surface.
 * Returns true and writes the ground pos to 'out' if walkable. */
static qboolean Nav_DropToGround(vec3_t start, vec3_t out)
{
	trace_t tr;
	vec3_t end, mins, maxs;

	VectorSet(mins, -NAV_PLAYER_HALF, -NAV_PLAYER_HALF, 0);
	VectorSet(maxs,  NAV_PLAYER_HALF,  NAV_PLAYER_HALF, NAV_PLAYER_HEIGHT);

	VectorCopy(start, end);
	end[2] -= NAV_GROUND_DROP;

	tr = gi.trace(start, mins, maxs, end, NULL, MASK_PLAYERSOLID);

	if (tr.fraction >= 1.0f || tr.allsolid || tr.startsolid)
		return false;
	if (tr.plane.normal[2] < NAV_MIN_NORMAL_Z)
		return false;

	/* reject hazardous surfaces */
	if (gi.pointcontents(tr.endpos) & (CONTENTS_LAVA | CONTENTS_SLIME))
		return false;

	VectorCopy(tr.endpos, out);
	out[2] += 1.0f;
	return true;
}

/* Line-of-sight check between two ground positions using a
 * player-sized bounding box (standing). */
static qboolean Nav_CanWalk(vec3_t from, vec3_t to)
{
	trace_t tr;
	vec3_t s, e, mins, maxs;

	VectorSet(mins, -NAV_PLAYER_HALF, -NAV_PLAYER_HALF, 0);
	VectorSet(maxs,  NAV_PLAYER_HALF,  NAV_PLAYER_HALF, NAV_PLAYER_HEIGHT);

	VectorCopy(from, s);
	s[2] += 8.0f;
	VectorCopy(to, e);
	e[2] += 8.0f;

	tr = gi.trace(s, mins, maxs, e, NULL, MASK_PLAYERSOLID);

	return (tr.fraction >= 1.0f && !tr.allsolid && !tr.startsolid);
}

/* Tight-radius duplicate check. Returns index or -1. */
static int Nav_FindNear(vec3_t pos, float dist_sq)
{
	int i;
	vec3_t d;

	for (i = 0; i < nav_count; i++) {
		VectorSubtract(pos, nav_nodes[i].pos, d);
		if (DotProduct(d, d) < dist_sq)
			return i;
	}
	return -1;
}

/* Add a new candidate node. Returns index or -1 on failure. */
static int Nav_AddNode(vec3_t pos)
{
	if (nav_count >= NAV_MAX_CANDIDATES)
		return -1;
	if (Nav_FindNear(pos, NAV_DEDUP_DIST_SQ) >= 0)
		return -1;

	VectorCopy(pos, nav_nodes[nav_count].pos);
	nav_nodes[nav_count].visited = 0;
	nav_nodes[nav_count].order   = -1;
	return nav_count++;
}

/* ------------------------------------------------------------------ */
/* Seed collection                                                    */
/* ------------------------------------------------------------------ */

static int Nav_CollectSeeds(vec3_t *seeds, int max)
{
	edict_t *ent;
	int i, n = 0;

	for (i = 1; i < globals.num_edicts && n < max; i++) {
		ent = &g_edicts[i];
		if (!ent->inuse || !ent->classname)
			continue;

		/* player spawns */
		if (!strcmp(ent->classname, "info_player_deathmatch") ||
		    !strcmp(ent->classname, "info_player_start")      ||
		    !strcmp(ent->classname, "info_player_team1")      ||
		    !strcmp(ent->classname, "info_player_team2")      ||
		    !strcmp(ent->classname, "info_player_coop")) {
			VectorCopy(ent->s.origin, seeds[n++]);
			continue;
		}

		/* items, weapons, ammo */
		if (!strncmp(ent->classname, "item_", 5)   ||
		    !strncmp(ent->classname, "weapon_", 7)  ||
		    !strncmp(ent->classname, "ammo_", 5)) {
			VectorCopy(ent->s.origin, seeds[n++]);
			continue;
		}

		/* teleport destinations */
		if (!strcmp(ent->classname, "misc_teleporter_dest") ||
		    !strcmp(ent->classname, "misc_teleporter")      ||
		    !strcmp(ent->classname, "trigger_teleport")) {
			VectorCopy(ent->s.origin, seeds[n++]);
			continue;
		}

		/* path corners (used by trains) */
		if (!strcmp(ent->classname, "path_corner")) {
			VectorCopy(ent->s.origin, seeds[n++]);
			continue;
		}

		/* CTF flags */
		if (!strcmp(ent->classname, "item_flag_team1") ||
		    !strcmp(ent->classname, "item_flag_team2")) {
			VectorCopy(ent->s.origin, seeds[n++]);
			continue;
		}
	}
	return n;
}

/* ------------------------------------------------------------------ */
/* Flood fill                                                         */
/* ------------------------------------------------------------------ */

static void Nav_FloodFill(void)
{
	int head = 0;
	int dx, dy;
	vec3_t test, ground;

	while (head < nav_count && nav_count < NAV_MAX_CANDIDATES - 16) {
		if (nav_nodes[head].visited) {
			head++;
			continue;
		}
		nav_nodes[head].visited = 1;

		/* try 8 neighbours on the XY grid */
		for (dx = -1; dx <= 1; dx++) {
			for (dy = -1; dy <= 1; dy++) {
				if (dx == 0 && dy == 0)
					continue;

				VectorCopy(nav_nodes[head].pos, test);
				test[0] += dx * NAV_GRID_STEP;
				test[1] += dy * NAV_GRID_STEP;
				test[2] += 64.0f;   /* lift to clear slopes */

				if (!Nav_DropToGround(test, ground))
					continue;
				if (!Nav_CanWalk(nav_nodes[head].pos, ground))
					continue;

				Nav_AddNode(ground);
			}
		}
		head++;
	}
}

/* ------------------------------------------------------------------ */
/* Nearest-neighbour ordering                                         */
/* ------------------------------------------------------------------ */

/* Produce a sequential ordering of nodes so that consecutive Route[]
 * entries are spatially close.  Uses a greedy nearest-neighbour walk
 * from the first node.  The result is written into nav_nodes[].order. */
static void Nav_OrderNodes(void)
{
	int i, cur, next, ordered;
	float best_d, d;
	vec3_t diff;

	if (nav_count == 0)
		return;

	/* start from node 0 (first spawn-derived node) */
	cur = 0;
	nav_nodes[cur].order = 0;
	ordered = 1;

	while (ordered < nav_count) {
		best_d = 1e30f;
		next   = -1;

		for (i = 0; i < nav_count; i++) {
			if (nav_nodes[i].order >= 0)
				continue;
			VectorSubtract(nav_nodes[i].pos, nav_nodes[cur].pos, diff);
			d = DotProduct(diff, diff);
			if (d < best_d) {
				best_d = d;
				next   = i;
			}
		}

		if (next < 0)
			break;

		nav_nodes[next].order = ordered++;
		cur = next;
	}

	/* assign remaining (if any) */
	for (i = 0; i < nav_count; i++) {
		if (nav_nodes[i].order < 0)
			nav_nodes[i].order = ordered++;
	}
}

/* ------------------------------------------------------------------ */
/* Special entity detection                                           */
/* ------------------------------------------------------------------ */

/* After nodes are placed in Route[], scan entities and mark nodes
 * that are near items, platforms, doors, trains, buttons, and
 * teleporters with the appropriate GRS_ state. */
static void Nav_MarkSpecialEntities(void)
{
	edict_t *ent;
	int i, j;
	vec3_t diff, entpos;
	float dsq, best;

	for (i = (int)maxclients->value + 1; i < globals.num_edicts; i++) {
		ent = &g_edicts[i];
		if (!ent->inuse || !ent->classname)
			continue;

		/* --- Items (weapons, health, ammo, armor, powerups) --- */
		if (ent->item && ent->classname[0] != 'f') {
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, ent->s.origin, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 128.0f * 128.0f) {
				Route[best_j].state = GRS_ITEMS;
				Route[best_j].ent   = ent;
				VectorCopy(ent->s.origin, Route[best_j].Pt);
			}
			continue;
		}

		/* --- Platforms --- */
		if (!Q_stricmp(ent->classname, "func_plat")) {
			VectorAdd(ent->s.origin, ent->mins, entpos);
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, entpos, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 256.0f * 256.0f) {
				Route[best_j].state = GRS_ONPLAT;
				Route[best_j].ent   = ent;
				VectorCopy(entpos, Route[best_j].Pt);
			}
			continue;
		}

		/* --- Doors --- */
		if (!Q_stricmp(ent->classname, "func_door")) {
			VectorAdd(ent->s.origin, ent->mins, entpos);
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, entpos, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 256.0f * 256.0f) {
				Route[best_j].state = GRS_ONDOOR;
				Route[best_j].ent   = ent;
				VectorCopy(entpos, Route[best_j].Pt);
			}
			continue;
		}

		/* --- Trains --- */
		if (!Q_stricmp(ent->classname, "func_train")) {
			VectorAdd(ent->s.origin, ent->mins, entpos);
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, entpos, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 256.0f * 256.0f) {
				Route[best_j].state = GRS_ONTRAIN;
				Route[best_j].ent   = ent;
				VectorCopy(entpos, Route[best_j].Pt);
			}
			continue;
		}

		/* --- Buttons --- */
		if (!Q_stricmp(ent->classname, "func_button")) {
			VectorAdd(ent->s.origin, ent->mins, entpos);
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, entpos, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 256.0f * 256.0f) {
				Route[best_j].state = GRS_PUSHBUTTON;
				Route[best_j].ent   = ent;
				VectorCopy(entpos, Route[best_j].Pt);
			}
			continue;
		}

		/* --- Teleporters --- */
		if (!strcmp(ent->classname, "misc_teleporter_dest")) {
			best = NAV_LINK_DIST_SQ;
			int best_j = -1;
			for (j = 0; j < CurrentIndex; j++) {
				if (Route[j].state != GRS_NORMAL)
					continue;
				VectorSubtract(Route[j].Pt, ent->s.origin, diff);
				dsq = DotProduct(diff, diff);
				if (dsq < best) {
					best = dsq;
					best_j = j;
				}
			}
			if (best_j >= 0 && best < 128.0f * 128.0f) {
				Route[best_j].state = GRS_TELEPORT;
				Route[best_j].ent   = ent;
			}
			continue;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Link generation                                                    */
/* ------------------------------------------------------------------ */

/* Build linkpod[] connectivity between all Route[] nodes.
 * This replaces the post-load G_FindRouteLink for auto-generated
 * routes (G_FindRouteLink is still called afterwards and may add
 * more links). */
static int Nav_BuildLinks(void)
{
	int i, j, k, total = 0;
	int max_links;
	vec3_t diff;
	float dsq, height;

	max_links = MAXLINKPOD - (ctf->value != 0);

	for (i = 0; i < CurrentIndex; i++) {
		k = 0; /* link count for this node */

		for (j = 0; j < CurrentIndex && k < max_links; j++) {
			if (i == j)
				continue;
			/* skip sequential neighbours (reachable by routeindex++) */
			if (abs(i - j) <= 2)
				continue;

			VectorSubtract(Route[j].Pt, Route[i].Pt, diff);
			dsq = DotProduct(diff, diff);
			if (dsq > NAV_LINK_DIST_SQ)
				continue;

			height = diff[2];
			if (height > JumpMax || height < -500.0f)
				continue;

			/* only link normal/items nodes (movers handled separately) */
			if (Route[i].state > GRS_ITEMS && Route[i].state < GRS_GRAPSHOT)
				continue;
			if (Route[j].state > GRS_ITEMS && Route[j].state < GRS_GRAPSHOT)
				continue;

			/* check duplicate link */
			{
				int dup = 0, l;
				for (l = 0; l < k; l++) {
					if ((int)Route[i].linkpod[l] == j) { dup = 1; break; }
				}
				if (dup) continue;
			}

			/* visibility check using raw trace (no entity) */
			{
				trace_t tr;
				vec3_t s, e;
				VectorCopy(Route[i].Pt, s); s[2] += 24.0f;
				VectorCopy(Route[j].Pt, e); e[2] += 24.0f;
				tr = gi.trace(s, NULL, NULL, e, NULL, MASK_SOLID);
				if (tr.fraction < 1.0f || tr.allsolid || tr.startsolid)
					continue;
			}

			Route[i].linkpod[k++] = (unsigned short)j;
			total++;
		}
	}
	return total;
}

/* ------------------------------------------------------------------ */
/* CTF flag direction markers                                         */
/* ------------------------------------------------------------------ */

static void Nav_MarkCTFFlags(void)
{
	int i, j, k;

	if (!ctf->value || !bot_team_flag1 || !bot_team_flag2)
		return;

	j = 0;
	k = 0;
	for (i = CurrentIndex - 1; i >= 0; i--) {
		if (Route[i].state < GRS_ITEMS) {
			if (Route[i].state == GRS_REDFLAG || Route[i].state == GRS_BLUEFLAG) {
				if (Route[i].ent == bot_team_flag1) { j = FOR_FLAG1; k = i; }
				else if (Route[i].ent == bot_team_flag2) { j = FOR_FLAG2; k = i; }
			}
			if (j == FOR_FLAG1)
				Route[i].linkpod[MAXLINKPOD - 1] = (CTF_FLAG1_FLAG | k);
			else if (j == FOR_FLAG2)
				Route[i].linkpod[MAXLINKPOD - 1] = (CTF_FLAG2_FLAG | k);
			else
				Route[i].linkpod[MAXLINKPOD - 1] = 0;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                 */
/* ------------------------------------------------------------------ */

void Nav_AutoGenerate(void)
{
	vec3_t seeds[NAV_MAX_SEEDS];
	vec3_t ground;
	int num_seeds, i;
	int total_links;
	float x;

	gi.dprintf("Nav Auto: starting generation for '%s'...\n", level.mapname);

	/* reset working storage */
	nav_count = 0;
	memset(nav_nodes, 0, sizeof(nav_nodes));
	CurrentIndex = 0;
	memset(Route, 0, sizeof(Route));

	/* compute JumpMax if not already done */
	if (JumpMax == 0) {
		x = VEL_BOT_JUMP - 1.0f * sv_gravity->value * FRAMETIME;
		JumpMax = 0;
		while (1) {
			JumpMax += x * FRAMETIME;
			x -= 1.0f * sv_gravity->value * FRAMETIME;
			if (x < 0) break;
		}
	}

	/* Step 1: gather seed positions from entities */
	num_seeds = Nav_CollectSeeds(seeds, NAV_MAX_SEEDS);
	gi.dprintf("Nav Auto: %d seed positions collected\n", num_seeds);

	if (num_seeds == 0) {
		gi.dprintf("Nav Auto: no seeds found, skipping generation\n");
		return;
	}

	/* Step 2: drop seeds to ground and add as initial nodes */
	for (i = 0; i < num_seeds; i++) {
		vec3_t lifted;
		VectorCopy(seeds[i], lifted);
		lifted[2] += 32.0f;
		if (Nav_DropToGround(lifted, ground))
			Nav_AddNode(ground);
	}
	gi.dprintf("Nav Auto: %d ground seeds\n", nav_count);

	/* Step 3: flood fill walkable area */
	Nav_FloodFill();
	gi.dprintf("Nav Auto: %d walkable positions after flood fill\n", nav_count);

	if (nav_count == 0) {
		gi.dprintf("Nav Auto: no walkable area found\n");
		return;
	}

	/* Step 4: order nodes (nearest-neighbour greedy) */
	Nav_OrderNodes();

	/* Step 5: copy to Route[] in order */
	CurrentIndex = nav_count;
	if (CurrentIndex > MAXNODES)
		CurrentIndex = MAXNODES;

	/* build a temporary map: order -> source index */
	{
		int *order_map;
		order_map = gi.TagMalloc(nav_count * sizeof(int), TAG_LEVEL);
		if (!order_map) {
			gi.dprintf("Nav Auto: out of memory for order map\n");
			return;
		}

		for (i = 0; i < nav_count; i++) {
			int ord = nav_nodes[i].order;
			if (ord >= 0 && ord < CurrentIndex)
				order_map[ord] = i;
		}

		for (i = 0; i < CurrentIndex; i++) {
			int src = order_map[i];
			VectorCopy(nav_nodes[src].pos, Route[i].Pt);
			Route[i].index = i;
			Route[i].state = GRS_NORMAL;
			Route[i].ent   = NULL;
			memset(Route[i].linkpod, 0, sizeof(Route[i].linkpod));
		}

		gi.TagFree(order_map);
	}

	/* Step 6: detect and mark special entities */
	Nav_MarkSpecialEntities();

	/* Step 7: build link-pod connectivity */
	total_links = Nav_BuildLinks();
	gi.dprintf("Nav Auto: %d links created\n", total_links);

	/* Step 8: mark CTF flag directions */
	Nav_MarkCTFFlags();

	gi.dprintf("Nav Auto: generation complete - %d nodes total\n",
	           CurrentIndex);
}
