/*
 * nav_astar.c - A* Pathfinding with Tactical Cost Weights
 *
 * Replaces linear routeindex++ traversal with dynamic A* pathfinding
 * on the Route[] graph.  Each bot plans its own path influenced by
 * danger zones, item needs, enemy positions, random noise, and
 * personality traits to produce varied, tactical navigation.
 */

#include "../header/bot.h"
#include "../header/ctf.h"
#include "../header/nav_astar.h"

/* ------------------------------------------------------------------ */
/* Per-bot navigation state (indexed by client number)                */
/* ------------------------------------------------------------------ */

#define MAX_CLIENTS_PLUS 256
static nav_state_t nav_states[MAX_CLIENTS_PLUS];

/* ------------------------------------------------------------------ */
/* A* working data (single-threaded, reused per search)               */
/* ------------------------------------------------------------------ */

typedef struct {
	float   g;          /* cost from start */
	float   f;          /* g + heuristic */
	int     parent;     /* previous node */
	int     open;       /* in open set */
	int     closed;     /* in closed set */
} astar_node_t;

static astar_node_t astar[MAXNODES];

/* Simple binary min-heap for the open set */
#define HEAP_MAX MAXNODES
static int heap[HEAP_MAX];
static int heap_size;

static void heap_clear(void) { heap_size = 0; }

static void heap_push(int node)
{
	int i, parent;
	if (heap_size >= HEAP_MAX) return;
	i = heap_size++;
	heap[i] = node;
	while (i > 0) {
		parent = (i - 1) / 2;
		if (astar[heap[i]].f < astar[heap[parent]].f) {
			int tmp = heap[i]; heap[i] = heap[parent]; heap[parent] = tmp;
			i = parent;
		} else break;
	}
}

static int heap_pop(void)
{
	int top, i, child;
	if (heap_size == 0) return -1;
	top = heap[0];
	heap[0] = heap[--heap_size];
	i = 0;
	while (1) {
		child = 2 * i + 1;
		if (child >= heap_size) break;
		if (child + 1 < heap_size && astar[heap[child + 1]].f < astar[heap[child]].f)
			child++;
		if (astar[heap[child]].f < astar[heap[i]].f) {
			int tmp = heap[i]; heap[i] = heap[child]; heap[child] = tmp;
			i = child;
		} else break;
	}
	return top;
}

/* ------------------------------------------------------------------ */
/* State access                                                       */
/* ------------------------------------------------------------------ */

nav_state_t *Nav_GetState(edict_t *ent)
{
	int idx;
	if (!ent || !ent->client) return &nav_states[0];
	idx = ent - g_edicts;
	if (idx < 0 || idx >= MAX_CLIENTS_PLUS) idx = 0;
	return &nav_states[idx];
}

void Nav_InitStates(void)
{
	memset(nav_states, 0, sizeof(nav_states));
}

/* ------------------------------------------------------------------ */
/* Nearest node lookup                                                */
/* ------------------------------------------------------------------ */

int Nav_NearestNode(vec3_t pos)
{
	int i, best = -1;
	float best_d = 1e30f, d;
	vec3_t diff;

	for (i = 0; i < CurrentIndex; i++) {
		VectorSubtract(pos, Route[i].Pt, diff);
		d = DotProduct(diff, diff);
		if (d < best_d) {
			best_d = d;
			best = i;
		}
	}
	return best;
}

/* ------------------------------------------------------------------ */
/* Cost function                                                      */
/* ------------------------------------------------------------------ */

static float Nav_Heuristic(int a, int b)
{
	vec3_t diff;
	VectorSubtract(Route[a].Pt, Route[b].Pt, diff);
	return VectorLength(diff);
}

/* Danger penalty: higher cost near recent death locations */
static float Nav_DangerCost(int node, edict_t *bot)
{
	nav_state_t *ns = Nav_GetState(bot);
	int i;
	float penalty = 0.0f;
	float age, factor;
	vec3_t diff;
	float dist;

	for (i = 0; i < ns->death_count && i < NAV_DEATHS_MAX; i++) {
		VectorSubtract(Route[node].Pt, ns->deaths[i], diff);
		dist = VectorLength(diff);
		if (dist < NAV_DANGER_RADIUS) {
			age = level.time - ns->death_times[i];
			if (age < 0) age = 0;
			factor = 1.0f - (age / NAV_DANGER_DECAY);
			if (factor < 0) factor = 0;
			penalty += (NAV_DANGER_RADIUS - dist) * factor * 0.5f;
		}
	}
	return penalty;
}

/* Item attraction: reduce cost for nodes near items the bot needs */
static float Nav_ItemAttraction(int node, edict_t *bot)
{
	float attraction = 0.0f;

	if (Route[node].state != GRS_ITEMS || !Route[node].ent)
		return 0.0f;

	if (!Route[node].ent->inuse || !Route[node].ent->item)
		return 0.0f;

	/* health need */
	if (bot->health < 50) {
		if (!strncmp(Route[node].ent->classname, "item_health", 11))
			attraction += (50.0f - bot->health) * 2.0f;
	}

	/* armor need */
	if (Route[node].ent->item->flags & IT_ARMOR) {
		attraction += 40.0f;
	}

	/* weapon/ammo */
	if (Route[node].ent->item->flags & (IT_WEAPON | IT_AMMO)) {
		attraction += 20.0f;
	}

	/* powerups */
	if (Route[node].ent->item->flags & IT_POWERUP) {
		attraction += 60.0f;
	}

	return attraction;
}

/* Personality factor: offensive bots take risky paths, defensive bots
 * prefer safe paths.  BOP_OFFENCE (param3) ranges 0-9. */
static float Nav_PersonalityNoise(edict_t *bot)
{
	float offense;
	if (!bot->client || !(bot->svflags & SVF_MONSTER))
		return 0.0f;

	offense = (float)Bot[bot->client->zc.botindex].param[BOP_OFFENCE];
	/* more offensive = more random (exploratory) */
	return (random() - 0.5f) * (20.0f + offense * 8.0f);
}

/* Edge cost from node 'from' to node 'to', influenced by
 * all tactical factors. */
static float Nav_EdgeCost(int from, int to, edict_t *bot)
{
	float cost;
	vec3_t diff;

	/* base: euclidean distance */
	VectorSubtract(Route[to].Pt, Route[from].Pt, diff);
	cost = VectorLength(diff);

	/* danger penalty */
	cost += Nav_DangerCost(to, bot);

	/* item attraction (subtract = lower cost = more attractive) */
	cost -= Nav_ItemAttraction(to, bot);

	/* personality-driven random noise */
	cost += Nav_PersonalityNoise(bot);

	/* clamp minimum cost to avoid negative cycles */
	if (cost < 1.0f) cost = 1.0f;

	return cost;
}

/* ------------------------------------------------------------------ */
/* Neighbor enumeration                                               */
/* ------------------------------------------------------------------ */

/* Collect all neighbors of a node into 'out'. Returns count.
 * Neighbors = sequential (i-1, i+1) + linkpods + reverse sequential. */
#define MAX_NEIGHBORS 32

static int Nav_GetNeighbors(int node, int *out)
{
	int n = 0, i, k;
	int max_links = MAXLINKPOD - (ctf->value != 0);

	/* sequential: forward */
	if (node + 1 < CurrentIndex) {
		out[n++] = node + 1;
	}

	/* sequential: backward */
	if (node - 1 >= 0) {
		out[n++] = node - 1;
	}

	/* linkpods */
	if (Route[node].state <= GRS_ITEMS) {
		for (i = 0; i < max_links && n < MAX_NEIGHBORS; i++) {
			k = (int)Route[node].linkpod[i];
			if (k <= 0 || k >= CurrentIndex)
				continue;
			/* avoid duplicates */
			if (k == node + 1 || k == node - 1)
				continue;
			out[n++] = k;
		}
	}

	return n;
}

/* ------------------------------------------------------------------ */
/* A* search                                                          */
/* ------------------------------------------------------------------ */

int Nav_PlanPath(edict_t *bot, int start_node, int goal_node)
{
	nav_state_t *ns = Nav_GetState(bot);
	int cur, i, nb_count;
	int neighbors[MAX_NEIGHBORS];
	float tentative_g;
	int iterations = 0;
	int max_iterations;

	if (start_node < 0 || start_node >= CurrentIndex ||
	    goal_node < 0 || goal_node >= CurrentIndex)
		return 0;

	if (start_node == goal_node) {
		ns->path[0] = goal_node;
		ns->pathlen = 1;
		ns->pathpos = 0;
		ns->goal_node = goal_node;
		return 1;
	}

	/* limit search effort proportional to graph size */
	max_iterations = CurrentIndex * 2;
	if (max_iterations > 20000) max_iterations = 20000;

	/* initialize A* data */
	for (i = 0; i < CurrentIndex; i++) {
		astar[i].g = 1e30f;
		astar[i].f = 1e30f;
		astar[i].parent = -1;
		astar[i].open = 0;
		astar[i].closed = 0;
	}

	heap_clear();

	astar[start_node].g = 0;
	astar[start_node].f = Nav_Heuristic(start_node, goal_node);
	astar[start_node].open = 1;
	heap_push(start_node);

	while (heap_size > 0 && iterations < max_iterations) {
		cur = heap_pop();
		iterations++;

		if (cur < 0 || cur >= CurrentIndex)
			break;

		if (cur == goal_node) {
			/* reconstruct path */
			int path_len = 0;
			int temp[NAV_PATH_MAX];
			int n = cur;

			while (n >= 0 && path_len < NAV_PATH_MAX) {
				temp[path_len++] = n;
				n = astar[n].parent;
			}

			/* reverse into ns->path */
			ns->pathlen = path_len;
			ns->pathpos = 0;
			ns->goal_node = goal_node;
			for (i = 0; i < path_len; i++)
				ns->path[i] = temp[path_len - 1 - i];

			return path_len;
		}

		astar[cur].open = 0;
		astar[cur].closed = 1;

		nb_count = Nav_GetNeighbors(cur, neighbors);
		for (i = 0; i < nb_count; i++) {
			int nb = neighbors[i];
			if (astar[nb].closed)
				continue;

			tentative_g = astar[cur].g + Nav_EdgeCost(cur, nb, bot);

			if (tentative_g < astar[nb].g) {
				astar[nb].g = tentative_g;
				astar[nb].f = tentative_g + Nav_Heuristic(nb, goal_node);
				astar[nb].parent = cur;

				if (!astar[nb].open) {
					astar[nb].open = 1;
					heap_push(nb);
				}
			}
		}
	}

	/* no path found - clear */
	ns->pathlen = 0;
	ns->pathpos = 0;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Path query helpers                                                 */
/* ------------------------------------------------------------------ */

qboolean Nav_HasPath(edict_t *bot)
{
	nav_state_t *ns = Nav_GetState(bot);
	return (ns->pathlen > 0 && ns->pathpos < ns->pathlen);
}

int Nav_NextNode(edict_t *bot)
{
	nav_state_t *ns = Nav_GetState(bot);
	if (ns->pathpos >= ns->pathlen)
		return -1;
	return ns->path[ns->pathpos];
}

void Nav_AdvancePath(edict_t *bot)
{
	nav_state_t *ns = Nav_GetState(bot);
	if (ns->pathpos < ns->pathlen)
		ns->pathpos++;
}

/* ------------------------------------------------------------------ */
/* Death tracking                                                     */
/* ------------------------------------------------------------------ */

void Nav_RecordDeath(edict_t *bot)
{
	nav_state_t *ns = Nav_GetState(bot);
	int idx = ns->death_next;

	VectorCopy(bot->s.origin, ns->deaths[idx]);
	ns->death_times[idx] = level.time;

	ns->death_next = (idx + 1) % NAV_DEATHS_MAX;
	if (ns->death_count < NAV_DEATHS_MAX)
		ns->death_count++;
}

/* ------------------------------------------------------------------ */
/* Goal selection                                                     */
/* ------------------------------------------------------------------ */

/* Pick a goal node for the bot based on current needs and game state.
 * Returns a Route[] index, or -1 if no good goal exists. */
int Nav_SelectGoal(edict_t *bot)
{
	int i, best = -1;
	float best_score = -1e30f;
	float score, dist;
	vec3_t diff;
	int cur_node;
	float offense;

	if (CurrentIndex <= 0)
		return -1;

	cur_node = Nav_NearestNode(bot->s.origin);
	if (cur_node < 0)
		return -1;

	offense = 5.0f;
	if (bot->client && (bot->svflags & SVF_MONSTER))
		offense = (float)Bot[bot->client->zc.botindex].param[BOP_OFFENCE];

	for (i = 0; i < CurrentIndex; i++) {
		if (i == cur_node)
			continue;

		score = 0.0f;

		/* distance penalty (prefer not-too-far goals) */
		VectorSubtract(Route[i].Pt, bot->s.origin, diff);
		dist = VectorLength(diff);
		score -= dist * 0.1f;

		/* item goals */
		if (Route[i].state == GRS_ITEMS && Route[i].ent &&
		    Route[i].ent->inuse && Route[i].ent->item) {

			/* health urgency */
			if (bot->health < 50 &&
			    !strncmp(Route[i].ent->classname, "item_health", 11))
				score += (60.0f - bot->health) * 3.0f;

			/* armor need */
			if (Route[i].ent->item->flags & IT_ARMOR)
				score += 80.0f;

			/* weapon/ammo */
			if (Route[i].ent->item->flags & (IT_WEAPON | IT_AMMO))
				score += 40.0f + offense * 5.0f;

			/* powerups */
			if (Route[i].ent->item->flags & IT_POWERUP)
				score += 120.0f + offense * 10.0f;
		}

		/* exploration bonus: prefer nodes far from last goal */
		{
			nav_state_t *ns = Nav_GetState(bot);
			if (ns->goal_node >= 0 && ns->goal_node < CurrentIndex) {
				VectorSubtract(Route[i].Pt, Route[ns->goal_node].Pt, diff);
				score += VectorLength(diff) * 0.02f;
			}
		}

		/* danger penalty */
		score -= Nav_DangerCost(i, bot);

		/* random exploration factor scaled by personality */
		score += random() * (30.0f + offense * 15.0f);

		if (score > best_score) {
			best_score = score;
			best = i;
		}
	}

	return best;
}
