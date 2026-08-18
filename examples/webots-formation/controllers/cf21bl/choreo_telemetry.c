/*
 * choreo_telemetry.c — see choreo_telemetry.h
 */

#include "choreo_telemetry.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

struct choreo_telemetry {
    FILE *f;
};

choreo_telemetry_t *choreo_telemetry_open(element_id_t element_id)
{
    const char *dir = getenv("TAPESTRY_TELEMETRY_DIR");
    if (dir == NULL || dir[0] == '\0') {
        return NULL;
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/choreo_%u.csv", dir, (unsigned)element_id);

    FILE *f = fopen(path, "w");
    if (f == NULL) {
        fprintf(stderr, "id=%u choreo_telemetry: could not open %s — "
                "telemetry disabled\n", (unsigned)element_id, path);
        return NULL;
    }

    choreo_telemetry_t *t = malloc(sizeof(*t));
    t->f = f;

    fprintf(f, "tick,wall_time_s,element_id,pos_x,pos_y,"
               "quorum_state,fresh_count,role,"
               "goal_type,script_step,script_complete,goal_achieved,"
               "directive_type,directive_target_x,directive_target_y,"
               "wm_json\n");
    return t;
}

/* Appends one wm_entries JSON element to buf (bounds-checked via the
 * caller's remaining-space bookkeeping). Mirrors the fields
 * sdk/python/tapestry expects in its wm_entries dicts (choreo.py's Choreo
 * class docstring): id, x, y, is_active, is_stale, is_self, achieved.
 *
 * "achieved" is the peer's gossiped own-goal predicate, and it is not
 * optional: a step with scope="all" advances on
 * choreo_collective_achieved(), which reads exactly this bit.  Omitting it
 * made the Python replay engine see every peer as never-achieved, so a
 * scope="all" step that advanced on achievement in flight replayed as
 * advancing on its timeout instead — a divergence in the harness, not in
 * the engine it exists to check. */
static int append_entry_json(char *buf, size_t bufsz, size_t off,
                             const wm_entry_t *e, element_id_t id,
                             bool first)
{
    return snprintf(buf + off, bufsz > off ? bufsz - off : 0,
                    "%s{\"id\":%u,\"x\":%.4f,\"y\":%.4f,"
                    "\"is_active\":%s,\"is_stale\":%s,\"is_self\":%s,"
                    "\"achieved\":%s}",
                    first ? "" : ",",
                    (unsigned)id,
                    (double)e->state.position.x, (double)e->state.position.y,
                    e->is_active ? "true" : "false",
                    e->is_stale  ? "true" : "false",
                    e->is_self   ? "true" : "false",
                    e->state.goal_achieved ? "true" : "false");
}

/* Writes s as one RFC 4180 CSV field: wrapped in double quotes, with every
 * embedded double quote doubled. wm_json's keys ("id", "x", ...) are
 * themselves quoted JSON strings, so this escaping is required, not
 * optional — Python's csv module (and any RFC 4180 reader) treats a lone
 * unescaped quote inside a quoted field as closing it early. */
static void write_csv_quoted(FILE *f, const char *s)
{
    fputc('"', f);
    for (; *s != '\0'; s++) {
        if (*s == '"') {
            fputc('"', f);
        }
        fputc(*s, f);
    }
    fputc('"', f);
}

void choreo_telemetry_write(choreo_telemetry_t *telemetry,
                            uint32_t tick,
                            double wall_time_s,
                            const world_model_t *wm,
                            const scr_state_t *scr,
                            const tapestry_bse_directive_t *dir)
{
    if (telemetry == NULL || telemetry->f == NULL) {
        return;
    }

    /* wm_entries snapshot as a JSON array — only entries this element has
     * ever heard from (is_active) are worth recording; a full MAX_ELEMENTS
     * sweep of never-seen slots would just be noise. */
    char wm_json[4096];
    size_t off = 0;
    off += (size_t)snprintf(wm_json + off, sizeof(wm_json) - off, "[");
    bool first = true;
    for (unsigned id = 0; id < MAX_ELEMENTS; id++) {
        const wm_entry_t *e = &wm->entries[id];
        if (!e->is_active) {
            continue;
        }
        int n = append_entry_json(wm_json, sizeof(wm_json), off, e,
                                  (element_id_t)id, first);
        if (n < 0 || off + (size_t)n >= sizeof(wm_json)) {
            break;   /* truncation guard — MAX_ELEMENTS=32 never gets here */
        }
        off += (size_t)n;
        first = false;
    }
    off += (size_t)snprintf(wm_json + off, sizeof(wm_json) - off, "]");

    const element_id_t self_id = wm->owner_id;
    const wm_entry_t *self_e = &wm->entries[self_id];

    fprintf(telemetry->f,
           "%u,%.3f,%u,%.4f,%.4f,%d,%u,%d,%d,%d,%d,%d,%d,%.4f,%.4f,",
           tick, wall_time_s, (unsigned)self_id,
           (double)self_e->state.position.x, (double)self_e->state.position.y,
           (int)scr->quorum_state, (unsigned)scr->fresh_count, (int)scr->role,
           (int)choreo_current_goal_type(), choreo_script_step(),
           choreo_script_complete() ? 1 : 0, choreo_goal_achieved() ? 1 : 0,
           (int)dir->type, (double)dir->target.x, (double)dir->target.y);
    write_csv_quoted(telemetry->f, wm_json);
    fputc('\n', telemetry->f);
}

void choreo_telemetry_close(choreo_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }
    if (telemetry->f != NULL) {
        fclose(telemetry->f);
    }
    free(telemetry);
}
