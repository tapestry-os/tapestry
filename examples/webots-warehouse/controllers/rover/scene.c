/*
 * scene.c — see scene.h
 */

#include "scene.h"

bool scene_get_script(int scene, scene_script_t *out)
{
    switch (scene) {
    case 1:  scene1_script(out); return true;
    case 2:  scene2_script(out); return true;
    case 3:  scene3_script(out); return true;
    default: return false;
    }
}

bool scene_has_rf_deck(int scene)
{
    return scene == 2;
}
