/*
 * cf21bl_syslink_tx.c — shared USART6 TX mutex definition.
 * See cf21bl_syslink_tx.h for which writers use this and why.
 */

#include "cf21bl_syslink_tx.h"

K_MUTEX_DEFINE(cf21bl_syslink_tx_mutex);
