/* Force-included ahead of store.c so its compiled-in "/store" paths land in a
 * writable temp directory instead. Function-like macros, so `struct stat` and
 * the like are untouched — only calls are redirected. */
#pragma once
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

FILE *sim_fopen(const char *path, const char *mode);
int   sim_stat(const char *path, struct stat *st);
int   sim_unlink(const char *path);
int   sim_rename(const char *a, const char *b);
void  sim_fs_setroot(const char *root);

#define fopen(p, m)  sim_fopen((p), (m))
#define stat(p, s)   sim_stat((p), (s))
#define unlink(p)    sim_unlink(p)
#define rename(a, b) sim_rename((a), (b))
