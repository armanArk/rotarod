/* staging.h - simple flash-backed staging/journal for event data */
#ifndef __STAGING_H
#define __STAGING_H

#include <stdint.h>
#include <stddef.h>

int staging_init(void);
int staging_append(const uint8_t *data, uint32_t len);
int staging_commit(void);
int staging_has_entries(void);
void staging_clear(void);

#endif /* __STAGING_H */
