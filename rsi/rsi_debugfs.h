/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2020-2023 Silicon Labs, Inc.
 */

#ifndef __RSI_DEBUGFS_H__
#define __RSI_DEBUGFS_H__

#include "rsi_main.h"
#include <linux/debugfs.h>

#ifndef CONFIG_RSI_DEBUGFS
static inline int rsi_init_dbgfs(struct rsi_hw *adapter)
{
  return 0;
}

static inline void rsi_remove_dbgfs(struct rsi_hw *adapter)
{
  return;
}
#else
struct rsi_dbg_files {
  const char *name;
  umode_t perms;
  const struct file_operations fops;
};

struct rsi_debugfs {
  struct dentry *subdir;
  struct rsi_dbg_ops *dfs_get_ops;
  struct dentry *rsi_files[MAX_DEBUGFS_ENTRIES];
};
int rsi_init_dbgfs(struct rsi_hw *adapter);
void rsi_remove_dbgfs(struct rsi_hw *adapter);
#endif
#endif
