/*******************************************************************************
* @file  rsi_debugfs.h
* @brief 
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/

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
