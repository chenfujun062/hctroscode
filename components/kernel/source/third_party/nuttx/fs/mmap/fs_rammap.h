/****************************************************************************
 * fs/mmap/fs_rammap.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __FS_MMAP_RAMMAP_H
#define __FS_MMAP_RAMMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <generated/br2_autoconf.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes one file that has been copied to memory and
 * managed as a share-able "memory mapped" file.  This functionality is
 * intended to provide a substitute for memory mapped files for architectures
 * that do not have MMUs and, hence, cannot support on demand paging of
 * blocks of a file.
 *
 * This copied file has many of the properties of a standard memory mapped
 * file except:
 *
 * - All of the file must be present in memory.  This limits the size of
 *   files that may be memory mapped (especially on MCUs with no significant
 *   RAM resources).
 * - All mapped files are read-only.  You can write to the in-memory image,
 *   but the file contents will not change.
 * - There are not access privileges.
 */

struct fs_rammap_s
{
  struct fs_rammap_s *flink;       /* Implements a singly linked list */
  FAR void           *addr;        /* Start of allocated memory */
  size_t              length;      /* Length of region */
  off_t               offset;      /* File offset */
};

/* This structure defines all "mapped" files */

struct fs_allmaps_s
{
  bool                initialized; /* True: This structure has been initialized */
 SemaphoreHandle_t    exclsem;     /* Provides exclusive access the list */
  struct fs_rammap_s *head;        /* List of mapped files */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the list of all mapped files */

extern struct fs_allmaps_s g_rammaps;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rammap_initialize
 *
 * Description:
 *   Verified that this capability has been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rammap_initialize(void);

/****************************************************************************
 * Name: rammmap
 *
 * Description:
 *   Support simulation of memory mapped files by copying files into RAM.
 *
 * Input Parameters:
 *   fd      file descriptor of the backing file -- required.
 *   length  The length of the mapping.  For exception #1 above, this length
 *           ignored:  The entire underlying media is always accessible.
 *   offset  The offset into the file to map
 *
 * Returned Value:
 *   On success, rammmap() returns a pointer to the mapped area. On error,
 *   the value MAP_FAILED is returned, and errno is set  appropriately.
 *
 *     EBADF
 *      'fd' is not a valid file descriptor.
 *     EINVAL
 *       'length' or 'offset' are invalid
 *     ENOMEM
 *       Insufficient memory is available to map the file.
 *
 ****************************************************************************/

FAR void *rammap(int fd, size_t length, off_t offset);

#endif /* CONFIG_FS_RAMMAP */
#endif /* __FS_MMAP_RAMMAP_H */
