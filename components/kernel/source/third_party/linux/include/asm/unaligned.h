/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2007 Ralf Baechle (ralf@linux-mips.org)
 */
#ifndef _ASM_MIPS_UNALIGNED_H
#define _ASM_MIPS_UNALIGNED_H

#include <linux/compiler.h>
# include <linux/unaligned/le_struct.h>
# include <linux/unaligned/be_byteshift.h>

# define get_unaligned	__get_unaligned_le
# define put_unaligned	__put_unaligned_le

# include <linux/unaligned/generic.h>

#endif /* _ASM_MIPS_UNALIGNED_H */
