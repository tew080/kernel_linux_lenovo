/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under both the BSD-style license (found in the
 * LICENSE file in the root directory of this source tree) and the GPLv2 (found
 * in the COPYING file in the root directory of this source tree).
 * You may select, at your option, one of the above-listed licenses.
 */

#ifndef ZSTD_PORTABILITY_MACROS_H
#define ZSTD_PORTABILITY_MACROS_H

/*
 * This header file contains macro definitions to support portability.
 * This header is shared between C and ASM code, so it MUST only
 * contain macro definitions. It MUST not contain any C code.
 *
 * This header ONLY defines macros to detect platforms/feature support.
 *
 */


/* compat. with non-clang compilers */
#ifndef __has_attribute
  #define __has_attribute(x) 0
#endif

/* compat. with non-clang compilers */
#ifndef __has_builtin
#  define __has_builtin(x) 0
#endif

/* compat. with non-clang compilers */
#ifndef __has_feature
#  define __has_feature(x) 0
#endif

/* detects whether we are being compiled under msan */

/* detects whether we are being compiled under asan */

/* detects whether we are being compiled under dfsan */

/* Mark the internal assembly functions as hidden  */
#ifdef __ELF__
# define ZSTD_HIDE_ASM_FUNCTION(func) .hidden func
#elif defined(__APPLE__)
# define ZSTD_HIDE_ASM_FUNCTION(func) .private_extern func
#else
# define ZSTD_HIDE_ASM_FUNCTION(func)
#endif

/* Compile time determination of BMI2 support */


/* Enable runtime BMI2 dispatch based on the CPU.
 * Enabled for clang & gcc >= 11.4 on x86 when BMI2 isn't enabled by default.
 * Disabled for gcc < 11.4 because of a segfault while compiling
 * HUF_compress1X_usingCTable_internal_body().
 */
#ifndef DYNAMIC_BMI2
#  if ((defined(__clang__) && __has_attribute(__target__)) \
      || (defined(__GNUC__) \
          && (__GNUC__ >= 12 || (__GNUC__ == 11 && __GNUC_MINOR__ >= 4)))) \
      && (defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)) \
      && !defined(__BMI2__)
#    define DYNAMIC_BMI2 1
#  else
#    define DYNAMIC_BMI2 0
#  endif
#endif

/*
 * Only enable assembly for GNU C compatible compilers,
 * because other platforms may not support GAS assembly syntax.
 *
 * Only enable assembly for Linux / MacOS / Win32, other platforms may
 * work, but they haven't been tested. This could likely be
 * extended to BSD systems.
 *
 * Disable assembly when MSAN is enabled, because MSAN requires
 * 100% of code to be instrumented to work.
 */
#define ZSTD_ASM_SUPPORTED 1

/*
 * Determines whether we should enable assembly for x86-64
 * with BMI2.
 *
 * Enable if all of the following conditions hold:
 * - ASM hasn't been explicitly disabled by defining ZSTD_DISABLE_ASM
 * - Assembly is supported
 * - We are compiling for x86-64 and either:
 *   - DYNAMIC_BMI2 is enabled
 *   - BMI2 is supported at compile time
 */
#define ZSTD_ENABLE_ASM_X86_64_BMI2 0

/*
 * For x86 ELF targets, add .note.gnu.property section for Intel CET in
 * assembly sources when CET is enabled.
 *
 * Additionally, any function that may be called indirectly must begin
 * with ZSTD_CET_ENDBRANCH.
 */
#if defined(__ELF__) && (defined(__x86_64__) || defined(__i386__)) \
    && defined(__has_include)
# if __has_include(<cet.h>)
#  include <cet.h>
#  define ZSTD_CET_ENDBRANCH _CET_ENDBR
# endif
#endif

#ifndef ZSTD_CET_ENDBRANCH
# define ZSTD_CET_ENDBRANCH
#endif

/*
 * ZSTD_IS_DETERMINISTIC_BUILD must be set to 0 if any compilation macro is
 * active that impacts the compressed output.
 *
 * NOTE: ZSTD_MULTITHREAD is allowed to be set or unset.
 */
#if defined(ZSTD_CLEVEL_DEFAULT) \
    || defined(ZSTD_EXCLUDE_DFAST_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_GREEDY_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_LAZY_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_LAZY2_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_BTLAZY2_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_BTOPT_BLOCK_COMPRESSOR) \
    || defined(ZSTD_EXCLUDE_BTULTRA_BLOCK_COMPRESSOR)
# define ZSTD_IS_DETERMINISTIC_BUILD 0
#else
# define ZSTD_IS_DETERMINISTIC_BUILD 1
#endif

#endif /* ZSTD_PORTABILITY_MACROS_H */
