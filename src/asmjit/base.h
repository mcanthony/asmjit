// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Guard]
#ifndef _ASMJIT_BASE_H
#define _ASMJIT_BASE_H

// [Dependencies - AsmJit]
#include "./build.h"

#include "./base/assembler.h"
#include "./base/codegen.h"
#include "./base/constpool.h"
#include "./base/containers.h"
#include "./base/cpuinfo.h"
#include "./base/cputicks.h"
#include "./base/error.h"
#include "./base/globals.h"
#include "./base/logger.h"
#include "./base/operand.h"
#include "./base/runtime.h"
#include "./base/string.h"
#include "./base/utils.h"
#include "./base/vectypes.h"
#include "./base/vmem.h"
#include "./base/zone.h"

#if !defined(ASMJIT_DISABLE_COMPILER)
#include "./base/compiler.h"
#include "./base/compilerfunc.h"
#include "./base/stream.h"
#endif // !ASMJIT_DISABLE_COMPILER

// [Guard]
#endif // _ASMJIT_BASE_H
