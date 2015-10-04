// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Export]
#define ASMJIT_EXPORTS

// [Dependencies - AsmJit]
#include "../base/globals.h"

// [Api-Begin]
#include "../apibegin.h"

namespace asmjit {

// ============================================================================
// [asmjit::DebugUtil]
// ============================================================================

void DebugUtil::debugOutput(const char* str) {
#if ASMJIT_OS_WINDOWS
  ::OutputDebugStringA(str);
#else
  ::fputs(str, stderr);
#endif
}

void DebugUtil::assertionFailed(const char* file, int line, const char* msg) {
  char str[1024];

  snprintf(str, 1024,
    "[asmjit] Assertion failed at %s (line %d):\n"
    "[asmjit] %s\n", file, line, msg);

  // Support buggy `snprintf` implementations.
  str[1023] = '\0';

  debugOutput(str);
  ::abort();
}

} // asmjit namespace

// [Api-End]
#include "../apiend.h"
