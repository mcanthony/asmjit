// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Guard]
#ifndef _ASMJIT_BUILD_H
#define _ASMJIT_BUILD_H

// ============================================================================
// [asmjit::Build - Configuration]
// ============================================================================

// AsmJit is by default compiled only for a host processor for the purpose of
// JIT code generation. Both Assembler and Compiler code generators are compiled
// by default. Preprocessor macros can be used to change the default behavior.

// External Config File
// --------------------
//
// Define in case your configuration is generated in an external file to be
// included.

#if defined(ASMJIT_CONFIG_FILE)
# include ASMJIT_CONFIG_FILE
#endif // ASMJIT_CONFIG_FILE

// AsmJit Static Builds and Embedding
// ----------------------------------
//
// These definitions can be used to enable static library build. Embed is used
// when AsmJit's source code is embedded directly in another project, implies
// static build as well.
//
// #define ASMJIT_EMBED              // Asmjit is embedded (implies ASMJIT_STATIC).
// #define ASMJIT_STATIC             // Define to enable static-library build.

// AsmJit Build Modes
// ------------------
//
// These definitions control the build mode and tracing support. The build mode
// should be auto-detected at compile time, but it's possible to override it in
// case that the auto-detection fails.
//
// Tracing is a feature that is never compiled by default and it's only used to
// debug AsmJit itself.
//
// #define ASMJIT_DEBUG              // Define to enable debug-mode.
// #define ASMJIT_RELEASE            // Define to enable release-mode.
// #define ASMJIT_TRACE              // Define to enable tracing.

// AsmJit Build Backends
// ---------------------
//
// These definitions control which backends to compile. If none of these is
// defined AsmJit will use host architecture by default (for JIT code generation).
//
// #define ASMJIT_BUILD_X86          // Define to enable x86 instruction set (32-bit).
// #define ASMJIT_BUILD_X64          // Define to enable x64 instruction set (64-bit).
// #define ASMJIT_BUILD_HOST         // Define to enable host instruction set.

// AsmJit Build Features
// ---------------------
//
// Flags can be defined to disable standard features. These are handy especially
// when building asmjit statically and some features are not needed or unwanted
// (like Compiler).
//
// AsmJit features are enabled by default.
// #define ASMJIT_DISABLE_COMPILER   // Disable Compiler (completely).
// #define ASMJIT_DISABLE_LOGGER     // Disable Logger (completely).
// #define ASMJIT_DISABLE_NAMES      // Disable everything that uses strings
//                                   // (instruction names, error names, ...).

// Prevent compile-time errors caused by misconfiguration.
#if defined(ASMJIT_DISABLE_NAMES) && !defined(ASMJIT_DISABLE_LOGGER)
# error "[asmjit] ASMJIT_DISABLE_NAMES requires ASMJIT_DISABLE_LOGGER to be defined."
#endif // ASMJIT_DISABLE_NAMES && !ASMJIT_DISABLE_LOGGER

// If ASMJIT_DEBUG and ASMJIT_RELEASE are not defined ASMJIT_DEBUG will be
// detected using the compiler specific macros. This enables to set the build
// type using an IDE.
#if !defined(ASMJIT_DEBUG) && !defined(ASMJIT_RELEASE)
# if defined(_DEBUG)
#  define ASMJIT_DEBUG
# endif
#endif

// Sanity.
#if defined(ASMJIT_EMBED) && !defined(ASMJIT_STATIC)
# define ASMJIT_STATIC
#endif

// ============================================================================
// [asmjit::Build - Version]
// ============================================================================

// [@VERSION{@]
#define ASMJIT_VERSION_MAJOR 1
#define ASMJIT_VERSION_MINOR 0
#define ASMJIT_VERSION_PATCH 0
#define ASMJIT_VERSION_STRING "1.0.0"
// [@VERSION}@]

// ============================================================================
// [asmjit::Build - CxxTool]
// ============================================================================

// [@WIN32_CRT_NO_DEPRECATE{@]
#if defined(_MSC_VER) && defined(ASMJIT_EXPORTS)
# if !defined(_CRT_SECURE_NO_DEPRECATE)
#  define _CRT_SECURE_NO_DEPRECATE
# endif
# if !defined(_CRT_SECURE_NO_WARNINGS)
#  define _CRT_SECURE_NO_WARNINGS
# endif
#endif
// [@WIN32_CRT_NO_DEPRECATE}@]

#include <new>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// [@WIN32_LEAN_AND_MEAN{@]
#if (defined(_WIN32) || defined(_WINDOWS)) && !defined(ASMJIT_SUPPRESS_WINDOWS_H)
# if !defined(WIN32_LEAN_AND_MEAN)
#  define WIN32_LEAN_AND_MEAN
#  define ASMJIT_UNDEF_WIN32_LEAN_AND_MEAN
# endif
# if !defined(NOMINMAX)
#  define NOMINMAX
#  define ASMJIT_UNDEF_NOMINMAX
# endif
# include <windows.h>
# if defined(ASMJIT_UNDEF_NOMINMAX)
#  undef NOMINMAX
#  undef ASMJIT_UNDEF_NOMINMAX
# endif
# if defined(ASMJIT_UNDEF_WIN32_LEAN_AND_MEAN)
#  undef WIN32_LEAN_AND_MEAN
#  undef ASMJIT_UNDEF_WIN32_LEAN_AND_MEAN
# endif
#endif
// [@WIN32_LEAN_AND_MEAN}@]

// [@OS{@]
#if defined(_WIN32) || defined(_WINDOWS)
#define ASMJIT_OS_WINDOWS       (1)
#else
#define ASMJIT_OS_WINDOWS       (0)
#endif

#if defined(__APPLE__)
# include <TargetConditionals.h>
# define ASMJIT_OS_MAC          (TARGET_OS_MAC)
# define ASMJIT_OS_IOS          (TARGET_OS_IPHONE)
#else
# define ASMJIT_OS_MAC          (0)
# define ASMJIT_OS_IOS          (0)
#endif

#if defined(__ANDROID__)
# define ASMJIT_OS_ANDROID      (1)
#else
# define ASMJIT_OS_ANDROID      (0)
#endif

#if defined(__linux__) || defined(__ANDROID__)
# define ASMJIT_OS_LINUX        (1)
#else
# define ASMJIT_OS_LINUX        (0)
#endif

#if defined(__DragonFly__)
# define ASMJIT_OS_DRAGONFLYBSD (1)
#else
# define ASMJIT_OS_DRAGONFLYBSD (0)
#endif

#if defined(__FreeBSD__)
# define ASMJIT_OS_FREEBSD      (1)
#else
# define ASMJIT_OS_FREEBSD      (0)
#endif

#if defined(__NetBSD__)
# define ASMJIT_OS_NETBSD       (1)
#else
# define ASMJIT_OS_NETBSD       (0)
#endif

#if defined(__OpenBSD__)
# define ASMJIT_OS_OPENBSD      (1)
#else
# define ASMJIT_OS_OPENBSD      (0)
#endif

#if defined(__QNXNTO__)
# define ASMJIT_OS_QNX          (1)
#else
# define ASMJIT_OS_QNX          (0)
#endif

#if defined(__sun)
# define ASMJIT_OS_SOLARIS      (1)
#else
# define ASMJIT_OS_SOLARIS      (0)
#endif

#define ASMJIT_OS_BSD           (ASMJIT_OS_FREEBSD || ASMJIT_OS_DRAGONFLYBSD || ASMJIT_OS_NETBSD || ASMJIT_OS_OPENBSD || ASMJIT_OS_MAC)
#define ASMJIT_OS_POSIX         (!ASMJIT_OS_WINDOWS)
// [@OS}@]

#if ASMJIT_OS_POSIX
# include <pthread.h>
#endif // ASMJIT_OS_POSIX

// [@ARCH{@]
// \def ASMJIT_ARCH_ARM
// Defined if the target architecture is a 32-bit ARM.
//
// \def ASMJIT_ARCH_ARM64
// Defined if the target architecture is a 64-bit ARM.
//
// \def ASMJIT_ARCH_X64
// Defined if the target architecture is a 64-bit X64/AMD64
//
// \def ASMJIT_ARCH_X86
// Defined if the target architecture is a 32-bit X86/IA32
#if (defined(_M_X64  ) || defined(__x86_64) || defined(__x86_64__) || \
     defined(_M_AMD64) || defined(__amd64 ) || defined(__amd64__ ))
# define ASMJIT_ARCH_X64          (1)
#else
# define ASMJIT_ARCH_X64          (0)
#endif
#if (defined(_M_IX86 ) || defined(__X86__ ) || defined(__i386  ) || \
     defined(__IA32__) || defined(__I86__ ) || defined(__i386__) || \
     defined(__i486__) || defined(__i586__) || defined(__i686__))
# define ASMJIT_ARCH_X86          (!ASMJIT_ARCH_X64)
#else
# define ASMJIT_ARCH_X86          (0)
#endif
#if ASMJIT_ARCH_X86 || ASMJIT_ARCH_X64
# define ASMJIT_ARCH_64BIT        (ASMJIT_ARCH_X64)
# define ASMJIT_ARCH_BE           (0)
# define ASMJIT_ARCH_LE           (1)
# define ASMJIT_ARCH_UNALIGNED_16 (1)
# define ASMJIT_ARCH_UNALIGNED_32 (1)
# define ASMJIT_ARCH_UNALIGNED_64 (1)
# if !defined(ASMJIT_ARCH_MMX) && (!ASMJIT_ARCH_X64 && (defined(__MMX__) || defined(__i686__)))
#  define ASMJIT_ARCH_MMX         (1)
# endif
# if !defined(ASMJIT_ARCH_SSE) && (ASMJIT_ARCH_X64 || (defined(_M_IX86_FP) && _M_IX86_FP >= 1) || defined(__SSE__))
#  define ASMJIT_ARCH_SSE         (1)
# endif
# if !defined(ASMJIT_ARCH_SSE2) && (ASMJIT_ARCH_X64 || (defined(_M_IX86_FP) && _M_IX86_FP >= 2) || defined(__SSE2__))
#  define ASMJIT_ARCH_SSE2        (1)
# endif
# if !defined(ASMJIT_ARCH_SSE3) && (defined(__SSE3__))
#  define ASMJIT_ARCH_SSE3        (1)
# endif
# if !defined(ASMJIT_ARCH_SSSE3) && (defined(__SSSE3__))
#  define ASMJIT_ARCH_SSSE3       (1)
# endif
# if !defined(ASMJIT_ARCH_SSE4_1) && (defined(__SSE4_1__))
#  define ASMJIT_ARCH_SSE4_1      (1)
# endif
# if !defined(ASMJIT_ARCH_SSE4_2) && (defined(__SSE4_2__))
#  define ASMJIT_ARCH_SSE4_2      (1)
# endif
# if !defined(ASMJIT_ARCH_AVX) && (defined(__AVX__))
#  define ASMJIT_ARCH_AVX         (1)
# endif
# if !defined(ASMJIT_ARCH_AVX2) && (defined(__AVX2__))
#  define ASMJIT_ARCH_AVX2        (1)
# endif
#endif
#if !defined(ASMJIT_ARCH_AVX2)
# define ASMJIT_ARCH_AVX2         (0)
#endif
#if !defined(ASMJIT_ARCH_AVX)
# define ASMJIT_ARCH_AVX          (ASMJIT_ARCH_AVX2)
#endif
#if !defined(ASMJIT_ARCH_SSE4_2)
# define ASMJIT_ARCH_SSE4_2       (ASMJIT_ARCH_AVX)
#endif
#if !defined(ASMJIT_ARCH_SSE4_1)
# define ASMJIT_ARCH_SSE4_1       (ASMJIT_ARCH_SSE4_2)
#endif
#if !defined(ASMJIT_ARCH_SSSE3)
# define ASMJIT_ARCH_SSSE3        (ASMJIT_ARCH_SSE4_1)
#endif
#if !defined(ASMJIT_ARCH_SSE3)
# define ASMJIT_ARCH_SSE3         (ASMJIT_ARCH_SSSE3)
#endif
#if !defined(ASMJIT_ARCH_SSE2)
# define ASMJIT_ARCH_SSE2         (ASMJIT_ARCH_SSE3)
#endif
#if !defined(ASMJIT_ARCH_SSE)
# define ASMJIT_ARCH_SSE          (ASMJIT_ARCH_SSE2)
#endif
#if !defined(ASMJIT_ARCH_MMX)
# define ASMJIT_ARCH_MMX          (0)
#endif

#if (defined(_M_ARM  ) || defined(__arm__ ) || defined(__arm) || \
     defined(_M_ARMT ) || defined(__thumb__))
# define ASMJIT_ARCH_ARM          (1)
# define ASMJIT_ARCH_ARM64        (0)
#else
# define ASMJIT_ARCH_ARM          (0)
# define ASMJIT_ARCH_ARM64        (0)
#endif
#if ASMJIT_ARCH_ARM || ASMJIT_ARCH_ARM64
# define ASMJIT_ARCH_64BIT        (ASMJIT_ARCH_ARM64)
# define ASMJIT_ARCH_BE           (0)
# define ASMJIT_ARCH_LE           (1)
# define ASMJIT_ARCH_UNALIGNED_16 (0)
# define ASMJIT_ARCH_UNALIGNED_32 (0)
# define ASMJIT_ARCH_UNALIGNED_64 (0)
# if !defined(ASMJIT_ARCH_NEON) && defined(__ARM_NEON__)
#  define ASMJIT_ARCH_NEON        (1)
# endif
#endif
#if !defined(ASMJIT_ARCH_NEON)
# define ASMJIT_ARCH_NEON         (0)
#endif
// [@ARCH}@]

// [@ARCH_INCLUDE{@]
#if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__CODEGEARC__)
# include <intrin.h>
#endif

#if ASMJIT_ARCH_SSE
# include <xmmintrin.h>
#endif
#if ASMJIT_ARCH_SSE2
# include <emmintrin.h>
#endif
#if ASMJIT_ARCH_SSE3 && !defined(_MSC_VER)
# include <pmmintrin.h>
#endif
#if ASMJIT_ARCH_SSSE3
# include <tmmintrin.h>
#endif
#if ASMJIT_ARCH_SSE4_1
# include <smmintrin.h>
#endif
#if ASMJIT_ARCH_SSE4_2
# include <nmmintrin.h>
#endif
#if ASMJIT_ARCH_AVX || ASMJIT_ARCH_AVX2
# include <immintrin.h>
#endif

#if ASMJIT_ARCH_NEON
# include <arm_neon.h>
#endif
// [@ARCH_INCLUDE}@]

// [@CC{@]
// \def ASMJIT_CC_CODEGEAR
// Defined if the detected C++ compiler is CodeGear or Borland (defined to __CODEGEARC__ or __BORLANDC__ version).
//
// \def ASMJIT_CC_CLANG
// Defined if the detected C++ compiler is CLANG (defined to __clang__ version).
//
// \def ASMJIT_CC_GCC
// Defined if the detected C++ compiler is GCC (defined to __GNUC__ value).
//
// \def ASMJIT_CC_MSC
// Defined if the detected C++ compiler is MSC (defined to _MSC_VER version).
//
// \def ASMJIT_CC_HAS_NATIVE_CHAR
// Defined if the C++ compiler treats char as a native type.
//
// \def ASMJIT_CC_HAS_NATIVE_WCHAR_T
// Defined if the C++ compiler treats wchar_t as a native type.
//
// \def ASMJIT_CC_HAS_NATIVE_CHAR16_T
// Defined if the C++ compiler treats char16_t as a native type.
//
// \def ASMJIT_CC_HAS_NATIVE_CHAR32_T
// Defined if the C++ compiler treats char32_t as a native type.
//
// \def ASMJIT_CC_HAS_OVERRIDE
// Defined if the C++ compiler supports override keyword.
//
// \def ASMJIT_CC_HAS_NOEXCEPT
// Defined if the C++ compiler supports noexcept keyword.
#define ASMJIT_CC_CLANG                        (0)
#define ASMJIT_CC_CODEGEAR                     (0)
#define ASMJIT_CC_GCC                          (0)
#define ASMJIT_CC_MSC                          (0)

#if defined(__BORLANDC__) || defined(__CODEGEARC__)
# undef  ASMJIT_CC_CODEGEAR
# if defined(__CODEGEARC__)
#  define ASMJIT_CC_CODEGEAR                   (__CODEGEARC__)
# else
#  define ASMJIT_CC_CODEGEAR                   (__BORLANDC__)
# endif
# define ASMJIT_CC_HAS_ASSUME                  (0)
# define ASMJIT_CC_HAS_ATTRIBUTE               (0)
# define ASMJIT_CC_HAS_BUILTIN                 (0)
# define ASMJIT_CC_HAS_DECLSPEC                (1)
# define ASMJIT_CC_HAS_DECLSPEC_ALIGN          (ASMJIT_CC_CODEGEAR >= 0x0610)
# define ASMJIT_CC_HAS_DECLSPEC_FORCEINLINE    (0)
# define ASMJIT_CC_HAS_DECLSPEC_NOINLINE       (0)
# define ASMJIT_CC_HAS_DECLSPEC_NORETURN       (ASMJIT_CC_CODEGEAR >= 0x0610)
# define ASMJIT_CC_HAS_DECLTYPE                (ASMJIT_CC_CODEGEAR >= 0x0610)
# define ASMJIT_CC_HAS_NATIVE_CHAR             (1)
# define ASMJIT_CC_HAS_NATIVE_WCHAR_T          (1)
# define ASMJIT_CC_HAS_RVALUE                  (ASMJIT_CC_CODEGEAR >= 0x0610)
# define ASMJIT_CC_HAS_STATIC_ASSERT           (ASMJIT_CC_CODEGEAR >= 0x0610)
#elif defined(__clang__) && defined(__clang_minor__)
# undef  ASMJIT_CC_CLANG
# define ASMJIT_CC_CLANG                       (__clang__)
# define ASMJIT_CC_CLANG_VERSION_EQ(x, y, z)   (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__ >= x * 10000 + y * 100 + z)
# define ASMJIT_CC_CLANG_VERSION_GE(x, y, z)   (__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__ >= x * 10000 + y * 100 + z)
# define ASMJIT_CC_HAS_ASSUME                  (0)
# define ASMJIT_CC_HAS_ATTRIBUTE               (1)
# define ASMJIT_CC_HAS_ATTRIBUTE_ALIGNED       (__has_attribute(__aligned__))
# define ASMJIT_CC_HAS_ATTRIBUTE_ALWAYS_INLINE (__has_attribute(__always_inline__))
# define ASMJIT_CC_HAS_ATTRIBUTE_NOINLINE      (__has_attribute(__noinline__))
# define ASMJIT_CC_HAS_ATTRIBUTE_NORETURN      (__has_attribute(__noreturn__))
# define ASMJIT_CC_HAS_BUILTIN                 (1)
# define ASMJIT_CC_HAS_BUILTIN_ASSUME          (__has_builtin(__builtin_assume))
# define ASMJIT_CC_HAS_BUILTIN_EXPECT          (__has_builtin(__builtin_expect))
# define ASMJIT_CC_HAS_BUILTIN_UNREACHABLE     (__has_builtin(__builtin_unreachable))
# define ASMJIT_CC_HAS_CONSTEXPR               (__has_extension(__cxx_constexpr__))
# define ASMJIT_CC_HAS_DECLSPEC                (0)
# define ASMJIT_CC_HAS_DECLTYPE                (__has_extension(__cxx_decltype__))
# define ASMJIT_CC_HAS_DEFAULT_FUNCTION        (__has_extension(__cxx_defaulted_functions__))
# define ASMJIT_CC_HAS_DELETE_FUNCTION         (__has_extension(__cxx_deleted_functions__))
# define ASMJIT_CC_HAS_FINAL                   (__has_extension(__cxx_override_control__))
# define ASMJIT_CC_HAS_INITIALIZER_LIST        (__has_extension(__cxx_generalized_initializers__))
# define ASMJIT_CC_HAS_LAMBDA                  (__has_extension(__cxx_lambdas__))
# define ASMJIT_CC_HAS_NATIVE_CHAR             (1)
# define ASMJIT_CC_HAS_NATIVE_CHAR16_T         (__has_extension(__cxx_unicode_literals__))
# define ASMJIT_CC_HAS_NATIVE_CHAR32_T         (__has_extension(__cxx_unicode_literals__))
# define ASMJIT_CC_HAS_NATIVE_WCHAR_T          (1)
# define ASMJIT_CC_HAS_NOEXCEPT                (__has_extension(__cxx_noexcept__))
# define ASMJIT_CC_HAS_NULLPTR                 (__has_extension(__cxx_nullptr__))
# define ASMJIT_CC_HAS_OVERRIDE                (__has_extension(__cxx_override_control__))
# define ASMJIT_CC_HAS_RVALUE                  (__has_extension(__cxx_rvalue_references__))
# define ASMJIT_CC_HAS_STATIC_ASSERT           (__has_extension(__cxx_static_assert__))
#elif defined(__GNUC__) && defined(__GNUC_MINOR__)
# undef  ASMJIT_CC_GCC
# define ASMJIT_CC_GCC                         (__GNUC__)
# define ASMJIT_CC_GCC_VERSION_EQ(x, y, z)     (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ >= x * 10000 + y * 100 + z)
# define ASMJIT_CC_GCC_VERSION_GE(x, y, z)     (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ >= x * 10000 + y * 100 + z)
# if defined(__GXX_EXPERIMENTAL_CXX0X__)
#  define ASMJIT_CC_GCC_CXX0X                  (1)
# else
#  define ASMJIT_CC_GCC_CXX0X                  (0)
# endif
# define ASMJIT_CC_HAS_ASSUME                  (0)
# define ASMJIT_CC_HAS_ATTRIBUTE               (1)
# define ASMJIT_CC_HAS_ATTRIBUTE_ALIGNED       (ASMJIT_CC_GCC_VERSION_GE(2, 7, 0))
# define ASMJIT_CC_HAS_ATTRIBUTE_ALWAYS_INLINE (ASMJIT_CC_GCC_VERSION_GE(4, 4, 0) && !defined(__MINGW32__))
# define ASMJIT_CC_HAS_ATTRIBUTE_NOINLINE      (ASMJIT_CC_GCC_VERSION_GE(3, 4, 0) && !defined(__MINGW32__))
# define ASMJIT_CC_HAS_ATTRIBUTE_NORETURN      (ASMJIT_CC_GCC_VERSION_GE(2, 5, 0))
# define ASMJIT_CC_HAS_BUILTIN                 (1)
# define ASMJIT_CC_HAS_BUILTIN_ASSUME          (0)
# define ASMJIT_CC_HAS_BUILTIN_EXPECT          (1)
# define ASMJIT_CC_HAS_BUILTIN_UNREACHABLE     (ASMJIT_CC_GCC_VERSION_GE(4, 5, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_CONSTEXPR               (ASMJIT_CC_GCC_VERSION_GE(4, 6, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_DECLSPEC                (0)
# define ASMJIT_CC_HAS_DECLTYPE                (ASMJIT_CC_GCC_VERSION_GE(4, 3, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_DEFAULT_FUNCTION        (ASMJIT_CC_GCC_VERSION_GE(4, 4, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_DELETE_FUNCTION         (ASMJIT_CC_GCC_VERSION_GE(4, 4, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_FINAL                   (ASMJIT_CC_GCC_VERSION_GE(4, 7, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_INITIALIZER_LIST        (ASMJIT_CC_GCC_VERSION_GE(4, 4, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_LAMBDA                  (ASMJIT_CC_GCC_VERSION_GE(4, 5, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_NATIVE_CHAR             (1)
# define ASMJIT_CC_HAS_NATIVE_CHAR16_T         (ASMJIT_CC_GCC_VERSION_GE(4, 5, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_NATIVE_CHAR32_T         (ASMJIT_CC_GCC_VERSION_GE(4, 5, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_NATIVE_WCHAR_T          (1)
# define ASMJIT_CC_HAS_NOEXCEPT                (ASMJIT_CC_GCC_VERSION_GE(4, 6, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_NULLPTR                 (ASMJIT_CC_GCC_VERSION_GE(4, 6, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_OVERRIDE                (ASMJIT_CC_GCC_VERSION_GE(4, 7, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_RVALUE                  (ASMJIT_CC_GCC_VERSION_GE(4, 3, 0) && ASMJIT_CC_GCC_CXX0X)
# define ASMJIT_CC_HAS_STATIC_ASSERT           (ASMJIT_CC_GCC_VERSION_GE(4, 3, 0) && ASMJIT_CC_GCC_CXX0X)
#elif defined(_MSC_VER)
# undef  ASMJIT_CC_MSC
# define ASMJIT_CC_MSC                         (_MSC_VER)
# define ASMJIT_CC_HAS_ASSUME                  (1)
# define ASMJIT_CC_HAS_ATTRIBUTE               (0)
# define ASMJIT_CC_HAS_BUILTIN                 (0)
# define ASMJIT_CC_HAS_CONSTEXPR               (0)
# define ASMJIT_CC_HAS_DECLSPEC                (1)
# define ASMJIT_CC_HAS_DECLSPEC_ALIGN          (1)
# define ASMJIT_CC_HAS_DECLSPEC_FORCEINLINE    (1)
# define ASMJIT_CC_HAS_DECLSPEC_NOINLINE       (1)
# define ASMJIT_CC_HAS_DECLSPEC_NORETURN       (1)
# define ASMJIT_CC_HAS_DECLTYPE                (_MSC_VER >= 1600)
# define ASMJIT_CC_HAS_DEFAULT_FUNCTION        (_MSC_VER >= 1800)
# define ASMJIT_CC_HAS_DELETE_FUNCTION         (_MSC_VER >= 1800)
# define ASMJIT_CC_HAS_FINAL                   (_MSC_VER >= 1400)
# define ASMJIT_CC_HAS_INITIALIZER_LIST        (_MSC_VER >= 1800)
# define ASMJIT_CC_HAS_LAMBDA                  (_MSC_VER >= 1600)
# define ASMJIT_CC_HAS_NATIVE_CHAR             (1)
# define ASMJIT_CC_HAS_NATIVE_CHAR16_T         (0)
# define ASMJIT_CC_HAS_NATIVE_CHAR32_T         (0)
# if defined(_NATIVE_WCHAR_T_DEFINED)
#  define ASMJIT_CC_HAS_NATIVE_WCHAR_T         (1)
# else
#  define ASMJIT_CC_HAS_NATIVE_WCHAR_T         (0)
# endif
# define ASMJIT_CC_HAS_NOEXCEPT                (_MSC_FULL_VER >= 180021114)
# define ASMJIT_CC_HAS_NULLPTR                 (_MSC_VER >= 1600)
# define ASMJIT_CC_HAS_OVERRIDE                (_MSC_VER >= 1400)
# define ASMJIT_CC_HAS_RVALUE                  (_MSC_VER >= 1600)
# define ASMJIT_CC_HAS_STATIC_ASSERT           (_MSC_VER >= 1600)
#else
# error "[asmjit] Unable to detect the C/C++ compiler."
#endif

#if !defined(ASMJIT_CC_GCC_VERSION_EQ)
# define ASMJIT_CC_GCC_VERSION_EQ(x, y, z)     (0)
# define ASMJIT_CC_GCC_VERSION_GE(x, y, z)     (0)
#endif

#if !defined(ASMJIT_CC_CLANG_VERSION_EQ)
# define ASMJIT_CC_CLANG_VERSION_EQ(x, y, z)   (0)
# define ASMJIT_CC_CLANG_VERSION_GE(x, y, z)   (0)
#endif

#if !ASMJIT_CC_HAS_ATTRIBUTE
# define ASMJIT_CC_HAS_ATTRIBUTE_ALIGNED       (0)
# define ASMJIT_CC_HAS_ATTRIBUTE_ALWAYS_INLINE (0)
# define ASMJIT_CC_HAS_ATTRIBUTE_NOINLINE      (0)
# define ASMJIT_CC_HAS_ATTRIBUTE_NORETURN      (0)
#endif

#if !ASMJIT_CC_HAS_BUILTIN
# define ASMJIT_CC_HAS_BUILTIN_ASSUME          (0)
# define ASMJIT_CC_HAS_BUILTIN_EXPECT          (0)
# define ASMJIT_CC_HAS_BUILTIN_UNREACHABLE     (0)
#endif

#if !ASMJIT_CC_HAS_DECLSPEC
# define ASMJIT_CC_HAS_DECLSPEC_ALIGN          (0)
# define ASMJIT_CC_HAS_DECLSPEC_FORCEINLINE    (0)
# define ASMJIT_CC_HAS_DECLSPEC_NOINLINE       (0)
# define ASMJIT_CC_HAS_DECLSPEC_NORETURN       (0)
#endif
// [@CC}@]

// [@CC_API{@]
// \def ASMJIT_API
// The decorated function is asmjit API and should be exported.
#if !defined(ASMJIT_API)
# if defined(ASMJIT_STATIC)
#  define ASMJIT_API
# elif ASMJIT_OS_WINDOWS
#  if (ASMJIT_CC_GCC || ASMJIT_CC_CLANG) && !defined(__MINGW32__)
#   if defined(ASMJIT_EXPORTS)
#    define ASMJIT_API __attribute__((__dllexport__))
#   else
#    define ASMJIT_API __attribute__((__dllimport__))
#   endif
#  else
#   if defined(ASMJIT_EXPORTS)
#    define ASMJIT_API __declspec(dllexport)
#   else
#    define ASMJIT_API __declspec(dllimport)
#   endif
#  endif
# else
#  if ASMJIT_CC_CLANG || ASMJIT_CC_GCC_VERSION_GE(4, 0, 0)
#   define ASMJIT_API __attribute__((__visibility__("default")))
#  endif
# endif
#endif
// [@CC_API}@]

// [@CC_VARAPI{@]
// \def ASMJIT_VARAPI
// The decorated variable is part of asmjit API and is exported.
#if !defined(ASMJIT_VARAPI)
# define ASMJIT_VARAPI extern ASMJIT_API
#endif
// [@CC_VARAPI}@]

// [@CC_VIRTAPI{@]
// \def ASMJIT_VIRTAPI
// The decorated class has a virtual table and is part of asmjit API.
//
// This is basically a workaround. When using MSVC and marking class as DLL
// export everything gets exported, which is unwanted in most projects. MSVC
// automatically exports typeinfo and vtable if at least one symbol of the
// class is exported. However, GCC has some strange behavior that even if
// one or more symbol is exported it doesn't export typeinfo unless the
// class itself is decorated with "visibility(default)" (i.e. asmjit_API).
#if (ASMJIT_CC_GCC || ASMJIT_CC_CLANG) && !ASMJIT_OS_WINDOWS
# define ASMJIT_VIRTAPI ASMJIT_API
#else
# define ASMJIT_VIRTAPI
#endif
// [@CC_VIRTAPI}@]

// [@CC_INLINE{@]
// \def ASMJIT_INLINE
// Always inline the decorated function.
#if ASMJIT_CC_HAS_ATTRIBUTE_ALWAYS_INLINE && ASMJIT_CC_CLANG
# define ASMJIT_INLINE inline __attribute__((__always_inline__, __visibility__("hidden")))
#elif ASMJIT_CC_HAS_ATTRIBUTE_ALWAYS_INLINE
# define ASMJIT_INLINE inline __attribute__((__always_inline__))
#elif ASMJIT_CC_HAS_DECLSPEC_FORCEINLINE
# define ASMJIT_INLINE __forceinline
#else
# define ASMJIT_INLINE inline
#endif
// [@CC_INLINE}@]

// [@CC_CDECL{@]
// \def ASMJIT_CDECL
// Standard C function calling convention decorator (__cdecl).
#if ASMJIT_ARCH_X86
# if ASMJIT_CC_HAS_ATTRIBUTE
#  define ASMJIT_CDECL __attribute__((__cdecl__))
# else
#  define ASMJIT_CDECL __cdecl
# endif
#else
# define ASMJIT_CDECL
#endif
// [@CC_CDECL}@]

// [@CC_STDCALL{@]
// \def ASMJIT_STDCALL
// StdCall function calling convention decorator (__stdcall).
#if ASMJIT_ARCH_X86
# if ASMJIT_CC_HAS_ATTRIBUTE
#  define ASMJIT_STDCALL __attribute__((__stdcall__))
# else
#  define ASMJIT_STDCALL __stdcall
# endif
#else
# define ASMJIT_STDCALL
#endif
// [@CC_STDCALL}@]

// [@CC_FASTCALL{@]
// \def ASMJIT_FASTCALL
// FastCall function calling convention decorator (__fastcall).
#if ASMJIT_ARCH_X86
# if ASMJIT_CC_HAS_ATTRIBUTE
#  define ASMJIT_FASTCALL __attribute__((__fastcall__))
# else
#  define ASMJIT_FASTCALL __fastcall
# endif
#else
# define ASMJIT_FASTCALL
#endif
// [@CC_FASTCALL}@]

// [@CC_REGPARM{@]
// \def ASMJIT_REGPARM(n)
// A custom calling convention which passes n arguments in registers.
#if ASMJIT_ARCH_X86 && (ASMJIT_CC_GCC || ASMJIT_CC_CLANG)
# define ASMJIT_REGPARM(n) __attribute__((__regparm__(n)))
#else
# define ASMJIT_REGPARM(n)
#endif
// [@CC_REGPARM}@]

// [@CC_NOP{@]
// \def ASMJIT_NOP
// No operation.
#if !defined(ASMJIT_NOP)
# define ASMJIT_NOP ((void)0)
#endif
// [@CC_NOP}@]

// [@CC_EXPECT{@]
// \def ASMJIT_LIKELY(exp)
// Expression exp is likely to be true.
//
// \def ASMJIT_UNLIKELY(exp)
// Expression exp is likely to be false.
#if ASMJIT_HAS_BUILTIN_EXPECT
# define ASMJIT_LIKELY(exp) __builtin_expect(!!(exp), 1)
# define ASMJIT_UNLIKELY(exp) __builtin_expect(!!(exp), 0)
#else
# define ASMJIT_LIKELY(exp) exp
# define ASMJIT_UNLIKELY(exp) exp
#endif
// [@CC_EXPECT}@]

// [@CC_UNUSED{@]
// \def ASMJIT_UNUSED(x)
// Mark a variable x as unused.
#define ASMJIT_UNUSED(x) (void)(x)
// [@CC_UNUSED}@]

// [@CC_OFFSET_OF{@]
// \def ASMJIT_OFFSET_OF(x, y).
// Get the offset of a member y of a struct x at compile-time.
#define ASMJIT_OFFSET_OF(x, y) ((int)(intptr_t)((const char*)&((const x*)0x1)->y) - 1)
// [@CC_OFFSET_OF}@]

// [@CC_ARRAY_SIZE{@]
// \def ASMJIT_ARRAY_SIZE(x)
// Get the array size of x at compile-time.
#define ASMJIT_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
// [@CC_ARRAY_SIZE}@]

// [@STDTYPES{@]
#if defined(__MINGW32__) || defined(__MINGW64__)
# include <sys/types.h>
#endif
#if defined(_MSC_VER) && (_MSC_VER < 1600)
# include <limits.h>
# if !defined(ASMJIT_SUPPRESS_STD_TYPES)
#  if (_MSC_VER < 1300)
typedef signed char      int8_t;
typedef signed short     int16_t;
typedef signed int       int32_t;
typedef signed __int64   int64_t;
typedef unsigned char    uint8_t;
typedef unsigned short   uint16_t;
typedef unsigned int     uint32_t;
typedef unsigned __int64 uint64_t;
#  else
typedef __int8           int8_t;
typedef __int16          int16_t;
typedef __int32          int32_t;
typedef __int64          int64_t;
typedef unsigned __int8  uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#  endif
# endif
# define ASMJIT_INT64_C(x) (x##i64)
# define ASMJIT_UINT64_C(x) (x##ui64)
#else
# include <stdint.h>
# include <limits.h>
# define ASMJIT_INT64_C(x) (x##ll)
# define ASMJIT_UINT64_C(x) (x##ull)
#endif
// [@STDTYPES}@]

// ============================================================================
// [asmjit::Build - Additional]
// ============================================================================

// Build host architecture if no architecture is selected.
#if !defined(ASMJIT_BUILD_HOST) && \
    !defined(ASMJIT_BUILD_X86) && \
    !defined(ASMJIT_BUILD_X64)
# define ASMJIT_BUILD_HOST
#endif

// Autodetect host architecture if enabled.
#if defined(ASMJIT_BUILD_HOST)
# if ASMJIT_ARCH_X86 && !defined(ASMJIT_BUILD_X86)
#  define ASMJIT_BUILD_X86
# endif // ASMJIT_ARCH_X86 && !ASMJIT_BUILD_X86
# if ASMJIT_ARCH_X64 && !defined(ASMJIT_BUILD_X64)
#  define ASMJIT_BUILD_X64
# endif // ASMJIT_ARCH_X64 && !ASMJIT_BUILD_X64
#endif // ASMJIT_BUILD_HOST

#if defined(_MSC_VER) && _MSC_VER >= 1400
# define ASMJIT_ENUM(_Name_) enum _Name_ : uint32_t
#else
# define ASMJIT_ENUM(_Name_) enum _Name_
#endif

#if ASMJIT_ARCH_LE
# define _ASMJIT_ARCH_INDEX(_Total_, _Index_) (_Index_)
#else
# define _ASMJIT_ARCH_INDEX(_Total_, _Index_) ((_Total_) - 1 - (_Index_))
#endif

#if !defined(ASMJIT_ALLOC) && !defined(ASMJIT_REALLOC) && !defined(ASMJIT_FREE)
# define ASMJIT_ALLOC(_Size_) ::malloc(_Size_)
# define ASMJIT_REALLOC(_Ptr_, _Size_) ::realloc(_Ptr_, _Size_)
# define ASMJIT_FREE(_Ptr_) ::free(_Ptr_)
#else
# if !defined(ASMJIT_ALLOC) || !defined(ASMJIT_REALLOC) || !defined(ASMJIT_FREE)
#  error "[asmjit] You must redefine ASMJIT_ALLOC, ASMJIT_REALLOC and ASMJIT_FREE."
# endif
#endif // !ASMJIT_ALLOC && !ASMJIT_REALLOC && !ASMJIT_FREE

#define ASMJIT_NO_COPY(_Type_) \
private: \
  ASMJIT_INLINE _Type_(const _Type_& other); \
  ASMJIT_INLINE _Type_& operator=(const _Type_& other); \
public:

// ============================================================================
// [asmjit::Build - Relative Path]
// ============================================================================

namespace asmjit {
namespace DebugUtil {

// Workaround that is used to convert an absolute path to a relative one at
// a C macro level, used by asserts and tracing. This workaround is needed
// as some build systems always convert the source code files to use absolute
// paths. Please note that if absolute paths are used this doesn't remove them
// from the compiled binary and can be still considered a security risk.
enum {
  kSourceRelativePathOffset = int(sizeof(__FILE__) - sizeof("asmjit/build.h"))
};

// ASMJIT_TRACE is only used by sources and private headers. It's safe to make
// it unavailable outside of AsmJit.
#if defined(ASMJIT_EXPORTS)
static inline int disabledTrace(...) { return 0; }
# if defined(ASMJIT_TRACE)
#  define ASMJIT_TSEC(section) section
#  define ASMJIT_TLOG ::printf
# else
#  define ASMJIT_TSEC(section) ASMJIT_NOP
#  define ASMJIT_TLOG 0 && ::asmjit::DebugUtil::disabledTrace
# endif // ASMJIT_TRACE
#endif // ASMJIT_EXPORTS

} // DebugUtil namespace
} // asmjit namespace

// ============================================================================
// [asmjit::Build - Test]
// ============================================================================

// Include a unit testing package if this is a `asmjit_test` build.
#if defined(ASMJIT_TEST)
# include "../test/broken.h"
#endif // ASMJIT_TEST

// [Guard]
#endif // _ASMJIT_BUILD_H
