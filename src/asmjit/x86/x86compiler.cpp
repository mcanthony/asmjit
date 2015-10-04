// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Export]
#define ASMJIT_EXPORTS

// [Guard]
#include "../build.h"
#if !defined(ASMJIT_DISABLE_COMPILER) && (defined(ASMJIT_BUILD_X86) || defined(ASMJIT_BUILD_X64))

// [Dependencies - AsmJit]
#include "../base/string.h"
#include "../base/utils.h"
#include "../x86/x86assembler.h"
#include "../x86/x86compiler.h"
#include "../x86/x86compilercontext_p.h"

// [Api-Begin]
#include "../apibegin.h"

namespace asmjit {

// ============================================================================
// [Debug]
// ============================================================================

#if !defined(ASMJIT_DEBUG)
#define ASMJIT_ASSERT_UNINITIALIZED(op) \
  do {} while(0)
#else
#define ASMJIT_ASSERT_UNINITIALIZED(op) \
  do { \
    if (op.isVar() || op.isLabel()) { \
      ASMJIT_ASSERT(op.getId() != kInvalidValue); \
    } \
  } while(0)
#endif

// ============================================================================
// [asmjit::X86VarInfo]
// ============================================================================

#define C(_Class_) kX86RegClass##_Class_
#define D(_Desc_) kVarFlag##_Desc_

const X86VarInfo _x86VarInfo[] = {
  /* 00: kVarTypeInt8     */ { kX86RegTypeGpbLo, 1 , C(Gp) , 0                , "gpb" },
  /* 01: kVarTypeUInt8    */ { kX86RegTypeGpbLo, 1 , C(Gp) , 0                , "gpb" },
  /* 02: kVarTypeInt16    */ { kX86RegTypeGpw  , 2 , C(Gp) , 0                , "gpw" },
  /* 03: kVarTypeUInt16   */ { kX86RegTypeGpw  , 2 , C(Gp) , 0                , "gpw" },
  /* 04: kVarTypeInt32    */ { kX86RegTypeGpd  , 4 , C(Gp) , 0                , "gpd" },
  /* 05: kVarTypeUInt32   */ { kX86RegTypeGpd  , 4 , C(Gp) , 0                , "gpd" },
  /* 06: kVarTypeInt64    */ { kX86RegTypeGpq  , 8 , C(Gp) , 0                , "gpq" },
  /* 07: kVarTypeUInt64   */ { kX86RegTypeGpq  , 8 , C(Gp) , 0                , "gpq" },
  /* 08: kVarTypeIntPtr   */ { 0               , 0 , C(Gp) , 0                , ""    }, // Remapped.
  /* 09: kVarTypeUIntPtr  */ { 0               , 0 , C(Gp) , 0                , ""    }, // Remapped.
  /* 10: kVarTypeFp32     */ { kX86RegTypeFp   , 4 , C(Fp) , D(Sp)            , "fp"  },
  /* 11: kVarTypeFp64     */ { kX86RegTypeFp   , 8 , C(Fp) , D(Dp)            , "fp"  },
  /* 12: kX86VarTypeMm    */ { kX86RegTypeMm   , 8 , C(Mm) , 0                , "mm"  },
  /* 13: kX86VarTypeK     */ { kX86RegTypeK    , 8 , C(K)  , 0                , "k"   },
  /* 14: kX86VarTypeXmm   */ { kX86RegTypeXmm  , 16, C(Xyz), 0                , "xmm" },
  /* 15: kX86VarTypeXmmSs */ { kX86RegTypeXmm  , 4 , C(Xyz), D(Sp)            , "xmm" },
  /* 16: kX86VarTypeXmmPs */ { kX86RegTypeXmm  , 16, C(Xyz), D(Sp) | D(Packed), "xmm" },
  /* 17: kX86VarTypeXmmSd */ { kX86RegTypeXmm  , 8 , C(Xyz), D(Dp)            , "xmm" },
  /* 18: kX86VarTypeXmmPd */ { kX86RegTypeXmm  , 16, C(Xyz), D(Dp) | D(Packed), "xmm" },
  /* 19: kX86VarTypeYmm   */ { kX86RegTypeYmm  , 32, C(Xyz), 0                , "ymm" },
  /* 20: kX86VarTypeYmmPs */ { kX86RegTypeYmm  , 32, C(Xyz), D(Sp) | D(Packed), "ymm" },
  /* 21: kX86VarTypeYmmPd */ { kX86RegTypeYmm  , 32, C(Xyz), D(Dp) | D(Packed), "ymm" },
  /* 22: kX86VarTypeZmm   */ { kX86RegTypeZmm  , 64, C(Xyz), 0                , "zmm" },
  /* 23: kX86VarTypeZmmPs */ { kX86RegTypeZmm  , 64, C(Xyz), D(Sp) | D(Packed), "zmm" },
  /* 24: kX86VarTypeZmmPd */ { kX86RegTypeZmm  , 64, C(Xyz), D(Dp) | D(Packed), "zmm" }
};

#undef D
#undef C

#if defined(ASMJIT_BUILD_X86)
const uint8_t _x86VarMapping[kX86VarTypeCount] = {
  /* 00: kVarTypeInt8     */ kVarTypeInt8,
  /* 01: kVarTypeUInt8    */ kVarTypeUInt8,
  /* 02: kVarTypeInt16    */ kVarTypeInt16,
  /* 03: kVarTypeUInt16   */ kVarTypeUInt16,
  /* 04: kVarTypeInt32    */ kVarTypeInt32,
  /* 05: kVarTypeUInt32   */ kVarTypeUInt32,
  /* 06: kVarTypeInt64    */ kInvalidVar,     // Invalid in 32-bit mode.
  /* 07: kVarTypeUInt64   */ kInvalidVar,     // Invalid in 32-bit mode.
  /* 08: kVarTypeIntPtr   */ kVarTypeInt32,   // Remapped to Int32.
  /* 09: kVarTypeUIntPtr  */ kVarTypeUInt32,  // Remapped to UInt32.
  /* 10: kVarTypeFp32     */ kVarTypeFp32,
  /* 11: kVarTypeFp64     */ kVarTypeFp64,
  /* 12: kX86VarTypeMm    */ kX86VarTypeMm,
  /* 13: kX86VarTypeK     */ kX86VarTypeK,
  /* 14: kX86VarTypeXmm   */ kX86VarTypeXmm,
  /* 15: kX86VarTypeXmmSs */ kX86VarTypeXmmSs,
  /* 16: kX86VarTypeXmmPs */ kX86VarTypeXmmPs,
  /* 17: kX86VarTypeXmmSd */ kX86VarTypeXmmSd,
  /* 18: kX86VarTypeXmmPd */ kX86VarTypeXmmPd,
  /* 19: kX86VarTypeYmm   */ kX86VarTypeYmm,
  /* 20: kX86VarTypeYmmPs */ kX86VarTypeYmmPs,
  /* 21: kX86VarTypeYmmPd */ kX86VarTypeYmmPd,
  /* 22: kX86VarTypeZmm   */ kX86VarTypeZmm,
  /* 23: kX86VarTypeZmmPs */ kX86VarTypeZmmPs,
  /* 24: kX86VarTypeZmmPd */ kX86VarTypeZmmPd
};
#endif // ASMJIT_BUILD_X86

#if defined(ASMJIT_BUILD_X64)
const uint8_t _x64VarMapping[kX86VarTypeCount] = {
  /* 00: kVarTypeInt8     */ kVarTypeInt8,
  /* 01: kVarTypeUInt8    */ kVarTypeUInt8,
  /* 02: kVarTypeInt16    */ kVarTypeInt16,
  /* 03: kVarTypeUInt16   */ kVarTypeUInt16,
  /* 04: kVarTypeInt32    */ kVarTypeInt32,
  /* 05: kVarTypeUInt32   */ kVarTypeUInt32,
  /* 06: kVarTypeInt64    */ kVarTypeInt64,
  /* 07: kVarTypeUInt64   */ kVarTypeUInt64,
  /* 08: kVarTypeIntPtr   */ kVarTypeInt64,   // Remapped to Int64.
  /* 09: kVarTypeUIntPtr  */ kVarTypeUInt64,  // Remapped to UInt64.
  /* 10: kVarTypeFp32     */ kVarTypeFp32,
  /* 11: kVarTypeFp64     */ kVarTypeFp64,
  /* 12: kX86VarTypeMm    */ kX86VarTypeMm,
  /* 13: kX86VarTypeK     */ kX86VarTypeK,
  /* 14: kX86VarTypeXmm   */ kX86VarTypeXmm,
  /* 15: kX86VarTypeXmmSs */ kX86VarTypeXmmSs,
  /* 16: kX86VarTypeXmmPs */ kX86VarTypeXmmPs,
  /* 17: kX86VarTypeXmmSd */ kX86VarTypeXmmSd,
  /* 18: kX86VarTypeXmmPd */ kX86VarTypeXmmPd,
  /* 19: kX86VarTypeYmm   */ kX86VarTypeYmm,
  /* 20: kX86VarTypeYmmPs */ kX86VarTypeYmmPs,
  /* 21: kX86VarTypeYmmPd */ kX86VarTypeYmmPd,
  /* 22: kX86VarTypeZmm   */ kX86VarTypeZmm,
  /* 23: kX86VarTypeZmmPs */ kX86VarTypeZmmPs,
  /* 24: kX86VarTypeZmmPd */ kX86VarTypeZmmPd
};
#endif // ASMJIT_BUILD_X64

// ============================================================================
// [asmjit::X86CallNode - Arg / Ret]
// ============================================================================

bool X86CallNode::_setArg(uint32_t i, const Operand& op) {
  if ((i & ~kFuncArgHi) >= _x86Decl.getNumArgs())
    return false;

  _args[i] = op;
  return true;
}

bool X86CallNode::_setRet(uint32_t i, const Operand& op) {
  if (i >= 2)
    return false;

  _ret[i] = op;
  return true;
}

// ============================================================================
// [asmjit::X86Compiler - Helpers (Private)]
// ============================================================================

static Error X86Compiler_emitConstPool(X86Compiler* self,
  Label& label, ConstPool& pool) {

  if (label.getId() == kInvalidValue)
    return kErrorOk;

  self->align(kAlignData, static_cast<uint32_t>(pool.getAlignment()));
  self->bind(label);

  ASData* embedNode = self->embed(NULL, static_cast<uint32_t>(pool.getSize()));
  if (embedNode == NULL)
    return kErrorNoHeapMemory;

  pool.fill(embedNode->getData());
  pool.reset();
  label.reset();

  return kErrorOk;
}

// ============================================================================
// [asmjit::X86Compiler - Construction / Destruction]
// ============================================================================

X86Compiler::X86Compiler(Runtime* runtime, uint32_t arch) :
  Compiler(runtime),
  zax(NoInit),
  zcx(NoInit),
  zdx(NoInit),
  zbx(NoInit),
  zsp(NoInit),
  zbp(NoInit),
  zsi(NoInit),
  zdi(NoInit) {

  setArch(arch);
}

X86Compiler::~X86Compiler() {}

// ============================================================================
// [asmjit::X86Compiler - Arch]
// ============================================================================

Error X86Compiler::setArch(uint32_t arch) {
#if defined(ASMJIT_BUILD_X86)
  if (arch == kArchX86) {
    _arch = kArchX86;
    _regSize = 4;

    _regCount.reset();
    _regCount._gp  = 8;
    _regCount._mm  = 8;
    _regCount._k   = 8;
    _regCount._xyz = 8;

    zax = x86::eax;
    zcx = x86::ecx;
    zdx = x86::edx;
    zbx = x86::ebx;
    zsp = x86::esp;
    zbp = x86::ebp;
    zsi = x86::esi;
    zdi = x86::edi;

    _targetVarMapping = _x86VarMapping;
    return kErrorOk;
  }
#endif // ASMJIT_BUILD_X86

#if defined(ASMJIT_BUILD_X64)
  if (arch == kArchX64) {
    _arch = kArchX64;
    _regSize = 8;

    _regCount.reset();
    _regCount._gp  = 16;
    _regCount._mm  = 8;
    _regCount._k   = 8;
    _regCount._xyz = 16;

    zax = x86::rax;
    zcx = x86::rcx;
    zdx = x86::rdx;
    zbx = x86::rbx;
    zsp = x86::rsp;
    zbp = x86::rbp;
    zsi = x86::rsi;
    zdi = x86::rdi;

    _targetVarMapping = _x64VarMapping;
    return kErrorOk;
  }
#endif // ASMJIT_BUILD_X64

  ASMJIT_ASSERT(!"Reached");
  return kErrorInvalidArgument;
}

// ============================================================================
// [asmjit::X86Compiler - Inst]
// ============================================================================

//! Get compiler instruction item size without operands assigned.
static ASMJIT_INLINE size_t X86Compiler_getInstSize(uint32_t code) {
  return Utils::inInterval<uint32_t>(code, _kX86InstIdJbegin, _kX86InstIdJend) ? sizeof(ASJump) : sizeof(ASInst);
}

static ASInst* X86Compiler_newInst(X86Compiler* self, void* p, uint32_t code, uint32_t options, Operand* opList, uint32_t opCount) {
  if (Utils::inInterval<uint32_t>(code, _kX86InstIdJbegin, _kX86InstIdJend)) {
    ASJump* node = new(p) ASJump(self, code, options, opList, opCount);
    ASLabel* jTarget = NULL;

    if ((options & kInstOptionUnfollow) == 0) {
      if (opList[0].isLabel())
        jTarget = self->getASLabel(static_cast<Label&>(opList[0]));
      else
        options |= kInstOptionUnfollow;
    }

    node->orFlags(code == kX86InstIdJmp ? kASNodeFlagIsJmp | kASNodeFlagIsTaken : kASNodeFlagIsJcc);
    node->_target = jTarget;
    node->_jumpNext = NULL;

    if (jTarget) {
      node->_jumpNext = static_cast<ASJump*>(jTarget->_from);
      jTarget->_from = node;
      jTarget->addNumRefs();
    }

    // The 'jmp' is always taken, conditional jump can contain hint, we detect it.
    if (code == kX86InstIdJmp)
      node->orFlags(kASNodeFlagIsTaken);
    else if (options & kInstOptionTaken)
      node->orFlags(kASNodeFlagIsTaken);

    node->addOptions(options);
    return node;
  }
  else {
    ASInst* node = new(p) ASInst(self, code, options, opList, opCount);
    node->addOptions(options);
    return node;
  }
}

ASInst* X86Compiler::newInst(uint32_t code) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size));

  if (inst == NULL)
    goto _NoMemory;

  return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), NULL, 0);

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::newInst(uint32_t code, const Operand& o0) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size + 1 * sizeof(Operand)));

  if (inst == NULL)
    goto _NoMemory;

  {
    Operand* opList = reinterpret_cast<Operand*>(reinterpret_cast<uint8_t*>(inst) + size);
    opList[0] = o0;
    ASMJIT_ASSERT_UNINITIALIZED(o0);
    return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), opList, 1);
  }

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::newInst(uint32_t code, const Operand& o0, const Operand& o1) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size + 2 * sizeof(Operand)));

  if (inst == NULL)
    goto _NoMemory;

  {
    Operand* opList = reinterpret_cast<Operand*>(reinterpret_cast<uint8_t*>(inst) + size);
    opList[0] = o0;
    opList[1] = o1;
    ASMJIT_ASSERT_UNINITIALIZED(o0);
    ASMJIT_ASSERT_UNINITIALIZED(o1);
    return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), opList, 2);
  }

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::newInst(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size + 3 * sizeof(Operand)));

  if (inst == NULL)
    goto _NoMemory;

  {
    Operand* opList = reinterpret_cast<Operand*>(reinterpret_cast<uint8_t*>(inst) + size);
    opList[0] = o0;
    opList[1] = o1;
    opList[2] = o2;
    ASMJIT_ASSERT_UNINITIALIZED(o0);
    ASMJIT_ASSERT_UNINITIALIZED(o1);
    ASMJIT_ASSERT_UNINITIALIZED(o2);
    return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), opList, 3);
  }

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::newInst(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, const Operand& o3) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size + 4 * sizeof(Operand)));

  if (inst == NULL)
    goto _NoMemory;

  {
    Operand* opList = reinterpret_cast<Operand*>(reinterpret_cast<uint8_t*>(inst) + size);
    opList[0] = o0;
    opList[1] = o1;
    opList[2] = o2;
    opList[3] = o3;
    ASMJIT_ASSERT_UNINITIALIZED(o0);
    ASMJIT_ASSERT_UNINITIALIZED(o1);
    ASMJIT_ASSERT_UNINITIALIZED(o2);
    ASMJIT_ASSERT_UNINITIALIZED(o3);
    return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), opList, 4);
  }

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::newInst(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, const Operand& o3, const Operand& o4) {
  size_t size = X86Compiler_getInstSize(code);
  ASInst* inst = static_cast<ASInst*>(_baseZone.alloc(size + 5 * sizeof(Operand)));

  if (inst == NULL)
    goto _NoMemory;

  {
    Operand* opList = reinterpret_cast<Operand*>(reinterpret_cast<uint8_t*>(inst) + size);
    opList[0] = o0;
    opList[1] = o1;
    opList[2] = o2;
    opList[3] = o3;
    opList[4] = o4;
    ASMJIT_ASSERT_UNINITIALIZED(o0);
    ASMJIT_ASSERT_UNINITIALIZED(o1);
    ASMJIT_ASSERT_UNINITIALIZED(o2);
    ASMJIT_ASSERT_UNINITIALIZED(o3);
    ASMJIT_ASSERT_UNINITIALIZED(o4);
    return X86Compiler_newInst(this, inst, code, getInstOptionsAndReset(), opList, 5);
  }

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASInst* X86Compiler::emit(uint32_t code) {
  ASInst* node = newInst(code);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0) {
  ASInst* node = newInst(code, o0);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1){
  ASInst* node = newInst(code, o0, o1);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2) {
  ASInst* node = newInst(code, o0, o1, o2);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, const Operand& o3){
  ASInst* node = newInst(code, o0, o1, o2, o3);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, const Operand& o3, const Operand& o4) {
  ASInst* node = newInst(code, o0, o1, o2, o3, o4);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, int o0_) {
  Imm o0(o0_);
  ASInst* node = newInst(code, o0);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, uint64_t o0_) {
  Imm o0(o0_);
  ASInst* node = newInst(code, o0);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, int o1_) {
  Imm o1(o1_);
  ASInst* node = newInst(code, o0, o1);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, uint64_t o1_) {
  Imm o1(o1_);
  ASInst* node = newInst(code, o0, o1);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, int o2_) {
  Imm o2(o2_);
  ASInst* node = newInst(code, o0, o1, o2);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, uint64_t o2_) {
  Imm o2(o2_);
  ASInst* node = newInst(code, o0, o1, o2);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, int o3_) {
  Imm o3(o3_);
  ASInst* node = newInst(code, o0, o1, o2, o3);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

ASInst* X86Compiler::emit(uint32_t code, const Operand& o0, const Operand& o1, const Operand& o2, uint64_t o3_) {
  Imm o3(o3_);
  ASInst* node = newInst(code, o0, o1, o2, o3);
  if (node == NULL)
    return NULL;
  return static_cast<ASInst*>(addNode(node));
}

// ============================================================================
// [asmjit::X86Compiler - Func]
// ============================================================================

X86FuncNode* X86Compiler::newFunc(const FuncPrototype& p) {
  X86FuncNode* func = newNode<X86FuncNode>();
  Error error;

  if (func == NULL)
    goto _NoMemory;

  // Create helper nodes.
  func->_entryNode = newTarget();
  func->_exitNode = newTarget();
  func->_end = newNode<ASSentinel>();

  if (func->_entryNode == NULL || func->_exitNode == NULL || func->_end == NULL)
    goto _NoMemory;

  // Function prototype.
  if ((error = func->_x86Decl.setPrototype(p)) != kErrorOk) {
    setError(error);
    return NULL;
  }

  // Function arguments stack size. Since function requires _argStackSize to be
  // set, we have to copy it from X86FuncDecl.
  func->_argStackSize = func->_x86Decl.getArgStackSize();
  func->_redZoneSize = static_cast<uint16_t>(func->_x86Decl.getRedZoneSize());
  func->_spillZoneSize = static_cast<uint16_t>(func->_x86Decl.getSpillZoneSize());

  // Expected/Required stack alignment.
  func->_expectedStackAlignment = getRuntime()->getStackAlignment();
  func->_requiredStackAlignment = 0;

  // Allocate space for function arguments.
  func->_args = NULL;
  if (func->getNumArgs() != 0) {
    func->_args = _baseZone.allocT<VarData*>(func->getNumArgs() * sizeof(VarData*));
    if (func->_args == NULL)
      goto _NoMemory;
    ::memset(func->_args, 0, func->getNumArgs() * sizeof(VarData*));
  }

  return func;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

X86FuncNode* X86Compiler::addFunc(const FuncPrototype& p) {
  X86FuncNode* func = newFunc(p);

  if (func == NULL) {
    setError(kErrorNoHeapMemory);
    return NULL;
  }

  ASMJIT_ASSERT(_func == NULL);
  _func = func;

  addNode(func);                 // Add function node.
  addNode(func->getEntryNode()); // Add function entry.
  ASNode* cursor = getCursor();

  addNode(func->getExitNode());  // Add function exit / epilog marker.
  addNode(func->getEnd());       // Add function end.
  setCursor(cursor);

  return func;
}

ASSentinel* X86Compiler::endFunc() {
  X86FuncNode* func = getFunc();
  ASMJIT_ASSERT(func != NULL);

  // Add local constant pool at the end of the function (if exist).
  setCursor(func->getExitNode());
  X86Compiler_emitConstPool(this, _localConstPoolLabel, _localConstPool);

  // Finalize...
  func->addFuncFlags(kFuncFlagIsFinished);
  _func = NULL;

  setCursor(func->getEnd());
  return func->getEnd();
}

// ============================================================================
// [asmjit::X86Compiler - Ret]
// ============================================================================

ASRet* X86Compiler::newRet(const Operand& o0, const Operand& o1) {
  ASRet* node = newNode<ASRet>(o0, o1);
  if (node == NULL)
    goto _NoMemory;
  return node;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASRet* X86Compiler::addRet(const Operand& o0, const Operand& o1) {
  ASRet* node = newRet(o0, o1);
  if (node == NULL)
    return node;
  return static_cast<ASRet*>(addNode(node));
}

// ============================================================================
// [asmjit::X86Compiler - Call]
// ============================================================================

X86CallNode* X86Compiler::newCall(const Operand& o0, const FuncPrototype& p) {
  X86CallNode* node = newNode<X86CallNode>(o0);
  Error error;
  uint32_t nArgs;

  if (node == NULL)
    goto _NoMemory;

  if ((error = node->_x86Decl.setPrototype(p)) != kErrorOk) {
    setError(error);
    return NULL;
  }

  // If there are no arguments skip the allocation.
  if ((nArgs = p.getNumArgs()) == 0)
    return node;

  node->_args = static_cast<Operand*>(_baseZone.alloc(nArgs * sizeof(Operand)));
  if (node->_args == NULL)
    goto _NoMemory;

  ::memset(node->_args, 0, nArgs * sizeof(Operand));
  return node;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

X86CallNode* X86Compiler::addCall(const Operand& o0, const FuncPrototype& p) {
  X86CallNode* node = newCall(o0, p);
  if (node == NULL)
    return NULL;
  return static_cast<X86CallNode*>(addNode(node));
}

// ============================================================================
// [asmjit::X86Compiler - Vars]
// ============================================================================

Error X86Compiler::setArg(uint32_t argIndex, Var& var) {
  X86FuncNode* func = getFunc();

  if (func == NULL)
    return kErrorInvalidArgument;

  if (!isVarValid(var))
    return kErrorInvalidState;

  VarData* vd = getVd(var);
  func->setArg(argIndex, vd);

  return kErrorOk;
}

Error X86Compiler::_newVar(Var* var, uint32_t vType, const char* name) {
  ASMJIT_ASSERT(vType < kX86VarTypeCount);

  vType = _targetVarMapping[vType];
  ASMJIT_ASSERT(vType != kInvalidVar);

  // There is not ASSERT in release mode and this should be checked.
  if (vType == kInvalidVar) {
    static_cast<X86Var*>(var)->reset();
    return kErrorInvalidArgument;
  }

  const X86VarInfo& vInfo = _x86VarInfo[vType];
  VarData* vd = _newVd(vType, vInfo.getSize(), vInfo.getClass(), name);

  if (vd == NULL) {
    static_cast<X86Var*>(var)->reset();
    return getError();
  }

  var->_init_packed_op_sz_w0_id(kOperandTypeVar, vd->getSize(), vInfo.getReg() << 8, vd->getId());
  var->_vreg.vType = vType;
  return kErrorOk;
}

// ============================================================================
// [asmjit::X86Compiler - Stack]
// ============================================================================

Error X86Compiler::_newStack(BaseMem* mem, uint32_t size, uint32_t alignment, const char* name) {
  if (size == 0)
    return kErrorInvalidArgument;

  if (alignment > 64)
    alignment = 64;

  VarData* vd = _newVd(kInvalidVar, size, kInvalidReg, name);
  if (vd == NULL) {
    static_cast<X86Mem*>(mem)->reset();
    return getError();
  }

  vd->_isStack = true;
  vd->_alignment = static_cast<uint8_t>(alignment);

  static_cast<X86Mem*>(mem)->_init(kMemTypeStackIndex, vd->getId(), 0, 0);
  return kErrorOk;
}

// ============================================================================
// [asmjit::X86Compiler - Const]
// ============================================================================

Error X86Compiler::_newConst(BaseMem* mem, uint32_t scope, const void* data, size_t size) {
  Error error = kErrorOk;
  size_t offset;

  Label* dstLabel;
  ConstPool* dstPool;

  if (scope == kConstScopeLocal) {
    dstLabel = &_localConstPoolLabel;
    dstPool = &_localConstPool;
  }
  else if (scope == kConstScopeGlobal) {
    dstLabel = &_globalConstPoolLabel;
    dstPool = &_globalConstPool;
  }
  else {
    error = kErrorInvalidArgument;
    goto _OnError;
  }

  error = dstPool->add(data, size, offset);
  if (error != kErrorOk)
    goto _OnError;

  if (dstLabel->getId() == kInvalidValue) {
    error = _newLabel(dstLabel);
    if (error != kErrorOk)
      goto _OnError;
  }

  *static_cast<X86Mem*>(mem) = x86::ptr(*dstLabel, static_cast<int32_t>(offset), static_cast<uint32_t>(size));
  return kErrorOk;

_OnError:
  return error;
}

// ============================================================================
// [asmjit::X86Compiler - Make]
// ============================================================================

void* X86Compiler::make() {
  Assembler* assembler = getAssembler();
  if (assembler == NULL) {
    setError(kErrorNoHeapMemory);
    return NULL;
  }

  Error error = serialize(assembler);
  if (error != kErrorOk) {
    setError(error);
    return NULL;
  }

  void* result = assembler->make();
  return result;
}

// ============================================================================
// [asmjit::X86Compiler - Assembler]
// ============================================================================

Assembler* X86Compiler::_newAssembler() {
  return new(std::nothrow) X86Assembler(_runtime, _arch);
}

// ============================================================================
// [asmjit::X86Compiler - Serialize]
// ============================================================================

Error X86Compiler::serialize(Assembler* assembler) {
  // Flush the global constant pool.
  X86Compiler_emitConstPool(this, _globalConstPoolLabel, _globalConstPool);

  if (_firstNode == NULL)
    return kErrorOk;

  X86Context context(this);
  Error error = kErrorOk;

  ASNode* node = _firstNode;
  ASNode* start;

  // Find function and use the context to translate/emit.
  do {
    start = node;
    _resetTokenGenerator();

    if (node->getType() == kASNodeTypeFunc) {
      node = static_cast<X86FuncNode*>(start)->getEnd();
      error = context.compile(static_cast<X86FuncNode*>(start));

      if (error != kErrorOk)
        goto _Error;
    }

    do {
      node = node->getNext();
    } while (node != NULL && node->getType() != kASNodeTypeFunc);

    error = context.serialize(assembler, start, node);
    if (error != kErrorOk)
      goto _Error;

    context.cleanup();
  } while (node != NULL);
  return kErrorOk;

_Error:
  context.cleanup();
  return error;
}

} // asmjit namespace

// [Api-End]
#include "../apiend.h"

// [Guard]
#endif // !ASMJIT_DISABLE_COMPILER && (ASMJIT_BUILD_X86 || ASMJIT_BUILD_X64)
