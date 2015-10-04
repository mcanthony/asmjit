// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Export]
#define ASMJIT_EXPORTS

// [Guard]
#include "../build.h"
#if !defined(ASMJIT_DISABLE_COMPILER)

// [Dependencies - AsmJit]
#include "../base/assembler.h"
#include "../base/compiler.h"
#include "../base/compilercontext_p.h"
#include "../base/cpuinfo.h"
#include "../base/logger.h"
#include "../base/utils.h"

// [Dependencies - C]
#include <stdarg.h>

// [Api-Begin]
#include "../apibegin.h"

namespace asmjit {

// ============================================================================
// [Constants]
// ============================================================================

static const char noName[1] = { '\0' };
enum { kBaseCompilerDefaultLookAhead = 64 };

// ============================================================================
// [asmjit::Compiler - Construction / Destruction]
// ============================================================================

Compiler::Compiler(Runtime* runtime) :
  CodeGen(runtime),
  _assembler(NULL),
  _nodeFlowId(0),
  _nodeFlags(0),
  _tokenGenerator(0),
  _maxLookAhead(kBaseCompilerDefaultLookAhead),
  _targetVarMapping(NULL),
  _firstNode(NULL),
  _lastNode(NULL),
  _cursor(NULL),
  _func(NULL),
  _varZone(4096 - kZoneOverhead),
  _stringZone(4096 - kZoneOverhead),
  _localConstZone(4096 - kZoneOverhead),
  _localConstPool(&_localConstZone),
  _globalConstPool(&_baseZone) {}

Compiler::~Compiler() {
  reset(true);

  if (_assembler != NULL)
    delete _assembler;
}

// ============================================================================
// [asmjit::Compiler - Clear / Reset]
// ============================================================================

void Compiler::reset(bool releaseMemory) {
  // CodeGen members.
  _baseAddress = kNoBaseAddress;
  _instOptions = 0;
  _error = kErrorOk;

  _baseZone.reset(releaseMemory);

  // Compiler members.
  _nodeFlowId = 0;
  _nodeFlags = 0;
  _tokenGenerator = 0;

  if (_assembler != NULL)
    _assembler->reset(releaseMemory);

  _firstNode = NULL;
  _lastNode = NULL;

  _cursor = NULL;
  _func = NULL;

  _localConstPool.reset();
  _globalConstPool.reset();

  _localConstPoolLabel.reset();
  _globalConstPoolLabel.reset();

  _varZone.reset(releaseMemory);
  _stringZone.reset(releaseMemory);
  _localConstZone.reset(releaseMemory);

  _targetList.reset(releaseMemory);
  _varList.reset(releaseMemory);
}

// ============================================================================
// [asmjit::Compiler - ASNode Management]
// ============================================================================

ASNode* Compiler::setCursor(ASNode* node) {
  ASNode* old = _cursor;
  _cursor = node;
  return old;
}

ASNode* Compiler::addNode(ASNode* node) {
  ASMJIT_ASSERT(node != NULL);
  ASMJIT_ASSERT(node->_prev == NULL);
  ASMJIT_ASSERT(node->_next == NULL);

  if (_cursor == NULL) {
    if (_firstNode == NULL) {
      _firstNode = node;
      _lastNode = node;
    }
    else {
      node->_next = _firstNode;
      _firstNode->_prev = node;
      _firstNode = node;
    }
  }
  else {
    ASNode* prev = _cursor;
    ASNode* next = _cursor->_next;

    node->_prev = prev;
    node->_next = next;

    prev->_next = node;
    if (next)
      next->_prev = node;
    else
      _lastNode = node;
  }

  _cursor = node;
  return node;
}

ASNode* Compiler::addNodeBefore(ASNode* node, ASNode* ref) {
  ASMJIT_ASSERT(node != NULL);
  ASMJIT_ASSERT(node->_prev == NULL);
  ASMJIT_ASSERT(node->_next == NULL);
  ASMJIT_ASSERT(ref != NULL);

  ASNode* prev = ref->_prev;
  ASNode* next = ref;

  node->_prev = prev;
  node->_next = next;

  next->_prev = node;
  if (prev)
    prev->_next = node;
  else
    _firstNode = node;

  return node;
}

ASNode* Compiler::addNodeAfter(ASNode* node, ASNode* ref) {
  ASMJIT_ASSERT(node != NULL);
  ASMJIT_ASSERT(node->_prev == NULL);
  ASMJIT_ASSERT(node->_next == NULL);
  ASMJIT_ASSERT(ref != NULL);

  ASNode* prev = ref;
  ASNode* next = ref->_next;

  node->_prev = prev;
  node->_next = next;

  prev->_next = node;
  if (next)
    next->_prev = node;
  else
    _lastNode = node;

  return node;
}

static ASMJIT_INLINE void BaseCompiler_nodeRemoved(Compiler* self, ASNode* node_) {
  if (node_->isJmpOrJcc()) {
    ASJump* node = static_cast<ASJump*>(node_);
    ASLabel* label = node->getTarget();

    if (label != NULL) {
      // Disconnect.
      ASJump** pPrev = &label->_from;
      for (;;) {
        ASMJIT_ASSERT(*pPrev != NULL);
        ASJump* current = *pPrev;

        if (current == NULL)
          break;

        if (current == node) {
          *pPrev = node->_jumpNext;
          break;
        }

        pPrev = &current->_jumpNext;
      }

      label->subNumRefs();
    }
  }
}

ASNode* Compiler::removeNode(ASNode* node) {
  ASNode* prev = node->_prev;
  ASNode* next = node->_next;

  if (_firstNode == node)
    _firstNode = next;
  else
    prev->_next = next;

  if (_lastNode == node)
    _lastNode  = prev;
  else
    next->_prev = prev;

  node->_prev = NULL;
  node->_next = NULL;

  if (_cursor == node)
    _cursor = prev;
  BaseCompiler_nodeRemoved(this, node);

  return node;
}

void Compiler::removeNodes(ASNode* first, ASNode* last) {
  if (first == last) {
    removeNode(first);
    return;
  }

  ASNode* prev = first->_prev;
  ASNode* next = last->_next;

  if (_firstNode == first)
    _firstNode = next;
  else
    prev->_next = next;

  if (_lastNode == last)
    _lastNode  = prev;
  else
    next->_prev = prev;

  ASNode* node = first;
  for (;;) {
    ASNode* next = node->getNext();
    ASMJIT_ASSERT(next != NULL);

    node->_prev = NULL;
    node->_next = NULL;

    if (_cursor == node)
      _cursor = prev;
    BaseCompiler_nodeRemoved(this, node);

    if (node == last)
      break;
    node = next;
  }
}

// ============================================================================
// [asmjit::Compiler - Align]
// ============================================================================

ASAlign* Compiler::newAlign(uint32_t alignMode, uint32_t offset) {
  ASAlign* node = newNode<ASAlign>(alignMode, offset);
  if (node == NULL) {
    setError(kErrorNoHeapMemory);
    return NULL;
  }
  return node;
}

ASAlign* Compiler::addAlign(uint32_t alignMode, uint32_t offset) {
  ASAlign* node = newAlign(alignMode, offset);
  if (node == NULL)
    return NULL;
  return static_cast<ASAlign*>(addNode(node));
}

// ============================================================================
// [asmjit::Compiler - Target]
// ============================================================================

ASLabel* Compiler::newTarget() {
  ASLabel* node = newNode<ASLabel>(
    OperandUtil::makeLabelId(static_cast<uint32_t>(_targetList.getLength())));

  if (node == NULL || _targetList.append(node) != kErrorOk) {
    setError(kErrorNoHeapMemory);
    return NULL;
  }

  return node;
}

ASLabel* Compiler::addTarget() {
  ASLabel* node = newTarget();
  if (node == NULL)
    return NULL;
  return static_cast<ASLabel*>(addNode(node));
}

// ============================================================================
// [asmjit::Compiler - Label]
// ============================================================================

Error Compiler::_newLabel(Label* dst) {
  dst->_init_packed_op_sz_b0_b1_id(kOperandTypeLabel, 0, 0, 0, kInvalidValue);
  dst->_init_packed_d2_d3(0, 0);

  ASLabel* node = newTarget();
  if (node == NULL)
    return setError(kErrorNoHeapMemory);

  dst->_label.id = node->getLabelId();
  return kErrorOk;
}

Error Compiler::bind(const Label& label) {
  uint32_t index = label.getId();
  ASMJIT_ASSERT(index < _targetList.getLength());

  addNode(_targetList[index]);
  return kErrorOk;
}

// ============================================================================
// [asmjit::Compiler - Embed]
// ============================================================================

ASData* Compiler::newEmbed(const void* data, uint32_t size) {
  ASData* node;

  if (size > ASData::kInlineBufferSize) {
    void* clonedData = _stringZone.alloc(size);
    if (clonedData == NULL)
      goto _NoMemory;

    if (data != NULL)
      ::memcpy(clonedData, data, size);
    data = clonedData;
  }

  node = newNode<ASData>(const_cast<void*>(data), size);
  if (node == NULL)
    goto _NoMemory;
  return node;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASData* Compiler::addEmbed(const void* data, uint32_t size) {
  ASData* node = newEmbed(data, size);
  if (node == NULL)
    return node;
  return static_cast<ASData*>(addNode(node));
}

// ============================================================================
// [asmjit::Compiler - Comment]
// ============================================================================

ASComment* Compiler::newComment(const char* str) {
  ASComment* node;

  if (str != NULL && str[0]) {
    str = _stringZone.sdup(str);
    if (str == NULL)
      goto _NoMemory;
  }

  node = newNode<ASComment>(str);
  if (node == NULL)
    goto _NoMemory;
  return node;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

ASComment* Compiler::addComment(const char* str) {
  ASComment* node = newComment(str);
  if (node == NULL)
    return NULL;
  return static_cast<ASComment*>(addNode(node));
}

ASComment* Compiler::comment(const char* fmt, ...) {
  char buf[256];
  char* p = buf;

  if (fmt) {
    va_list ap;
    va_start(ap, fmt);
    p += vsnprintf(p, 254, fmt, ap);
    va_end(ap);
  }

  p[0] = '\0';
  return addComment(buf);
}

// ============================================================================
// [asmjit::Compiler - Hint]
// ============================================================================

ASHint* Compiler::newHint(Var& var, uint32_t hint, uint32_t value) {
  if (var.getId() == kInvalidValue)
    return NULL;

  VarData* vd = getVd(var);
  ASHint* node = newNode<ASHint>(vd, hint, value);

  if (node == NULL) {
    setError(kErrorNoHeapMemory);
    return NULL;
  }

  return node;
}

ASHint* Compiler::addHint(Var& var, uint32_t hint, uint32_t value) {
  if (var.getId() == kInvalidValue)
    return NULL;

  ASHint* node = newHint(var, hint, value);
  if (node == NULL)
    return NULL;
  return static_cast<ASHint*>(addNode(node));
}

// ============================================================================
// [asmjit::Compiler - Vars]
// ============================================================================

VarData* Compiler::_newVd(uint32_t type, uint32_t size, uint32_t c, const char* name) {
  VarData* vd = reinterpret_cast<VarData*>(_varZone.alloc(sizeof(VarData)));
  if (vd == NULL)
    goto _NoMemory;

  vd->_name = noName;
  vd->_id = OperandUtil::makeVarId(static_cast<uint32_t>(_varList.getLength()));
  vd->_localId = kInvalidValue;

  if (name != NULL && name[0] != '\0') {
    vd->_name = _stringZone.sdup(name);
  }

  vd->_type = static_cast<uint8_t>(type);
  vd->_class = static_cast<uint8_t>(c);
  vd->_flags = 0;
  vd->_priority = 10;

  vd->_state = kVarStateNone;
  vd->_regIndex = kInvalidReg;
  vd->_isStack = false;
  vd->_isMemArg = false;
  vd->_isCalculated = false;
  vd->_saveOnUnuse = false;
  vd->_modified = false;
  vd->_reserved0 = 0;
  vd->_alignment = static_cast<uint8_t>(Utils::iMin<uint32_t>(size, 64));

  vd->_size = size;
  vd->_homeMask = 0;

  vd->_memOffset = 0;
  vd->_memCell = NULL;

  vd->rReadCount = 0;
  vd->rWriteCount = 0;
  vd->mReadCount = 0;
  vd->mWriteCount = 0;

  vd->_va = NULL;

  if (_varList.append(vd) != kErrorOk)
    goto _NoMemory;
  return vd;

_NoMemory:
  setError(kErrorNoHeapMemory);
  return NULL;
}

void Compiler::alloc(Var& var) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintAlloc, kInvalidValue);
}

void Compiler::alloc(Var& var, uint32_t regIndex) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintAlloc, regIndex);
}

void Compiler::alloc(Var& var, const Reg& reg) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintAlloc, reg.getRegIndex());
}

void Compiler::save(Var& var) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintSave, kInvalidValue);
}

void Compiler::spill(Var& var) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintSpill, kInvalidValue);
}

void Compiler::unuse(Var& var) {
  if (var.getId() == kInvalidValue)
    return;
  addHint(var, kVarHintUnuse, kInvalidValue);
}

uint32_t Compiler::getPriority(Var& var) const {
  if (var.getId() == kInvalidValue)
    return kInvalidValue;

  VarData* vd = getVdById(var.getId());
  return vd->getPriority();
}

void Compiler::setPriority(Var& var, uint32_t priority) {
  if (var.getId() == kInvalidValue)
    return;

  if (priority > 255)
    priority = 255;

  VarData* vd = getVdById(var.getId());
  vd->_priority = static_cast<uint8_t>(priority);
}

bool Compiler::getSaveOnUnuse(Var& var) const {
  if (var.getId() == kInvalidValue)
    return false;

  VarData* vd = getVdById(var.getId());
  return static_cast<bool>(vd->_saveOnUnuse);
}

void Compiler::setSaveOnUnuse(Var& var, bool value) {
  if (var.getId() == kInvalidValue)
    return;

  VarData* vd = getVdById(var.getId());
  vd->_saveOnUnuse = value;
}

void Compiler::rename(Var& var, const char* name) {
  if (var.getId() == kInvalidValue)
    return;

  VarData* vd = getVdById(var.getId());
  vd->_name = noName;

  if (name != NULL && name[0] != '\0') {
    vd->_name = _stringZone.sdup(name);
  }
}

// ============================================================================
// [asmjit::Compiler - Assembler]
// ============================================================================

Assembler* Compiler::getAssembler() {
  Assembler* a = _assembler;

  if (a != NULL) {
    a->reset(false);
  }
  else {
    a = _newAssembler();
    _assembler = a;
  }

#if !defined(ASMJIT_DISABLE_LOGGER)
  Logger* logger = _logger;
  if (logger != NULL)
    a->setLogger(logger);
#endif // !ASMJIT_DISABLE_LOGGER

  a->setBaseAddress(_baseAddress);
  a->setFeatures(_features);

  return a;
}

} // asmjit namespace

// [Api-End]
#include "../apiend.h"

// [Guard]
#endif // !ASMJIT_DISABLE_COMPILER
