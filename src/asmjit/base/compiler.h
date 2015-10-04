// [AsmJit]
// Complete x86/x64 JIT and Remote Assembler for C++.
//
// [License]
// Zlib - See LICENSE.md file in the package.

// [Guard]
#ifndef _ASMJIT_BASE_COMPILER_H
#define _ASMJIT_BASE_COMPILER_H

#include "../build.h"
#if !defined(ASMJIT_DISABLE_COMPILER)

// [Dependencies - AsmJit]
#include "../base/assembler.h"
#include "../base/codegen.h"
#include "../base/compilerfunc.h"
#include "../base/constpool.h"
#include "../base/containers.h"
#include "../base/error.h"
#include "../base/operand.h"
#include "../base/stream.h"
#include "../base/utils.h"
#include "../base/zone.h"

// [Api-Begin]
#include "../apibegin.h"

namespace asmjit {

// ============================================================================
// [Forward Declarations]
// ============================================================================

struct VarAttr;
struct VarData;
struct VarMap;
struct VarState;

// ============================================================================
// [asmjit::ConstScope]
// ============================================================================

//! \addtogroup asmjit_base_compiler
//! \{

//! Scope of the constant.
ASMJIT_ENUM(ConstScope) {
  //! Local constant, always embedded right after the current function.
  kConstScopeLocal = 0,
  //! Global constant, embedded at the end of the currently compiled code.
  kConstScopeGlobal = 1
};

//! \}

// ============================================================================
// [asmjit::Compiler]
// ============================================================================

//! \addtogroup asmjit_base_general
//! \{

//! Base compiler.
//!
//! \sa Assembler.
struct ASMJIT_VIRTAPI Compiler : public CodeGen {
  ASMJIT_NO_COPY(Compiler)

  // --------------------------------------------------------------------------
  // [Construction / Destruction]
  // --------------------------------------------------------------------------

  //! Create a new `Compiler` instance.
  ASMJIT_API Compiler(Runtime* runtime);
  //! Destroy the `Compiler` instance.
  ASMJIT_API virtual ~Compiler();

  // --------------------------------------------------------------------------
  // [LookAhead]
  // --------------------------------------------------------------------------

  //! Get maximum look ahead.
  ASMJIT_INLINE uint32_t getMaxLookAhead() const {
    return _maxLookAhead;
  }

  //! Set maximum look ahead to `val`.
  ASMJIT_INLINE void setMaxLookAhead(uint32_t val) {
    _maxLookAhead = val;
  }

  // --------------------------------------------------------------------------
  // [Clear / Reset]
  // --------------------------------------------------------------------------

  //! Reset the compiler.
  //!
  //! If `releaseMemory` is true all buffers will be released to the system.
  ASMJIT_API void reset(bool releaseMemory = false);

  // --------------------------------------------------------------------------
  // [Token ID]
  // --------------------------------------------------------------------------

  //! \internal
  //!
  //! Reset the token-id generator.
  ASMJIT_INLINE void _resetTokenGenerator() { _tokenGenerator = 0; }

  //! \internal
  //!
  //! Generate a new unique token id.
  ASMJIT_INLINE uint32_t _generateUniqueToken() { return ++_tokenGenerator; }

  // --------------------------------------------------------------------------
  // [Nodes]
  // --------------------------------------------------------------------------

  template<typename T>
  ASMJIT_INLINE T* newNode() {
    void* p = _baseZone.alloc(sizeof(T));
    return new(p) T(this);
  }

  template<typename T, typename P0>
  ASMJIT_INLINE T* newNode(P0 p0) {
    void* p = _baseZone.alloc(sizeof(T));
    return new(p) T(this, p0);
  }

  template<typename T, typename P0, typename P1>
  ASMJIT_INLINE T* newNode(P0 p0, P1 p1) {
    void* p = _baseZone.alloc(sizeof(T));
    return new(p) T(this, p0, p1);
  }

  template<typename T, typename P0, typename P1, typename P2>
  ASMJIT_INLINE T* newNode(P0 p0, P1 p1, P2 p2) {
    void* p = _baseZone.alloc(sizeof(T));
    return new(p) T(this, p0, p1, p2);
  }

  //! Get the first node.
  ASMJIT_INLINE ASNode* getFirstNode() const { return _firstNode; }
  //! Get the last node.
  ASMJIT_INLINE ASNode* getLastNode() const { return _lastNode; }

  //! Get current node.
  //!
  //! \note If this method returns `NULL` it means that nothing has been emitted
  //! yet.
  ASMJIT_INLINE ASNode* getCursor() const { return _cursor; }
  //! Set the current node without returning the previous node (private).
  ASMJIT_INLINE void _setCursor(ASNode* node) { _cursor = node; }
  //! Set the current node to `node` and return the previous one.
  ASMJIT_API ASNode* setCursor(ASNode* node);

  //! Add node `node` after current and set current to `node`.
  ASMJIT_API ASNode* addNode(ASNode* node);
  //! Add node before `ref`.
  ASMJIT_API ASNode* addNodeBefore(ASNode* node, ASNode* ref);
  //! Add node after `ref`.
  ASMJIT_API ASNode* addNodeAfter(ASNode* node, ASNode* ref);
  //! Remove node `node`.
  ASMJIT_API ASNode* removeNode(ASNode* node);
  //! Remove multiple nodes.
  ASMJIT_API void removeNodes(ASNode* first, ASNode* last);

  // --------------------------------------------------------------------------
  // [Func]
  // --------------------------------------------------------------------------

  //! Get current function.
  ASMJIT_INLINE ASFunc* getFunc() const { return _func; }

  // --------------------------------------------------------------------------
  // [Align]
  // --------------------------------------------------------------------------

  //! Create a new `ASAlign`.
  ASMJIT_API ASAlign* newAlign(uint32_t alignMode, uint32_t offset);
  //! Add a new `ASAlign`.
  ASMJIT_API ASAlign* addAlign(uint32_t alignMode, uint32_t offset);

  //! Align target buffer to the `offset` specified.
  //!
  //! The sequence that is used to fill the gap between the aligned location
  //! and the current depends on `alignMode`, see \ref AlignMode.
  ASMJIT_INLINE ASAlign* align(uint32_t alignMode, uint32_t offset) {
    return addAlign(alignMode, offset);
  }

  // --------------------------------------------------------------------------
  // [Label]
  // --------------------------------------------------------------------------

  //! Create a new `ASLabel`.
  ASMJIT_API ASLabel* newTarget();
  //! Add a new `ASLabel`.
  ASMJIT_API ASLabel* addTarget();

  //! Get `ASLabel` by `id`.
  ASMJIT_INLINE ASLabel* getASLabel(uint32_t id) {
    ASMJIT_ASSERT(OperandUtil::isLabelId(id));
    ASMJIT_ASSERT(id < _targetList.getLength());

    return _targetList[id];
  }

  //! Get `ASLabel` by `label`.
  ASMJIT_INLINE ASLabel* getASLabel(const Label& label) {
    return getASLabel(label.getId());
  }

  //! Get count of created labels.
  ASMJIT_INLINE size_t getLabelsCount() const {
    return _targetList.getLength();
  }

  //! Get whether `label` is created.
  ASMJIT_INLINE bool isLabelValid(const Label& label) const {
    return isLabelValid(label.getId());
  }

  //! \overload
  ASMJIT_INLINE bool isLabelValid(uint32_t id) const {
    return static_cast<size_t>(id) < _targetList.getLength();
  }

  //! Get `label` offset or -1 if the label is not bound.
  //!
  //! This method can be only called after the code has been serialized to the
  //! `Assembler`, otherwise the offset returned will be -1 (even if the label
  //! has been bound).
  ASMJIT_INLINE intptr_t getLabelOffset(const Label& label) const {
    return getLabelOffset(label.getId());
  }

  //! \overload
  ASMJIT_INLINE intptr_t getLabelOffset(uint32_t id) const {
    ASMJIT_ASSERT(isLabelValid(id));
    return _targetList[id]->getOffset();
  }

  //! \internal
  //!
  //! Create and initialize a new `Label`.
  ASMJIT_API Error _newLabel(Label* dst);

  //! Create and return a new `Label`.
  ASMJIT_INLINE Label newLabel() {
    Label result(NoInit);
    _newLabel(&result);
    return result;
  }

  //! Bind label to the current offset.
  //!
  //! \note Label can be bound only once!
  ASMJIT_API Error bind(const Label& label);

  // --------------------------------------------------------------------------
  // [Embed]
  // --------------------------------------------------------------------------

  //! Create a new `ASData`.
  ASMJIT_API ASData* newEmbed(const void* data, uint32_t size);
  //! Add a new `ASData`.
  ASMJIT_API ASData* addEmbed(const void* data, uint32_t size);

  //! Embed data.
  ASMJIT_INLINE ASData* embed(const void* data, uint32_t size) {
    return addEmbed(data, size);
  }

  // --------------------------------------------------------------------------
  // [Comment]
  // --------------------------------------------------------------------------

  //! Create a new `ASComment`.
  ASMJIT_API ASComment* newComment(const char* str);
  //! Add a new `ASComment`.
  ASMJIT_API ASComment* addComment(const char* str);

  //! Emit a single comment line.
  ASMJIT_API ASComment* comment(const char* fmt, ...);

  // --------------------------------------------------------------------------
  // [Hint]
  // --------------------------------------------------------------------------

  //! Create a new `ASHint`.
  ASMJIT_API ASHint* newHint(Var& var, uint32_t hint, uint32_t value);
  //! Add a new `ASHint`.
  ASMJIT_API ASHint* addHint(Var& var, uint32_t hint, uint32_t value);

  // --------------------------------------------------------------------------
  // [Vars]
  // --------------------------------------------------------------------------

  //! Get whether variable `var` is created.
  ASMJIT_INLINE bool isVarValid(const Var& var) const {
    return static_cast<size_t>(var.getId() & kOperandIdNum) < _varList.getLength();
  }

  //! \internal
  //!
  //! Get `VarData` by `var`.
  ASMJIT_INLINE VarData* getVd(const Var& var) const {
    return getVdById(var.getId());
  }

  //! \internal
  //!
  //! Get `VarData` by `id`.
  ASMJIT_INLINE VarData* getVdById(uint32_t id) const {
    ASMJIT_ASSERT(id != kInvalidValue);
    ASMJIT_ASSERT(static_cast<size_t>(id & kOperandIdNum) < _varList.getLength());

    return _varList[id & kOperandIdNum];
  }

  //! \internal
  //!
  //! Get an array of 'VarData*'.
  ASMJIT_INLINE VarData** _getVdArray() const {
    return const_cast<VarData**>(_varList.getData());
  }

  //! \internal
  //!
  //! Create a new `VarData`.
  ASMJIT_API VarData* _newVd(uint32_t type, uint32_t size, uint32_t c, const char* name);

  //! Create a new `Var`.
  virtual Error _newVar(Var* var, uint32_t type, const char* name) = 0;

  //! Alloc variable `var`.
  ASMJIT_API void alloc(Var& var);
  //! Alloc variable `var` using `regIndex` as a register index.
  ASMJIT_API void alloc(Var& var, uint32_t regIndex);
  //! Alloc variable `var` using `reg` as a register operand.
  ASMJIT_API void alloc(Var& var, const Reg& reg);
  //! Spill variable `var`.
  ASMJIT_API void spill(Var& var);
  //! Save variable `var` if the status is `modified` at this point.
  ASMJIT_API void save(Var& var);
  //! Unuse variable `var`.
  ASMJIT_API void unuse(Var& var);

  //! Get priority of variable `var`.
  ASMJIT_API uint32_t getPriority(Var& var) const;
  //! Set priority of variable `var` to `priority`.
  ASMJIT_API void setPriority(Var& var, uint32_t priority);

  //! Get save-on-unuse `var` property.
  ASMJIT_API bool getSaveOnUnuse(Var& var) const;
  //! Set save-on-unuse `var` property to `value`.
  ASMJIT_API void setSaveOnUnuse(Var& var, bool value);

  //! Rename variable `var` to `name`.
  //!
  //! \note Only new name will appear in the logger.
  ASMJIT_API void rename(Var& var, const char* name);

  // --------------------------------------------------------------------------
  // [Stack]
  // --------------------------------------------------------------------------

  //! \internal
  //!
  //! Create a new memory chunk allocated on the current function's stack.
  virtual Error _newStack(BaseMem* mem, uint32_t size, uint32_t alignment, const char* name) = 0;

  // --------------------------------------------------------------------------
  // [Const]
  // --------------------------------------------------------------------------

  //! \internal
  //!
  //! Put data to a constant-pool and get a memory reference to it.
  virtual Error _newConst(BaseMem* mem, uint32_t scope, const void* data, size_t size) = 0;

  // --------------------------------------------------------------------------
  // [Assembler]
  // --------------------------------------------------------------------------

  //! Get an assembler instance that is associated with the compiler.
  //!
  //! \note One instance of `Assembler` is shared and has lifetime same as the
  //! compiler, however, each call to `getAssembler()` resets the assembler so
  //! new code can be serialized into it.
  ASMJIT_API Assembler* getAssembler();

  //! \internal
  //!
  //! Create a new `Assembler` instance associated with the compiler.
  virtual Assembler* _newAssembler() = 0;

  // --------------------------------------------------------------------------
  // [Serialize]
  // --------------------------------------------------------------------------

  //! Serialize a compiled code to `assembler`.
  virtual Error serialize(Assembler* assembler) = 0;

  // --------------------------------------------------------------------------
  // [Members]
  // --------------------------------------------------------------------------

  //! Internal assembler.
  Assembler* _assembler;

  //! Flow id added to each node created (used only by `Context)`.
  uint32_t _nodeFlowId;
  //! Flags added to each node created (used only by `Context)`.
  uint32_t _nodeFlags;

  //! Processing token generator.
  //!
  //! Used to get a unique token that is then used to process `ASNode`s. See
  //! `Compiler::_getUniqueToken()` for more details.
  uint32_t _tokenGenerator;

  //! Maximum count of nodes to look ahead when allocating/spilling
  //! registers.
  uint32_t _maxLookAhead;

  //! Variable mapping (translates incoming VarType into target).
  const uint8_t* _targetVarMapping;

  //! First node.
  ASNode* _firstNode;
  //! Last node.
  ASNode* _lastNode;

  //! Current node.
  ASNode* _cursor;
  //! Current function.
  ASFunc* _func;

  //! Variable zone.
  Zone _varZone;
  //! String/data zone.
  Zone _stringZone;
  //! Local constant pool zone.
  Zone _localConstZone;

  //! ASLabel list.
  PodVector<ASLabel*> _targetList;
  //! VarData list.
  PodVector<VarData*> _varList;

  //! Local constant pool, flushed at the end of each function.
  ConstPool _localConstPool;
  //! Global constant pool, flushed at the end of the compilation.
  ConstPool _globalConstPool;

  //! Label to start of the local constant pool.
  Label _localConstPoolLabel;
  //! Label to start of the global constant pool.
  Label _globalConstPoolLabel;
};

//! \}

// ============================================================================
// [Defined-Later]
// ============================================================================

ASMJIT_INLINE Label::Label(Compiler& c) : Operand(NoInit) {
  c._newLabel(this);
}

ASMJIT_INLINE ASNode::ASNode(Compiler* compiler, uint32_t type) {
  _prev = NULL;
  _next = NULL;
  _type = static_cast<uint8_t>(type);
  _opCount = 0;
  _flags = static_cast<uint16_t>(compiler->_nodeFlags);
  _flowId = compiler->_nodeFlowId;
  _tokenId = 0;
  _comment = NULL;
  _map = NULL;
  _liveness = NULL;
  _state = NULL;
}

} // asmjit namespace

// [Api-End]
#include "../apiend.h"

// [Guard]
#endif // !ASMJIT_DISABLE_COMPILER
#endif // _ASMJIT_BASE_COMPILER_H
