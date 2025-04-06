#ifndef __CPU_O3_MDP_MEMORY_DEPENDENCY_PREDICTOR__
#define __CPU_O3_MDP_MEMORY_DEPENDENCY_PREDICTOR__

#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "params/MemoryDependencePredictor.hh"
#include "sim/sim_object.hh"

namespace gem5 {
namespace o3 {
class MemoryDependencePredictor : public SimObject {
public:
  MemoryDependencePredictor(const MemoryDependencePredictorParams &params);

  /** Records a memory ordering violation between the younger load
   * and the older store. */
  virtual void violation(const DynInstPtr &store, const DynInstPtr &load) = 0;

  /** Inserts a load into the predictor. This is called when an instruction is
   * first inserted in the IQ
   */
  virtual void insertLoad(const DynInstPtr &load) = 0;

  /** Inserts a store into the predictor. This is called when an instruction is
   * first inserted into the IQ */
  virtual void insertStore(const DynInstPtr &store) = 0;

  /** Checks if the given instruction is dependent upon any store. Inserts the
   * sequence numbers of the producing stores (if any) into the second inout
   * parameter.
   */
  virtual void checkInst(const DynInstPtr &inst,
                               std::vector<InstSeqNum> &producing_stores) = 0;

  /** Records this instruction as issued. */
  virtual void issued(const DynInstPtr &issued) = 0;

  /** Squashes until the given sequence number. */
  virtual void squash(InstSeqNum squashed_num) = 0;

  /** Resets all tables. */
  virtual void clear() = 0;

  virtual ~MemoryDependencePredictor() = default;
};
} // namespace o3
} // namespace gem5

#endif // __CPU_O3_MDP__
