#ifndef __CPU_O3_MDP_STORE_VECTOR__
#define __CPU_O3_MDP_STORE_VECTOR__

#include "cpu/o3/mdp/memory_dependence_predictor.hh"
#include "params/StoreVectorMDP.hh"
#include <cstdint>

namespace gem5 {
namespace o3 {
class StoreVectorMDP : public MemoryDependencePredictor {
public:
  StoreVectorMDP(const StoreVectorMDPParams &params);

  void violation(const DynInstPtr &store, const DynInstPtr &load) override;

  void insertLoad(const DynInstPtr &load) override;

  void insertStore(const DynInstPtr &store) override;

  void checkInst(const DynInstPtr &inst,
                 std::vector<InstSeqNum> &producing_stores) override;

  void issued(const DynInstPtr &issued) override;

  void squash(InstSeqNum squashed_num, ThreadID tid) override;

  void clear() override;

private:
  size_t getSVIdx(Addr load_PC);

  void checkClear();

  std::vector<std::vector<bool>> SVT;

  /** Number of loads/stores to process before wiping predictor so all
   * entries don't get saturated
   */
  uint64_t clearPeriod;

  uint64_t SVTSize;

  uint64_t SVTVectorSize;

  uint64_t SQSize;

  /** Number of memory operations predicted since last clear of predictor */
  int memOpsPred;
};
} // namespace o3
} // namespace gem5

#endif
