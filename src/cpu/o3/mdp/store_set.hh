#ifndef __CPU_O3_MDP_STORE_SET__
#define __CPU_O3_MDP_STORE_SET__

#include "cpu/o3/mdp/memory_dependence_predictor.hh"
#include "params/StoreSetMDP.hh"

namespace gem5 {
namespace o3 {
class StoreSetMDP : public MemoryDependencePredictor {
public:
  StoreSetMDP(const StoreSetMDPParams &params);

  void violation(const DynInstPtr &store, const DynInstPtr &load) override;

  /** Inserts a load into the store set predictor.  This does nothing but
   * is included in case other predictors require a similar function.
   */
  void insertLoad(const DynInstPtr &load) override;

  /** Inserts a store into the store set predictor.  Updates the
   * LFST if the store has a valid SSID. */
  void insertStore(const DynInstPtr &store) override;

  void checkInst(const DynInstPtr &inst,
                 std::vector<InstSeqNum> &producing_stores) override;

  /** Records this PC/sequence number as issued. */
  void issued(const DynInstPtr &issued) override;

  /** Squashes for a specific thread until the given sequence number. */
  void squash(InstSeqNum squashed_num) override;

  /** Resets all tables. */
  void clear() override;
};
} // namespace o3
} // namespace gem5

#endif
