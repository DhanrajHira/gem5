#ifndef __CPU_O3_MDP_NAIVE_MDP__
#define __CPU_O3_MDP_NAIVE_MDP__

#include "cpu/o3/mdp/memory_dependence_predictor.hh"
#include "params/NaiveMDP.hh"

namespace gem5 {
namespace o3 {
class NaiveMDP : public MemoryDependencePredictor {
public:
  NaiveMDP(const NaiveMDPParams &params);

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
  void squash(InstSeqNum squashed_num, ThreadID tid) override;

  /** Resets all tables. */
  void clear() override;

private:
  /** Whether this naive predictor is optimistically predicting no dependencies
   */
  bool isOptimistic;

  /** Pessimistic naive predictor needs to keep track of unissued load so that 
   * it can say that every load depends on them */
  std::vector<InstSeqNum> unissuedStores;
};
} // namespace o3
} // namespace gem5

#endif
