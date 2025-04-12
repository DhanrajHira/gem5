#include "cpu/o3/mdp/naive_mdp.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"

namespace gem5 {
namespace o3 {
NaiveMDP::NaiveMDP(const NaiveMDPParams &params)
    : MemoryDependencePredictor(params), isOptimistic(params.optimistic) {
  if (!isOptimistic) {
    unissuedStores.reserve(128);
  }
}

void NaiveMDP::violation(const DynInstPtr &store, const DynInstPtr &load) {
  return; // NaiveMDP does nothing.
}

void NaiveMDP::insertLoad(const DynInstPtr &load) {
  return; // NaiveMDP does nothing.
}

void NaiveMDP::insertStore(const DynInstPtr &store) {
  // Remember unissued stores if we are not optimistic
  // else do nothing.
  if (!isOptimistic) {
    unissuedStores.push_back(store->seqNum);
  }
  return;
}

void NaiveMDP::checkInst(const DynInstPtr &inst,
                         std::vector<InstSeqNum> &producing_stores) {
  if (isOptimistic) {
    return; // Optimistic MDP predicts there are never any dependencies.
  }

  for (const auto seqNum : unissuedStores) {
    producing_stores.push_back(seqNum);
  }
}

/** Records this PC/sequence number as issued. */
void NaiveMDP::issued(const DynInstPtr &issued) {
  if (!isOptimistic) {
    unissuedStores.erase(std::remove(unissuedStores.begin(),
                                     unissuedStores.end(), issued->seqNum),
                         unissuedStores.end());
  }
}

/** Squashes for a specific thread until the given sequence number. */
void NaiveMDP::squash(InstSeqNum squashed_num, ThreadID tid) {
  if (isOptimistic) {
    return;
  }

  auto less_than_squashed_num = [squashed_num](InstSeqNum seqNum) {
    return seqNum <= squashed_num;
  };
  auto partition_point = std::remove_if(
      unissuedStores.begin(), unissuedStores.end(), less_than_squashed_num);
  unissuedStores.erase(partition_point, unissuedStores.end());
}

/** Resets all tables. */
void NaiveMDP::clear() { unissuedStores.clear(); }
} // namespace o3
} // namespace gem5
