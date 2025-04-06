#include "cpu/o3/mdp/store_set.hh"

#include "cpu/o3/mdp/memory_dependence_predictor.hh"

namespace gem5 {
namespace o3 {
StoreSetMDP::StoreSetMDP(const StoreSetMDPParams &params)
    : MemoryDependencePredictor(params) {}

void StoreSetMDP::violation(const DynInstPtr &store, const DynInstPtr &load) {};

void StoreSetMDP::insertLoad(const DynInstPtr &load) {};

void StoreSetMDP::insertStore(const DynInstPtr &store) {};

void StoreSetMDP::checkInst(const DynInstPtr &inst,
                       std::vector<InstSeqNum> &producing_stores) {};

void StoreSetMDP::issued(const DynInstPtr &issued) {};

void StoreSetMDP::squash(InstSeqNum squashed_num) {};

void StoreSetMDP::clear() {};
}; // namespace o3
} // namespace gem5
