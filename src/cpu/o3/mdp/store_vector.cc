#include "cpu/o3/mdp/store_vector.hh"
#include "base/intmath.hh"
#include "cpu/o3/dyn_inst.hh"
#include "debug/MDP.hh"

namespace gem5 {
namespace o3 {

StoreVectorMDP::StoreVectorMDP(const StoreVectorMDPParams &params)
    : MemoryDependencePredictor(params), clearPeriod(params.clearPeriod),
      SVTSize(params.SVTSize), SVTVectorSize(params.SVTVectorSize),
      SQSize(params.SQSize) {

  if (!isPowerOf2(SVTSize)) {
    fatal("StoreVectorMDP: The SVT size must be a power of 2!");
  }

  // Init the SVT.
  SVT.reserve(SVTSize);
  for (auto i = 0; i < SVTSize; i++)
    SVT.push_back(std::vector<bool>(SVTVectorSize));

  DPRINTF(MDP,
          "Initialized StoreVectorMDP with [SVTSize=%lu, VectorSize=%lu]\n",
          SVTSize, SVTVectorSize);
}

void StoreVectorMDP::violation(const DynInstPtr &store,
                               const DynInstPtr &violating_load) {
  assert(store->isStore());
  auto violating_store_offset = violating_load->sqIdx - store->sqIdx;
  assert(violating_store_offset >= 0);
  auto load_PC = violating_load->pcState().instAddr();
  auto SV_index = getSVIdx(load_PC);
  if (violating_store_offset >= SVTVectorSize) {
    fatal("violating_store_offset >= SVTVectorSize");
  }
  auto &store_vector = SVT[SV_index];
  store_vector[violating_store_offset] = true;
}

void StoreVectorMDP::checkClear() {
  memOpsPred++;
  if (memOpsPred > clearPeriod) {
    memOpsPred = 0;
    clear();
  }
}

void StoreVectorMDP::insertLoad(const DynInstPtr &load) {
  checkClear();
  return; // StoreVector does nothing
}

void StoreVectorMDP::insertStore(const DynInstPtr &store) {
  checkClear();
  return; // StoreVector does nothing
}

void StoreVectorMDP::checkInst(const DynInstPtr &inst,
                               std::vector<InstSeqNum> &producing_stores) {

  if (!inst->isLoad())
    return;

  auto load_PC = inst->pcState().instAddr();
  auto SV_idx = getSVIdx(load_PC);
  auto sq_it = inst->sqIt;
  const auto *sq = sq_it.getCircularQueue();
  auto sq_size = sq->size();
  // Nothing in the SQ, no point in predicting.
  if (sq_size <= 0) {
    return;
  }

  const auto &SV = SVT[SV_idx];
  auto sq_entry = sq_it;
  for (auto i = 0; i < SVTVectorSize && sq_size; i++, --sq_size) {
    --sq_entry;
    InstSeqNum producing_store = sq_entry->instruction()->seqNum;
    bool does_depend = SV[i];
    if (!does_depend) {
      continue;
    }
    producing_stores.push_back(producing_store);
  }
}

void StoreVectorMDP::issued(const DynInstPtr &issued) {
  return; // StoreVector does nothing
}

void StoreVectorMDP::squash(InstSeqNum squashed_num, ThreadID tid) {
  return; // StoreVector does nothing
}

void StoreVectorMDP::clear() {
  for (std::vector<bool> &SV : SVT)
    for (int i = 0; i < SVTVectorSize; ++i)
      SV[i] = false;
};

size_t StoreVectorMDP::getSVIdx(Addr load_PC) {
  return (load_PC & (SVTSize - 1));
}

} // namespace o3
} // namespace gem5
