#ifndef __CPU_O3_MDP_STORE_SET__
#define __CPU_O3_MDP_STORE_SET__

#include "base/cache/associative_cache.hh"
#include "base/cache/cache_entry.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/mdp/memory_dependence_predictor.hh"
#include "params/StoreSetMDP.hh"
#include "debug/MDP.hh"
#include <functional>

namespace gem5 {
namespace o3 {
class StoreSetMDP : public MemoryDependencePredictor {
public:
  using SSID = Addr;

  class SSITEntry : public CacheEntry {
  private:
    SSID _ssid;

  public:
    using TagExtractor = std::function<Addr(Addr)>;

    SSITEntry(TagExtractor ext) : CacheEntry(ext), _ssid(MaxAddr) {}

    void setSSID(SSID id) { _ssid = id; }
    SSID getSSID(void) const { return _ssid; }
  };
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
  void squash(InstSeqNum squashed_num, ThreadID tid) override;

  /** Resets all tables. */
  void clear() override;

private:
  /** Calculates a Store Set ID based on the PC. */
  inline SSID calcSSID(Addr PC) { return ((PC ^ (PC >> 10)) % LFSTSize); }

  /** Checks if the table needs to be cleared at the set number of predictions
   */
  void checkClear();

  /** The Store Set ID Table. */
  AssociativeCache<SSITEntry> SSIT;

  /** Last Fetched Store Table. */
  std::vector<InstSeqNum> LFST;

  /** Bit vector to tell if the LFST has a valid entry. */
  std::vector<bool> validLFST;

  /** Map of stores that have been inserted into the store set, but
   * not yet issued or squashed.
   */
  using SeqNumMapT = std::map<InstSeqNum, int, std::greater<InstSeqNum>>;
  using SeqNumMapIt = SeqNumMapT::iterator;

  SeqNumMapT storeList;

  /** Number of loads/stores to process before wiping predictor so all
   * entries don't get saturated
   */
  uint64_t clearPeriod;

  /** Store Set ID Table size, in entries. */
  int SSITSize;

  /** Last Fetched Store Table size, in entries. */
  int LFSTSize;

  /** Number of memory operations predicted since last clear of predictor */
  int memOpsPred;
};
} // namespace o3
} // namespace gem5

#endif
