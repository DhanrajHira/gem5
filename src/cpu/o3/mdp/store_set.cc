#include "cpu/o3/mdp/store_set.hh"

#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/mdp/memory_dependence_predictor.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

namespace gem5 {
namespace o3 {
StoreSetMDP::StoreSetMDP(const StoreSetMDPParams &params)
    : MemoryDependencePredictor(params),
      SSIT("SSIT", params.SSITSize, params.SSITAssoc, params.SSITReplPolicy,
           params.SSITIndexingPolicy,
           SSITEntry(genTagExtractor(params.SSITIndexingPolicy))),
      clearPeriod(params.clear_period), SSITSize(params.SSITSize),
      LFSTSize(params.LFSTSize), memOpsPred(0) {
  DPRINTF(MDP, "Initialized StoreSetMDP with [SSITSize=%lu, LFSTSize=%lu]\n",
          SSITSize, LFSTSize);
  if (!isPowerOf2(SSITSize)) {
    fatal("Invalid SSIT size!\n");
  }

  if (!isPowerOf2(LFSTSize)) {
    fatal("Invalid LFST size!\n");
  }

  LFST.resize(LFSTSize);
  validLFST.resize(LFSTSize);

  for (int i = 0; i < LFSTSize; ++i) {
    validLFST[i] = false;
    LFST[i] = 0;
  }
}

void StoreSetMDP::violation(const DynInstPtr &store, const DynInstPtr &load) {
  auto load_PC = load->pcState().instAddr();
  auto store_PC = store->pcState().instAddr();
  auto ld_entry = SSIT.findEntry(load_PC);
  auto st_entry = SSIT.findEntry(store_PC);

  bool valid_load_SSID = ld_entry && ld_entry->isValid();
  bool valid_store_SSID = st_entry && st_entry->isValid();

  if (!valid_load_SSID && !valid_store_SSID) {
    // Calculate a new SSID here.
    SSID new_set = calcSSID(load_PC);

    assert(new_set < LFSTSize);

    SSITEntry *ld_entry = SSIT.findVictim({load_PC});
    ld_entry->setSSID(new_set);
    SSIT.insertEntry({load_PC}, ld_entry);

    SSITEntry *st_entry = SSIT.findVictim({store_PC});
    st_entry->setSSID(new_set);
    SSIT.insertEntry({store_PC}, st_entry);
    DPRINTF(MDP,
            "StoreSet: Neither load nor store had a valid "
            "storeset, creating a new one: %i for load %#x, store %#x\n",
            new_set, load_PC, store_PC);

  } else if (valid_load_SSID && !valid_store_SSID) {
    SSID load_SSID = ld_entry->getSSID();
    SSITEntry *st_entry = SSIT.findVictim({store_PC});
    st_entry->setSSID(load_SSID);
    SSIT.insertEntry({store_PC}, st_entry);

    assert(load_SSID < LFSTSize);

    DPRINTF(MDP,
            "StoreSet: Load had a valid store set.  Adding "
            "store to that set: %i for load %#x, store %#x\n",
            load_SSID, load_PC, store_PC);

  } else if (!valid_load_SSID && valid_store_SSID) {
    SSID store_SSID = st_entry->getSSID();
    SSITEntry *ld_entry = SSIT.findVictim({load_PC});
    ld_entry->setSSID(store_SSID);
    SSIT.insertEntry({load_PC}, ld_entry);

    DPRINTF(MDP,
            "StoreSet: Store had a valid store set: %i for "
            "load %#x, store %#x\n",
            store_SSID, load_PC, store_PC);

  } else {
    SSID load_SSID = ld_entry->getSSID();
    SSID store_SSID = st_entry->getSSID();

    assert(load_SSID < LFSTSize && store_SSID < LFSTSize);

    // The store set with the lower number wins
    if (store_SSID > load_SSID) {
      st_entry->setSSID(load_SSID);
      DPRINTF(MDP,
              "StoreSet: Load had smaller store set: %i; "
              "for load %#x, store %#x\n",
              load_SSID, load_PC, store_PC);
    } else {
      ld_entry->setSSID(store_SSID);

      DPRINTF(MDP,
              "StoreSet: Store had smaller store set: %i; "
              "for load %#x, store %#x\n",
              store_SSID, load_PC, store_PC);
    }
  }
};

void StoreSetMDP::checkClear() {
  memOpsPred++;
  if (memOpsPred > clearPeriod) {
    DPRINTF(MDP, "Wiping predictor state beacuse %d ld/st executed\n",
            clearPeriod);
    memOpsPred = 0;
    clear();
  }
}

void StoreSetMDP::insertLoad(const DynInstPtr &load) {
  checkClear();
  return; // StoreSet does nothing.
};

void StoreSetMDP::insertStore(const DynInstPtr &store) {
  auto store_PC = store->pcState().instAddr();
  auto st_entry = SSIT.findEntry(store_PC);
  bool valid_entry = st_entry && st_entry->isValid();

  checkClear();

  if (!valid_entry) {
    // Do nothing if there's no valid entry.
    return;
  } else {
    auto store_SSID = st_entry->getSSID();
    auto store_seq_num = store->seqNum;

    assert(store_SSID < LFSTSize);

    // Update the last store that was fetched with the current one.
    LFST[store_SSID] = store_seq_num;
    validLFST[store_SSID] = 1;
    storeList[store_seq_num] = store_SSID;

    DPRINTF(MDP, "Store %#x updated the LFST, SSID: %i\n", store_PC,
            store_SSID);
  }
};

void StoreSetMDP::checkInst(const DynInstPtr &inst,
                            std::vector<InstSeqNum> &producing_stores) {
  auto PC = inst->pcState().instAddr();
  auto entry = SSIT.findEntry(PC);
  bool valid_ssit = entry && entry->isValid();

  if (!valid_ssit || !validLFST[entry->getSSID()]) {
    if (!valid_ssit) {
      DPRINTF(MDP, "Inst %#x has no SSID\n", PC);
    } else {
      DPRINTF(MDP,
              "Inst %#x with SSID %i had no "
              "dependency\n",
              PC, entry->getSSID());
    }
    // No dependencies if there is no SSIT entry or its LFST is invalid.
    return;
  } else {
    auto inst_SSID = entry->getSSID();
    assert(inst_SSID < LFSTSize);
    DPRINTF(MDP,
            "Inst %#x with SSID %i had LFST "
            "inum of %i\n",
            PC, inst_SSID, LFST[inst_SSID]);
    producing_stores.push_back(LFST[inst_SSID]);
    return;
  }
};

void StoreSetMDP::issued(const DynInstPtr &issued) {
  if (!issued->isStore()) {
    return; // We only care about stores.
  }

  auto issued_PC = issued->pcState().instAddr();
  auto issued_seq_num = issued->seqNum;
  auto entry = SSIT.findEntry(issued_PC);
  bool valid_ssit = entry && entry->isValid();

  int store_SSID;

  SeqNumMapIt store_list_it = storeList.find(issued_seq_num);

  if (store_list_it != storeList.end()) {
    storeList.erase(store_list_it);
  }

  // Make sure the SSIT still has a valid entry for the issued store.
  if (!valid_ssit) {
    return;
  }

  store_SSID = entry->getSSID();

  assert(store_SSID < LFSTSize);

  // If the last fetched store in the store set refers to the store that
  // was just issued, then invalidate the entry.
  if (validLFST[store_SSID] && LFST[store_SSID] == issued_seq_num) {
    DPRINTF(MDP, "StoreSet: store invalidated itself in LFST.\n");
    validLFST[store_SSID] = false;
  }
};

void StoreSetMDP::squash(InstSeqNum squashed_num, ThreadID tid) {
  int idx;
  SeqNumMapIt store_list_it = storeList.begin();
  DPRINTF(MDP, "StoreSet: Squashing until inum %i\n", squashed_num);

  //@todo:Fix to only delete from correct thread
  while (!storeList.empty()) {
    idx = (*store_list_it).second;

    if ((*store_list_it).first <= squashed_num) {
      break;
    }

    bool younger = LFST[idx] > squashed_num;

    if (validLFST[idx] && younger) {
      DPRINTF(MDP, "Squashed [sn:%lli]\n", LFST[idx]);
      validLFST[idx] = false;
      storeList.erase(store_list_it++);
    } else if (!validLFST[idx] && younger) {
      storeList.erase(store_list_it++);
    }
  }
};

void StoreSetMDP::clear() {
  SSIT.clear();
  for (int i = 0; i < LFSTSize; ++i) {
    validLFST[i] = false;
  }
  storeList.clear();
};
}; // namespace o3
} // namespace gem5
