#include "cpu/o3/store_wait_table.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"
#include "debug/StoreWaitTable.hh"

namespace gem5
{

namespace o3
{

StoreWaitTable::StoreWaitTable(uint64_t clear_period,
        int _SSIT_size, int _LFST_size)
    : clearPeriod(clear_period)
{
    DPRINTF(StoreWaitTable, "StoreWaitTable: Creating store set object.\n");

    memOpsPred = 0;
}

StoreWaitTable::~StoreWaitTable()
{
}

void
StoreWaitTable::init(uint64_t clear_period, int _SSIT_size, int _LFST_size)
{
    clearPeriod = clear_period;

    DPRINTF(StoreWaitTable, "StoreWaitTable: Creating store set object.\n");

    memOpsPred = 0;
}


void
StoreWaitTable::violation(Addr store_PC, Addr load_PC)
{
    // Start keeping track of this load to not let it issue OOO
    // with all the older stores.
    DPRINTF(StoreWaitTable, "load(PC=%#x) violated store(PC=%#x)\n",
            load_PC, store_PC);
    known_violating_loads.insert(load_PC);
}

void
StoreWaitTable::checkClear()
{
    memOpsPred++;
    if (memOpsPred > clearPeriod) {
        DPRINTF(StoreWaitTable,
                "Wiping predictor state beacuse %d ld/st executed\n",
                clearPeriod);
        memOpsPred = 0;
        clear();
    }
}

void
StoreWaitTable::insertLoad(Addr load_PC, InstSeqNum load_seq_num)
{
    checkClear();
    // Does nothing.
    return;
}

void StoreWaitTable::insertStore(Addr store_PC, InstSeqNum store_seq_num,
        ThreadID tid)
{
    checkClear();
    unissued_stores.insert(store_seq_num);
}

void StoreWaitTable::checkInst(const DynInstPtr &inst,
        std::vector<InstSeqNum> &producing_stores)
{
    // For store wait table, we don't introduce dependencies between stores
    if (!inst->isLoad())
        return;

    // This is not a known violator.
    if (known_violating_loads.find(inst->pcState().instAddr())
            == known_violating_loads.end())
        return;

    InstSeqNum load_seq_num = inst->seqNum;
    for (InstSeqNum store_seq_num : unissued_stores) {
        if (load_seq_num > store_seq_num) {
        DPRINTF(StoreWaitTable,
                "Predicted load(PC=%#x,seq=%lu) depends on store(seq=%lu)\n",
                inst->pcState().instAddr(), inst->seqNum, store_seq_num);
            producing_stores.push_back(store_seq_num);
        }
    }
}

void StoreWaitTable::issued(Addr issued_PC, InstSeqNum issued_seq_num,
        bool is_store)
{
    if (!is_store)
        return;
    SeqNumMapIt unissued_store = unissued_stores.find(issued_seq_num);
    if (unissued_store != unissued_stores.end())
        unissued_stores.erase(unissued_store);
}

void
StoreWaitTable::squash(InstSeqNum squashed_num, ThreadID tid)
{
    //@todo:Fix to only delete from correct thread
    auto erase_until = unissued_stores.lower_bound(squashed_num + 1);
    unissued_stores.erase(unissued_stores.begin(), erase_until);
}

void
StoreWaitTable::clear()
{
    known_violating_loads.clear();
    unissued_stores.clear();
}

void
StoreWaitTable::dump()
{
    cprintf("unissued_stores.size(): %i\n", unissued_stores.size());
    SeqNumMapIt store_list_it = unissued_stores.begin();

    int num = 0;

    while (store_list_it != unissued_stores.end()) {
        cprintf("%i: [sn:%lli]\n",
                num, *store_list_it);
        num++;
        store_list_it++;
    }
}

} // namespace o3
} // namespace gem5
