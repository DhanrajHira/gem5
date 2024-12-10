/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/o3/store_vector.hh"

#include <cassert>
#include <cstddef>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "debug/StoreVector.hh"

namespace gem5
{

namespace o3
{

StoreVector::StoreVector(uint64_t clear_period,
        int load_queue_size, int store_queue_size)
{
    init(clear_period, load_queue_size, store_queue_size);
}

StoreVector::~StoreVector()
{
}

void
StoreVector::init(uint64_t clear_period,
        int load_queue_size, int store_queue_size)
{
    clearPeriod = clear_period;
    if (!isPowerOf2(load_queue_size))
        fatal("StoreVector requires that LQSize be a power of 2\n");
    SVTSize = load_queue_size;
    SVTVectorSize = store_queue_size;
    SVT.reserve(SVTSize);
    for (auto i = 0; i < load_queue_size; i++)
        SVT.push_back(std::vector<char>(store_queue_size));
    memOpsPred = 0;
}

size_t StoreVector::getSVIdx(Addr load_PC)
{
    return (load_PC & (SVTSize - 1));
}

void
StoreVector::violation(DynInstPtr store, DynInstPtr violating_load,
        size_t cur_SQ_tail)
{
    assert(store->isStore());
    auto violating_store_offset = cur_SQ_tail - store->sqIdx;
    auto store_PC = store->pcState().instAddr();
    auto load_PC = violating_load->pcState().instAddr();
    auto SV_index = getSVIdx(load_PC);
    DPRINTF(StoreVector, "load(PC=%#x,idx=%lu,seqNum=%lu) violated "
            "store(PC=%#x,offset=%lu,seqNum=%lu)\n",
            load_PC, SV_index, violating_load->seqNum,
            store_PC, violating_store_offset, store->seqNum);
    if (violating_store_offset >= SVTVectorSize)
        fatal("violating_store_offset >= SVTVectorSize");
    auto &store_vector = SVT[SV_index];
    store_vector[violating_store_offset] = 1;
    // DPRINTF(StoreVector, "Bit %lu @ %p in SV set to %d\n",
    //         violating_store_offset, &store_vector[violating_store_offset],
    //         store_vector[violating_store_offset]);
    // for (int i = 0; i < SVTVectorSize; i++)
    //     store_vector[i] = 1;
}

void
StoreVector::checkClear()
{
    memOpsPred++;
    if (memOpsPred > clearPeriod) {
        DPRINTF(StoreVector,
                "Wiping predictor state beacuse %d ld/st executed\n",
                clearPeriod);
        memOpsPred = 0;
        clear();
    }
}

void
StoreVector::insertLoad(Addr load_PC, InstSeqNum load_seq_num)
{
    checkClear();
    // Does nothing.
    return;
}

void
StoreVector::insertStore(Addr store_PC, InstSeqNum store_seq_num, ThreadID tid)
{
    checkClear();
    // Do nothing
}

void
StoreVector::checkInst(const DynInstPtr &inst, size_t head_idx,
        size_t tail_idx, std::vector<InstSeqNum> &producing_stores)
{
    if (!inst->isLoad())
        return;

    auto load_PC = inst->pcState().instAddr();
    auto SV_idx = getSVIdx(load_PC);
    // DPRINTF(StoreVector,
    //         "Checking load with PC=%#x for dependencies\n", load_PC);

    // Nothing in the SQ, no point in predicting.
    if (head_idx > tail_idx) {
        DPRINTF(StoreVector, "Skipping load(PC=%#x,idx=%lu) as SQ "
                "is empty\n", load_PC, SV_idx);
        return;
    }

    // DPRINTF(StoreVector,
    //         "Got headIdx=%lu and tail=%lu for SQ\n", head_idx, tail_idx);
    // assert(inst->sqIdx == tail_idx);

    // Iter from tail to head, check the vector to see if it is dependent.
    auto sq_entry = inst->sqIt;  // This is the end iter
    --sq_entry; // we need to get the iter to the last element.
    assert(sq_entry.idx() == tail_idx);
    const auto &SV = SVT[SV_idx];
    for (auto i = 0; i < SVTVectorSize && (tail_idx > head_idx);
            i++, --sq_entry, --tail_idx) {
        bool does_depend = SV[i];
        if (!does_depend) {
            // DPRINTF(StoreVector,
            //         "Skipping %d because SV[%d] @ %p = %d\n",
            //         i, i, &SV[i], SV[i]);
            continue;
        }
        InstSeqNum producing_store = sq_entry->instruction()->seqNum;
        Addr producing_addr = sq_entry->instruction()->pcState().instAddr();
        DPRINTF(StoreVector,
                "Predicting that load(PC=%#x,idx=%lu,seqNum=%lu) "
                "depends on store(PC=%#x,offset=%d,seqNum=%lu)\n",
                load_PC, SV_idx, inst->seqNum, producing_addr, i,
                producing_store);
        producing_stores.push_back(producing_store);
    }
}

void
StoreVector::issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store)
{
    // do nothing
}

void
StoreVector::squash(InstSeqNum squashed_num, ThreadID tid)
{
    // do nothing
}

void
StoreVector::clear()
{
    for (std::vector<char> &SV : SVT)
        for (int i = 0; i < SVTVectorSize; ++i)
            SV[i] = 0;
}

void
StoreVector::dump()
{
}

} // namespace o3
} // namespace gem5
