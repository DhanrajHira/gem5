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

#include <algorithm>
#include <cassert>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst.hh"

namespace gem5
{

namespace o3
{

StoreVector::StoreVector(uint64_t clear_period, int _SSIT_size, int _LFST_size)
{
    init(clear_period, _SSIT_size, _LFST_size);
}

StoreVector::~StoreVector()
{
}

void
StoreVector::init(uint64_t clear_period, int _SSIT_size, int _LFST_size)
{
    clear_period = clear_period;
    if (!isPowerOf2(_SSIT_size))
        fatal("SVT Size (set by SSIT size) must be a power of 2!\n");
    SVTSize = _SSIT_size;
    SVTVectorSize = _LFST_size;
    SVT.reserve(SVTSize);
    for (auto i = 0; i < SVTSize; i++)
        SVT.push_back(std::vector<bool>(_LFST_size));
    memOpsPred = 0;
    storeAddrs = CircularQueue<Addr>(_LFST_size);
    storeSeqNums = CircularQueue<Addr>(_LFST_size);
}
std::vector<bool> &StoreVector::getStoreVector(Addr load_PC) {
     auto SVT_idx = (load_PC & (SVTSize - 1));
     assert(SVT_idx < SVTSize);
     return SVT[SVT_idx];
}

void
StoreVector::violation(Addr store_PC, Addr load_PC)
{
    auto violating_store_iter = std::find(
            storeAddrs.begin(), storeAddrs.end(), store_PC);
    if (violating_store_iter == storeAddrs.end())
        return;

    auto violating_store_offset = violating_store_iter - storeAddrs.begin();
    assert(violating_store_offset < SVTVectorSize);
    auto store_vector = getStoreVector(load_PC);
    store_vector[violating_store_offset] = true;
}

void
StoreVector::checkClear()
{
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
    storeAddrs.push_back(store_PC);
    storeSeqNums.push_back(store_seq_num);
}

void
StoreVector::checkInst(const DynInstPtr &inst,
        std::vector<InstSeqNum> &producing_stores)
{
    if (inst->isLoad())
        return;
    auto SV = getStoreVector(inst->pcState().instAddr());
    for (auto i = 0; i < SVTVectorSize; i++) {
        bool does_depend = SV[i];
        if (!does_depend)
            continue;
        producing_stores.push_back(storeSeqNums[i]);
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
    auto seq_num_iter = storeSeqNums.begin();
    auto seq_num_end = storeSeqNums.end();
    auto addr_iter = storeAddrs.begin();
    while (seq_num_iter != seq_num_end) {
        if (*seq_num_iter > squashed_num)
            break;
        *(seq_num_iter++) = 0;
        *(addr_iter++) = 0;
    }
}

void
StoreVector::clear()
{
    storeAddrs.flush();
    storeSeqNums.flush();
}

void
StoreVector::dump()
{
}

} // namespace o3
} // namespace gem5
