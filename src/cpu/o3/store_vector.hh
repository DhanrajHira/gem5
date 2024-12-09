/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __CPU_O3_STORE_VECTOR_HH__
#define __CPU_O3_STORE_VECTOR_HH__

#include <cstdint>
#include <vector>

#include "base/circular_queue.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/dyn_inst_ptr.hh"

namespace gem5
{

namespace o3
{

/**
 * Implements a store set predictor for determining if memory
 * instructions are dependent upon each other.  See paper "Memory
 * Dependence Prediction using Store Sets" by Chrysos and Emer.  SSID
 * stands for Store Set ID, SSIT stands for Store Set ID Table, and
 * LFST is Last Fetched Store Table.
 */
class StoreVector
{
  public:
    /** Default constructor.  init() must be called prior to use. */
    StoreVector() { };

    /** Creates store set predictor with given table sizes. */
    StoreVector(uint64_t clear_period, int _SVT_size, int _SVT_vector_size);

    /** Default destructor. */
    ~StoreVector();

    /** Initializes the store set predictor with the given table sizes. */
    void init(uint64_t clear_period, int _SVT_size, int _SVT_vector_size);

    /** Records a memory ordering violation between the younger load
     * and the older store. */
    void violation(DynInstPtr store, DynInstPtr load, size_t cur_SQ_tail);

    /** Clears the store set predictor every so often so that all the
     * entries aren't used and stores are constantly predicted as
     * conflicting.
     */
    void checkClear();

    /** Inserts a load into the store set predictor.  This does nothing but
     * is included in case other predictors require a similar function.
     */
    void insertLoad(Addr load_PC, InstSeqNum load_seq_num);

    /** Inserts a store into the store set predictor.  Updates the
     * LFST if the store has a valid SSID. */
    void insertStore(Addr store_PC, InstSeqNum store_seq_num, ThreadID tid);

    /** Checks if the instruction with the given PC is dependent upon
     * any store.  @return Returns the sequence number of the store
     * instruction this PC is dependent upon.  Returns 0 if none.
     */
    void checkInst(const DynInstPtr &inst, size_t head_idx, size_t tail_idx,
            std::vector<InstSeqNum> &producing_stores);

    /** Records this PC/sequence number as issued. */
    void issued(Addr issued_PC, InstSeqNum issued_seq_num, bool is_store);

    /** Squashes for a specific thread until the given sequence number. */
    void squash(InstSeqNum squashed_num, ThreadID tid);

    /** Resets all tables. */
    void clear();

    /** Debug function to dump the contents of the store list. */
    void dump();

  private:
    size_t getSVIdx(Addr load_PC);

    std::vector<std::vector<char>> SVT;

    /** Number of loads/stores to process before wiping predictor so all
     * entries don't get saturated
     */
    uint64_t clearPeriod;

    uint64_t SVTSize;

    uint64_t SVTVectorSize;

    /** Number of memory operations predicted since last clear of predictor */
    int memOpsPred;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_STORE_VECTOR_HH__
