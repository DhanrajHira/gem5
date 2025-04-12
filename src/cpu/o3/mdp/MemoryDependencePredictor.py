from m5.objects.SimObject import SimObject
from m5.objects.IndexingPolicies import *
from m5.objects.ReplacementPolicies import *
from m5.params import *
from m5.proxy import *
from m5.SimObject import *

class MemoryDependencePredictor(SimObject):
    type = "MemoryDependencePredictor"
    abstract = True
    cxx_header = "cpu/o3/mdp/memory_dependence_predictor.hh"
    cxx_class  = "gem5::o3::MemoryDependencePredictor"

class StoreSetMDP(MemoryDependencePredictor):
    type = "StoreSetMDP"
    cxx_header = "cpu/o3/mdp/store_set.hh"
    cxx_class  = "gem5::o3::StoreSetMDP"

    clear_period = Param.Unsigned(
        250000,
        "Number of load/store insts before the dep predictor "
        "should be invalidated",
    )

    LFSTSize = Param.Unsigned(1024, "Last fetched store table size")
    SSITSize = Param.MemorySize("1024", "Store set ID table size")
    SSITAssoc = Param.Unsigned(1, "SSIT table associativity")
    SSITReplPolicy = Param.BaseReplacementPolicy(
        LRURP(), "SSIT replacement policy"
    )
    SSITIndexingPolicy = Param.BaseIndexingPolicy(
        SetAssociative(
            size=Parent.SSITSize * 4,
            assoc=Parent.SSITAssoc,
            entry_size=4,
        ),
        "SSIT indexing policy",
    )

class StoreVectorMDP(MemoryDependencePredictor):
    type = "StoreVectorMDP"
    cxx_header = "cpu/o3/mdp/store_vector.hh"
    cxx_class = "gem5::o3::StoreVectorMDP"

    clearPeriod = Param.Unsigned(
        250000,
        "Number of load/store insts before the dep predictor "
        "should be invalidated",
    )

    SVTSize = Param.Unsigned(128, "Size of the Store Vector Table")
    SVTVectorSize = Param.Unsigned(16, "Size of the Store Vectors")
    SQSize = Param.Unsigned(
            Parent.SQEntries, "Size of the store queue, this should not be set directly but instead should be read from the parent."
    )

