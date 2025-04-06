from m5.objects.SimObject import SimObject

class MemoryDependencePredictor(SimObject):
    type = "MemoryDependencePredictor"
    abstract = True
    cxx_header = "cpu/o3/mdp/memory_dependence_predictor.hh"
    cxx_class  = "gem5::o3::MemoryDependencePredictor"

class StoreSetMDP(MemoryDependencePredictor):
    type = "StoreSetMDP"
    cxx_header = "cpu/o3/mdp/store_set.hh"
    cxx_class  = "gem5::o3::StoreSetMDP"

