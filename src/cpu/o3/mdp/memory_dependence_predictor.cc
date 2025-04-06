#include "cpu/o3/mdp/memory_dependence_predictor.hh"

namespace gem5 {
namespace o3 {
MemoryDependencePredictor::MemoryDependencePredictor(
    const MemoryDependencePredictorParams &params)
    : SimObject(params) {}
} // namespace o3
} // namespace gem5
