#ifndef BIOAI_INTERFACE_H
#define BIOAI_INTERFACE_H

#include "BioAI_Structs.h"

#ifdef __cplusplus
extern "C" {
#endif

	// --- Kern-Logik (Intern) ---
	BioBrain* internal_create_brain(uint64_t key);
	void internal_free_brain(BioBrain* brain);
	Neuron* internal_get_neuron(BioBrain* brain, TokenID input_id);
	void internal_load_plan(BioBrain* brain, const TokenID* steps, int count, uint8_t strict);
	void internal_ltm_commit(BioBrain* brain, Neuron* n, TokenID action, Real delta, Real salt);

	TokenID bio_think_logic(BioBrain* brain, const TokenID* inputs, int count);
	TokenID bio_simulate_logic(BioBrain* brain, const TokenID* inputs, int count, int depth);
	void bio_learn_logic(BioBrain* brain, Real reward, TokenID action);

	// --- Serialisierung ---
	void* internal_serialize(const BioBrain* b, int* outSize);
	BioBrain* internal_deserialize(const void* data, int size);

	// --- Öffentliche API (Exportiert) ---
	EXPORT void* API_CreateBrain(uint64_t key);
	EXPORT void  API_FreeBrain(void* brainPtr);
	EXPORT void  API_SetMode(void* brainPtr, int mode);
	EXPORT void  API_LoadPlan(void* brainPtr, const uint64_t* steps, int count, int strict);
	EXPORT void  API_AbortPlan(void* brainPtr);
	EXPORT int   API_GetPlanStatus(void* brainPtr);
	EXPORT uint64_t API_Update(void* brainPtr, const uint64_t* inputs, int count);
	EXPORT uint64_t API_Simulate(void* brainPtr, const uint64_t* inputs, int count, int depth);
	EXPORT void  API_Feedback(void* brainPtr, float reward, uint64_t action);
	EXPORT void  API_Teach(void* brainPtr, uint64_t input, uint64_t action, float weight);
	EXPORT float API_Inspect(void* brainPtr, uint64_t input, uint64_t action);
	EXPORT void* API_Serialize(const void* brainPtr, int* outSize);
	EXPORT void* API_Deserialize(const void* data, int size);
	EXPORT void  API_FreeBuffer(void* buffer);

#ifdef __cplusplus
}
#endif

#endif // BIOAI_INTERFACE_H