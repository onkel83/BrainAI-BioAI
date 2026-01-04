#ifndef BIOAI_STRUCTS_H
#define BIOAI_STRUCTS_H

#include "BioAI_Config.h"

typedef uint64_t TokenID;

typedef struct {
    TokenID target_action;
    Real weight;
} Synapse;

typedef struct {
    TokenID target_action;
    Real weight;
    uint16_t hits;
} Candidate;

typedef struct {
    TokenID action_context;
    TokenID next_concept;
    Real confidence;
} Prediction;

typedef struct {
    TokenID input_concept;
    Synapse* ltm; Index ltm_count; Index ltm_capacity;
    Candidate* stm; Index stm_count; Index stm_capacity;
    Prediction* predictions; Index pred_count; Index pred_capacity;
} Neuron;

typedef struct {
    TokenID inputs[MAX_INPUTS_PER_STEP];
    uint8_t input_count;
    TokenID action;
} TraceEntry;

typedef struct {
    TraceEntry buffer[TRACE_STEPS];
    uint8_t head;
    uint8_t count;
} HebbianTrace;

typedef struct {
    TokenID steps[MAX_PLAN_STEPS];
    uint16_t length;
    uint16_t current_step;
    uint8_t active;
    uint8_t strict;
} BioPlan;

typedef struct {
    Neuron* neurons;
    Index neuron_count;
    Index neuron_capacity;

    HashIdx* hash_table;
    Index hash_capacity;
    Index hash_mask;

    HebbianTrace trace;
    BioPlan active_plan;
    Real learning_rate;
    Real decay_step;
    uint64_t license_key;
    uint8_t fixed_structure;
} BioBrain;

#endif // BIOAI_STRUCTS_H