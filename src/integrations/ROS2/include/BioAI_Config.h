#ifndef BIOAI_CONFIG_H
#define BIOAI_CONFIG_H

#include <stdint.h>

// --- TIER DEFINITIONS ---

/* ----------------------------------------------------------------------------
   TIER 1: IOT / EMBEDDED (8-Bit Architecture)
   ----------------------------------------------------------------------------
*/
#if defined(ARDUINO) || defined(__AVR__) || defined(BIOAI_TIER_IOT)
#define BIOAI_MODE_NAME "IoT (8-Bit)"
#define TRACE_STEPS 2
#define STM_CAPACITY 2
#define LTM_CONSOLIDATE_HITS 1
#define MAX_INPUTS_PER_STEP 2
#define MAX_SIM_DEPTH 2
#define MAX_PLAN_STEPS 8
#define MAX_SYNAPSES 16  
typedef float Real;
typedef uint8_t Index;
typedef int16_t HashIdx;

/* ----------------------------------------------------------------------------
   TIER 2: SMARTHOME / EDGE (16/32-Bit Architecture)
   ----------------------------------------------------------------------------
*/
#elif defined(ESP32) || defined(BIOAI_TIER_SMARTHOME)
#define BIOAI_MODE_NAME "SmartHome (16-Bit)"
#define TRACE_STEPS 16          
#define STM_CAPACITY 6          
#define LTM_CONSOLIDATE_HITS 2  
#define MAX_INPUTS_PER_STEP 8   
#define MAX_SIM_DEPTH 5         
#define MAX_PLAN_STEPS 64       
#define MAX_SYNAPSES 64
typedef float Real;
typedef uint16_t Index;
typedef int32_t HashIdx;

/* ----------------------------------------------------------------------------
   TIER 4: NEXT (High-End 64-Bit / Double Precision)
   Definition: Verdoppelt die Kapazitäten des Ultra-Tiers.
   ----------------------------------------------------------------------------
*/
#elif defined(BIOAI_TIER_NEXT)
#define BIOAI_MODE_NAME "Next (64-Bit)"
#define TRACE_STEPS 128          
#define STM_CAPACITY 24         
#define LTM_CONSOLIDATE_HITS 4  
#define MAX_INPUTS_PER_STEP 64  
#define MAX_SIM_DEPTH 20        
#define MAX_PLAN_STEPS 1024      
#define MAX_SYNAPSES 512
typedef double Real;     // Höhere Präzision für Next
typedef uint64_t Index;  // Maximaler Adressraum
typedef int64_t HashIdx;

/* ----------------------------------------------------------------------------
   TIER 3: ULTRA (32-Bit Windows/Linux Default)
   ----------------------------------------------------------------------------
*/
#else // Windows / Linux Default
#define BIOAI_MODE_NAME "Ultra (32-Bit Index)"
#define TRACE_STEPS 64          
#define STM_CAPACITY 12         
#define LTM_CONSOLIDATE_HITS 2  
#define MAX_INPUTS_PER_STEP 32  
#define MAX_SIM_DEPTH 10        
#define MAX_PLAN_STEPS 512      
#define MAX_SYNAPSES 256
typedef float Real;
typedef uint32_t Index;
typedef int32_t HashIdx;
#endif

// --- CLUSTER MASKS ---
#define CLUSTER_OBJECT      0x1000000000000000ULL
#define CLUSTER_ACTION      0x2000000000000000ULL
#define CLUSTER_TIME        0x3000000000000000ULL
#define CLUSTER_LOGIC       0x4000000000000000ULL
#define CLUSTER_SELF        0x5000000000000000ULL
#define SUB_LOGIC_REFLEX    0x4010000000000000ULL
#define IS_REFLEX(t)        ((t & 0xFF10000000000000ULL) == SUB_LOGIC_REFLEX)

// --- DLL EXPORT MACROS ---
#ifdef _WIN32
#ifdef BIOAI_EXPORTS
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT __attribute__((visibility("default")))
#endif

#endif // BIOAI_CONFIG_H