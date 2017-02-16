/*
 * Created on: Dec, 2016
 * Author: dongchen@ini.uzh.ch
 */

#ifndef LEARNINGFILTER_H_
#define LEARNINGFILTER_H_

//for visualizer
#define VMIN 0
#define VMAX 20

#define DELTA_WEIGHT_LUT_LENGTH 80 //800 //80
#define SYNAPSE_UPGRADE_THRESHOLD_LUT_LENGTH 128 //800 //80
#define SPIKE_QUEUE_LENGTH 500
#define SPIKE_QUEUE_WIDTH 2
#define MAXIMUM_CONSIDERED_SPIKE_DELAY 800 //80 us 
#define MINIMUM_CONSIDERED_SPIKE_NUM 100 //10

//for configuring the deep spiking neural network on chip
#define MEMORY_NEURON_ADDR_OFFSET 1024

#define TOTAL_NEURON_NUM_IN_CORE 256
#define TOTAL_NEURON_NUM_ON_CHIP 1024
#define TOTAL_NEURON_NUM_ON_BOARD (TOTAL_NEURON_NUM_ON_CHIP * 4)
#define TOTAL_CAM_NUM 64
#define TOTAL_CAM_NUM_LEARNING 63 //60
#define TOTAL_SRAM_NUM 4
#define MAXIMUM_FILTER_SIZE 17*17
#define FILTER_MAP_SIZE_WIDTH 1
#define CAM_SIZE_WIDTH 1

#define VIRTUAL_SYNAPSE 0
#define REAL_SYNAPSE 1
#define EXTERNAL_REAL_SYNAPSE 2

#define VIRTUAL_CHIP_ID 0
#define CHIP_UP_LEFT_ID 1
#define CHIP_UP_RIGHT_ID 2
#define CHIP_DOWN_LEFT_ID 3
#define CHIP_DOWN_RIGHT_ID 4

#define CORE_UP_LEFT_ID 0
#define CORE_UP_RIGHT_ID 1
#define CORE_DOWN_LEFT_ID 2
#define CORE_DOWN_RIGHT_ID 3

#define INPUT_L 32
#define INPUT_W 32
#define INPUT_N (INPUT_L * INPUT_W)

#define FILTER1_L 17
#define FILTER1_W 17
#define FILTER1_N (FILTER1_L * FILTER1_W)
#define FEATURE1_L 16
#define FEATURE1_W 16
#define FEATURE1_N (FEATURE1_L * FEATURE1_W)
#define FEATURE1_LAYERS_N 3
#define FEATURE1_CAM_INHIBITORY_N (FEATURE1_LAYERS_N - 1)

#define VISUALIZER_HEIGHT_FEATURE (FILTER1_L * FEATURE1_L * (4/2))+32*2-2 //FEATURE1_LAYERS_N
#define VISUALIZER_WIDTH_FEATURE (FILTER1_W * FEATURE1_W * (4/2))+32*2-2 //FEATURE1_LAYERS_N

#define VISUALIZER_HEIGHT_OUTPUT (FEATURE1_L * FEATURE1_LAYERS_N) //FEATURE1_LAYERS_N
#define VISUALIZER_WIDTH_OUTPUT (FEATURE1_W * OUTPUT2_N) //FEATURE1_LAYERS_N

#define POOLING1_L 8
#define POOLING1_W 8
#define POOLING1_N (POOLING1_L*POOLING1_W)
#define POOLING1_LAYERS_N 4
#define POOLING1_CAM_INHIBITORY_N (POOLING1_LAYERS_N - 1)

#define FILTER2_L 5
#define FILTER2_W 5
#define FILTER2_N (FILTER2_L * FILTER2_W)
#define FEATURE2_L 4
#define FEATURE2_W 4
#define FEATURE2_N (FEATURE2_L * FEATURE2_W)
#define FEATURE2_LAYERS_N 32
#define FEATURE2_CAM_INHIBITORY_N (FEATURE2_LAYERS_N - 1)

#define POOLING2_L 2
#define POOLING2_W 2
#define POOLING2_N (POOLING2_L*POOLING2_W)
#define POOLING2_LAYERS_N 32
#define POOLING2_CAM_INHIBITORY_N (POOLING2_LAYERS_N - 1)

#define OUTPUT1_N 512
#define OUTPUT2_N 4 //3 or 4

//for encoding the chip input commands
#define CXQ_PROGRAM (1 << 17) // (0x80 << 10)
#define CXQ_EVENT (0x8 << 10)

#define CXQ_CHIPID_SHIFT 30
#define CXQ_ADDR_SHIFT 20
#define CXQ_SOURCE_CORE_SHIFT 18
#define CXQ_COREID_SHIFT 8
#define CXQ_ROW_SHIFT 4
#define CXQ_COLUMN_SHIFT 0

#define CXQ_SRAM_VIRTUAL_SOURCE_CORE_SHIFT 28
#define CXQ_SRAM_SY_SHIFT 27
#define CXQ_SRAM_DY_SHIFT 25
#define CXQ_SRAM_SX_SHIFT 24
#define CXQ_SRAM_DX_SHIFT 22
#define CXQ_SRAM_DEST_CORE_SHIFT 18

#define CXQ_PROGRAM_COREID_SHIFT 15
#define CXQ_PROGRAM_ROW_SHIFT 5
#define CXQ_PROGRAM_COLUMN_SHIFT 0

#define CXQ_CAM_EI_SHIFT 29
#define CXQ_CAM_FS_SHIFT 28
#define CXQ_EVENT_SY 9
#define CXQ_EVENT_DY 7
#define CXQ_EVENT_SX 6
#define CXQ_EVENT_DX 4
#define CXQ_EVENT_CORE_D 0

#define INPUT_CHIP_ID 1
#define INPUT_CHIP_SY 0
#define INPUT_CHIP_DY 1
#define INPUT_CHIP_SX 0
#define INPUT_CHIP_DX 0

#define NEURON_ADDRESS_BITS 0xff
#define NEURON_COREID_BITS 0x300
#define NEURON_COREID_SHIFT 8
#define NEURON_CHIPID_SHIFT 10

#define NEURON_ROW_BITS 0xf0
#define NEURON_COL_BITS 0xf
#define NEURON_ROW_SHIFT 4
#define CAM_NEURON_ROW_SHIFT 6

#define FAST_SYNAPSE_ID 2
#define SLOW_SYNAPSE_ID 1
#define NO_SYNAPSE_ID 0
#define NO_SYNAPSE_ADDRESS 0
#define NO_SYNAPSE_CORE 0
#define EXCITATORY_SYNAPSE_SIGN 1
#define EXCITATORY_FAST_SYNAPSE_ID (EXCITATORY_SYNAPSE_SIGN * FAST_SYNAPSE_ID)
#define EXCITATORY_SLOW_SYNAPSE_ID (EXCITATORY_SYNAPSE_SIGN * SLOW_SYNAPSE_ID)
#define EXCITATORY_SYNAPSE 1
#define INHIBITORY_SYNAPSE 0
#define FAST_SYNAPSE 1
#define SLOW_SYNAPSE 0

#define BOARD_HEIGHT 2
#define BOARD_WIDTH 2

#define EVENT_DIRECTION_Y_UP 0
#define EVENT_DIRECTION_Y_DOWN 1
#define EVENT_DIRECTION_X_RIGHT 0
#define EVENT_DIRECTION_X_LEFT 1

#define SRAM_NEURON_ROW_SHIFT 6
#define SRAM_NEURON_COL_SHIFT 2
#define SRAM_COL_VALUE 16

#include "main.h"
#include "modules/ini/dynapse_common.h"

#include <libcaer/events/spike.h>
#include <libcaer/events/frame.h> //display

void caerLearningFilter(uint16_t moduleID, caerSpikeEventPacket spike,
		caerFrameEventPacket *weightplotfeature, caerFrameEventPacket *synapseplotfeature);

#endif /* LEARNINGFILTER_H_ */
