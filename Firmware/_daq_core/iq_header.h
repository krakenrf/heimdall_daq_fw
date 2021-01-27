/*
 * Desctiption: IQ Frame header definition
 *	For header field description check the corresponding documentation
 *	Project: HeIMDALL DAQ Firmware
 *	Author: Tamás Pető
 */

#define __STDC_FORMAT_MACROS
#include <stdio.h>
#include <inttypes.h>

#define FRAME_TYPE_DATA  0
#define FRAME_TYPE_DUMMY 1
#define FRAME_TYPE_RAMP  2
#define FRAME_TYPE_CAL   3
#define FRAME_TYPE_TRIGW 4

#define SYNC_WORD 0x2bf7b95a

#define IQ_HEADER_LENGTH 1024
struct iq_frame_struct 
{
	struct iq_header_struct* header;
	uint8_t* payload;
	int payload_size; // Used when the channel buffer is not equal to the CPI size
};

struct iq_frame_struct_32 // Complex float 32 compatible
{
	struct iq_header_struct* header;
	float* payload; //TODO: Modifiy this to CF32 type
	int payload_size; // Used when the channel buffer is not equal to the CPI size
};

struct iq_header_struct {
	uint32_t sync_word;            //Updates: RTL-DAQ - Static   
	uint32_t frame_type;           //Updates: RTL-DAQ - Static
	char hardware_id [16];         //Updates: RTL-DAQ - Static
	uint32_t unit_id;              //Updates: RTL-DAQ - Static
	uint32_t active_ant_chs;       //Updates: RTL-DAQ - Static
	uint32_t ioo_type;             //Updates: RTL-DAQ - Static
	uint64_t rf_center_freq;       //Updates: RTL-DAQ - Static
	uint64_t adc_sampling_freq;    //Updates: RTL-DAQ - Static
	uint64_t sampling_freq;        //Updates: Decimator - Static
	uint32_t cpi_length;           //Updates: Rebuffer / Decimator
	uint64_t time_stamp;           //Updates: RTL-DAQ
	uint32_t daq_block_index;      //Updates: RTL-DAQ      
	uint32_t cpi_index;            //Updates: Decimator
	uint64_t ext_integration_cntr; //Updates: RTL-DAQ   
	uint32_t data_type;            //Updates: Decimator - Static
	uint32_t sample_bit_depth;     //Updates: RTL-DAQ->Decimator - Static
	uint32_t adc_overdrive_flags;  //Updates: RTL-DAQ -> Rebuffer
	uint32_t if_gains[32];         //Updates: RTL-DAQ
	uint32_t delay_sync_flag;      //Updates: Delay synchronizer
	uint32_t iq_sync_flag;         //Updates: Delay synchronizer
	uint32_t sync_state;           //Updates: Delay synchronizer
	uint32_t noise_source_state;   //Updates: RTL-DAQ	
	uint32_t reserved[192];        //Updates: RTL-DAQ - Static
	uint32_t header_version;       //Updates: RTL-DAQ - Static   
};
void dump_iq_header(struct iq_header_struct* iq_header);
int check_sync_word(struct iq_header_struct* iq_header);
