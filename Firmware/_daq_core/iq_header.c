/*
 * Desctiption: IQ Frame header definition
 *  For header field description check the corresponding documentation
 *	Project: HeIMDALL DAQ Firmware
 *	Author: Tamás Pető
 */

#include <stdio.h>
#include <inttypes.h>
#include "iq_header.h"

void dump_iq_header(struct iq_header_struct* iq_header){
	fprintf(stderr, "Sync word: %u\n", iq_header->sync_word);
	fprintf(stderr, "Header version: %u\n", iq_header->header_version);
	fprintf(stderr, "Frame type ID: %u\n", iq_header->frame_type);
	fprintf(stderr, "Hardware ID: %s\n", iq_header->hardware_id);
	fprintf(stderr, "Unit ID: %u\n", iq_header->unit_id);
	fprintf(stderr, "Active antenna channels: %u\n", iq_header->active_ant_chs);
	fprintf(stderr, "IoO type: %u\n", iq_header->ioo_type);
	fprintf(stderr, "RF center frequency: %.2f MHz\n", (float) iq_header->rf_center_freq/1000000);
	fprintf(stderr, "ADC sampling frequency: %.2f MHz\n", (float) iq_header->adc_sampling_freq/1000000);
	fprintf(stderr, "IQ sampling frequency: %.2f MHz\n", (float) iq_header->sampling_freq/1000000);
	fprintf(stderr, "CPI length: %u \n", iq_header->cpi_length);	
	fprintf(stderr, "Unix Epoch timestamp: %"PRIu64"\n", iq_header->time_stamp);
	fprintf(stderr, "DAQ block index %u\n", iq_header->daq_block_index);
	fprintf(stderr, "CPI index: %u \n", iq_header->cpi_index);
	fprintf(stderr, "Extended integration counter: %"PRIu64"\n", iq_header->ext_integration_cntr);
	fprintf(stderr, "Data type: %u \n", iq_header->data_type);
	fprintf(stderr, "Sample bit depth: %u \n", iq_header->sample_bit_depth);
	fprintf(stderr, "ADC overdrive flag: 0x%08X \n", iq_header->adc_overdrive_flags);
	for(int m=0;m<32;m++)
	{
	    fprintf(stderr, "Ch: %u IF gain: %u \n",m, iq_header->if_gains[m]);
	}
	fprintf(stderr, "Delay sync flag: %u \n", iq_header->delay_sync_flag);
	fprintf(stderr, "IQ sync flag: %u \n", iq_header->iq_sync_flag);
	fprintf(stderr, "Sync state: %u \n", iq_header->sync_state);
	fprintf(stderr, "Noise source state: %u \n", iq_header->noise_source_state);
}

int check_sync_word(struct iq_header_struct* iq_header)
{
	if (iq_header->sync_word != SYNC_WORD){return -1;}
	else{return 0;}
}