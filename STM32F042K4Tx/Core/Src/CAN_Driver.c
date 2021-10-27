#include "CAN_Driver.h"

/*
 * some info on the CAN peripheral:
 * 	- uses byte array for data section of the frame
 * 	- has structs for tx and rx frames
 * 	- has structs for filter settings (must be configured for peripheral to work)
 * 	- must call HAL_CAN_Start(&hcan); to start peripheral
 * 	- can set MCR register to 0x60 to allow for CAN peripheral to work while in debug mode
 * 	- uses a mailbox variable to figure out how full each fifo is
 * 	- settings on can bit timings need to be configured on the .ioc file, use this website to
 * 	  figure out the values: http://www.bittiming.can-wiki.info/
 *  - to send a CAN msg, use the following code:
 * 		HAL_CAN_AddTxMessage(&hcan, &TxHeader, Data, &TxMailBox );  // load message to mailbox
 *		while (HAL_CAN_IsTxMessagePending( &hcan, TxMailBox)); 		//waiting till message gets through
 *	- to poll the CAN peripheral for new msgs use this code:
 *		if (HAL_CAN_GetRxFifoFillLevel(&hcan, fifo ) !=0) { 		// check if mail box is not empty
 *			HAL_CAN_GetRxMessage(&hcan, fifo, &RxHeader, Data); 	// copy frame data to RX header
 *			//code to do something with the frame
 *		}
 *	- instead of polling, once can use the RxFifoMsgPendingCallback interrupt that will get called
 *	  when a new message has entered the fifo (recommend setting up the filter properly for this)
 */


void data2bytes(void *val, uint8_t *bytes_array, uint8_t type, uint8_t is_float) {

	switch (type)
	{
		case BIT8: ;
			// Create union of shared memory space
			union {
				uint8_t val_variable;
				uint8_t temp_array[8];
			} u;
			u.val_variable = *(uint8_t*)val;

			// Assign bytes to input array
			memcpy(bytes_array, u.temp_array, type);
			break;

		case BIT16: ;
			// Create union of shared memory space
			union {
				uint16_t val_variable;
				uint8_t temp_array[8];
			} v;
			v.val_variable = *(uint16_t*)val;

			// Assign bytes to input array
			memcpy(bytes_array, v.temp_array, type);
			break;

		case BIT32: ;
			if (is_float > 0) {
				// Create union of shared memory space
				union {
					float val_variable;
					uint8_t temp_array[8];
				} w;
				w.val_variable = *(float*)val;

				// Assign bytes to input array
				memcpy(bytes_array, w.temp_array, type);
			} else {
				// Create union of shared memory space
				union {
					uint32_t val_variable;
					uint8_t temp_array[8];
				} y;
				y.val_variable = *(uint32_t*)val;

				// Assign bytes to input array
				memcpy(bytes_array, y.temp_array, type);
			}
			break;

		case BIT64: ;
			// Create union of shared memory space
			union {
				uint64_t val_variable;
				uint8_t temp_array[8];
			} z;
			z.val_variable = *(uint64_t*)val;

			// Assign bytes to input array
			memcpy(bytes_array, z.temp_array, type);
			break;

		default:
			break;

	}

}

void add_data(void *val, uint8_t *bytes_array, uint8_t size, uint8_t is_float, uint8_t start_pos) {

	uint8_t meme[8];
	//turning val into byte_array
	data2bytes(val, meme, size, is_float);

	//adding val in LSB->MSB to data byte array
	for (uint8_t i = 0; i < size; i++) {
		bytes_array[i + start_pos] = meme[size - i];
	}

}

void bytes2data( void *val, uint8_t *bytes_array, uint8_t type, uint8_t is_float, uint8_t start_pos) {

	if (is_float > 0) {

		union {
		    float f;
		    uint32_t ul;
		 } u;

		 u.ul = (bytes_array[start_pos + 3] << 24) | (bytes_array[start_pos + 2] << 16) |
				(bytes_array[start_pos + 1] << 8)  | bytes_array[start_pos];
		 val = &u.f;
	} else {

		uint64_t meme = 0;

		for(uint8_t i = 0; i< type; i++) {
			meme |= (bytes_array[start_pos + i] << i*8);
		}

		val = &meme;
	}

}


