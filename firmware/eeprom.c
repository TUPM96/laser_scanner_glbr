/*
  eeprom.c - Implementation phương thức EEPROM
  
  CHỨC NĂNG:
  - Đọc/ghi dữ liệu từ EEPROM
  - Copy dữ liệu với checksum để đảm bảo tính toàn vẹn
  - Lưu trữ cài đặt và dữ liệu quan trọng
  - Hỗ trợ split EEPROM erase/write cho ATmega48/88/168/328
  
  LƯU Ý: File này đã được chuẩn bị cho Doxygen documentation
  Dựa trên AVR103 AppNote - Using the EEPROM Programming Modes
  
  Part of Grbl
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Based on Atmel AVR103 example code
*/

#include <avr/io.h>
#include <avr/interrupt.h>

// Các bit EEPROM có tên khác nhau trên các thiết bị khác nhau
#ifndef EEPE
		#define EEPE  EEWE  // EEPROM program/write enable
		#define EEMPE EEMWE // EEPROM master program/write enable
#endif

// Hai bit này không được định nghĩa trong device include files
#define EEPM1 5 // EEPROM Programming Mode Bit 1
#define EEPM0 4 // EEPROM Programming Mode Bit 0

// Định nghĩa để giảm kích thước code - bỏ qua SPM flag polling
#define EEPROM_IGNORE_SELFPROG

// Đọc 1 byte từ EEPROM
// LƯU Ý: CPU bị halt trong 4 clock cycles khi đọc EEPROM
// addr: địa chỉ EEPROM cần đọc
// Trả về: byte đọc được từ EEPROM
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) ); // Chờ hoàn thành write trước đó
	EEAR = addr; // Set địa chỉ EEPROM
	EECR = (1<<EERE); // Bắt đầu đọc EEPROM
	return EEDR; // Trả về byte đọc được
}

/* Ghi 1 byte vào EEPROM
 * Sự khác biệt giữa byte hiện tại và giá trị mới được dùng
 * to select the most efficient EEPROM programming mode.
 *
 * \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 * \note  When this function returns, the new EEPROM value is not available
 *        until the EEPROM programming time has passed. The EEPE bit in EECR
 *        should be polled to check whether the programming is finished.
 *
 * \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 * \param  addr  EEPROM address to write to.
 * \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	char old_value; // Old EEPROM value.
	char diff_mask; // Difference mask, i.e. old value XOR new value.

	cli(); // Ensure atomic operation for the write operation.
	
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM.
	#endif
	
	EEAR = addr; // Set EEPROM address register.
	EECR = (1<<EERE); // Start EEPROM read operation.
	old_value = EEDR; // Get old EEPROM value.
	diff_mask = old_value ^ new_value; // Get bit differences.
	
	// Check if any bits are changed to '1' in the new value.
	if( diff_mask & new_value ) {
		// Now we know that _some_ bits need to be erased to '1'.
		
		// Check if any bits in the new value are '0'.
		if( new_value != 0xff ) {
			// Now we know that some bits need to be programmed to '0' also.
			
			EEDR = new_value; // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode.
			EECR |= (1<<EEPE);  // Start Erase+Write operation.
		} else {
			// Now we know that all bits should be erased.

			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM0);  // ...and Erase-only mode.
			EECR |= (1<<EEPE);  // Start Erase-only operation.
		}
	} else {
		// Now we know that _no_ bits need to be erased to '1'.
		
		// Check if any bits are changed from '1' in the old value.
		if( diff_mask ) {
			// Now we know that _some_ bits need to the programmed to '0'.
			
			EEDR = new_value;   // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM1);  // ...and Write-only mode.
			EECR |= (1<<EEPE);  // Start Write-only operation.
		}
	}
	
	sei(); // Restore interrupt flag state.
}

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
