/*
  serial.c - Implementation module giao tiếp serial cấp thấp
  
  CHỨC NĂNG:
  - Gửi/nhận byte qua cổng serial với interrupt
  - Quản lý buffer TX và RX (circular buffer)
  - Hỗ trợ XON/XOFF flow control
  - Xử lý real-time commands từ serial stream
  
  Part of Grbl
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
*/

#include "grbl.h"

// Buffer RX (circular buffer)
uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;        // Con trỏ ghi
volatile uint8_t serial_rx_buffer_tail = 0; // Con trỏ đọc (volatile vì thay đổi trong ISR)

// Buffer TX (circular buffer)
uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;        // Con trỏ ghi
volatile uint8_t serial_tx_buffer_tail = 0; // Con trỏ đọc (volatile vì thay đổi trong ISR)

#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Biến trạng thái flow control
#endif

// Trả về số byte đã dùng trong buffer RX
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy để tránh gọi nhiều lần volatile
  if (serial_rx_buffer_head >= rtail) {
    return(serial_rx_buffer_head-rtail);
  }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

// Trả về số byte đã dùng trong buffer TX (chủ yếu dùng để debug)
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy để tránh gọi nhiều lần volatile
  if (serial_tx_buffer_head >= ttail) {
    return(serial_tx_buffer_head-ttail);
  }
  return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}

// Khởi tạo giao tiếp serial
void serial_init()
{
  // Thiết lập tốc độ baud
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit
}


// Ghi 1 byte vào buffer TX (circular buffer)
void serial_write(uint8_t data) {
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; } // Wrap around

  // Chờ đến khi có chỗ trống trong buffer
  while (next_head == serial_tx_buffer_tail) { 
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Kiểm tra reset để tránh vòng lặp vô hạn
  }

  // Lưu dữ liệu và tăng con trỏ head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;
  
  // Bật interrupt Data Register Empty để đảm bảo tx-streaming chạy
  UCSR0B |=  (1 << UDRIE0); 
}

// Interrupt handler khi Data Register Empty (sẵn sàng ghi byte tiếp theo)
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  
  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) { 
      UDR0 = XOFF_CHAR; 
      flow_ctrl = XOFF_SENT; 
    } else if (flow_ctrl == SEND_XON) { 
      UDR0 = XON_CHAR; 
      flow_ctrl = XON_SENT; 
    } else
  #endif
  {
    UDR0 = serial_tx_buffer[tail]; // Gửi byte từ buffer
    
    tail++; // Tăng con trỏ tail
    if (tail == TX_BUFFER_SIZE) { tail = 0; } // Wrap around
  
    serial_tx_buffer_tail = tail;
  }

  // Tắt interrupt nếu buffer rỗng (kết thúc truyền)
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}

// Đọc byte đầu tiên từ buffer RX
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Copy để tối ưu volatile
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA; // Buffer rỗng
  } else {
    uint8_t data = serial_rx_buffer[tail];
    
    tail++; // Tăng con trỏ tail
    if (tail == RX_BUFFER_SIZE) { tail = 0; } // Wrap around
    serial_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      // Gửi XON nếu buffer đã giảm xuống ngưỡng thấp
      if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX để gửi XON
      }
    #endif
    
    return data;
  }
}

// Interrupt handler khi nhận byte từ serial
ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  
  // Xử lý real-time command characters trực tiếp từ serial stream
  // Các ký tự này không vào buffer, mà set flag bits cho real-time execution
  switch (data) {
    case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Báo cáo trạng thái
    case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Bắt đầu chu trình
    case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Tạm dừng feed
    case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Cửa an toàn
    case CMD_RESET:         mc_reset(); break; // Reset motion control
    default: // Ghi ký tự vào buffer
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full.
      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;    
        
        #ifdef ENABLE_XONXOFF
          if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR0B |=  (1 << UDRIE0); // Force TX
          } 
        #endif
        
      }
      //TODO: else alarm on overflow?
  }
}


void serial_reset_read_buffer() 
{
  serial_rx_buffer_tail = serial_rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
