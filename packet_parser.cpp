#include "packet_parser.h"


Thread* global_thread_ = NULL;

void dma_complete_signal(MODSERIAL_IRQ_INFO *q) {
  if (global_thread_ != NULL) {
    global_thread_->signal_set(DMA_COMPLETE_FLAG);
  }
}

PacketParser::PacketParser(
  uint32_t baudrate, PinName tx_pin, PinName rx_pin, PinName tx_led, PinName rx_led) :
  pc_(tx_pin, rx_pin), dma_(),
  tx_led_(tx_led), send_thread_(&PacketParser::thread_starter, this),
  rx_led_(rx_led) {
  
  pc_.baud(baudrate);
  pc_.MODDMA(&dma_);
  //pc_.attach_dmaSendComplete(this, &PacketParser::send_complete);
  pc_.attach_dmaSendComplete(&dma_complete_signal);
  global_thread_ = &send_thread_;

  out_pkt_ = NULL;
  tx_sequence_ = 0;

  pc_.attach(this, &PacketParser::receive_callback, MODSERIAL::RxIrq);
  in_pkt_ = (packet_union_t*)in_box_.alloc();
  in_pkt_idx_ = 0;
  in_pkt_len_ = MAX_PACKET_LENGTH;
  in_pkt_crc_ = 0;
  
  send_thread_.signal_set(START_THREAD_FLAG);
}

packet_union_t* PacketParser::get_received_packet(void) {
  osEvent evt = in_box_.get(0);
  if (evt.status == osEventMail) {
    return (packet_union_t*)evt.value.p;
  } else {
    return NULL;
  }
}

void PacketParser::free_received_packet(packet_union_t* packet) {
  in_box_.free(packet);
}

packet_union_t* PacketParser::get_send_packet(void) {
  return (packet_union_t*)out_box_.alloc();
}

void PacketParser::send_packet(packet_union_t* packet) {
  out_box_.put(packet);
}

void PacketParser::thread_starter(void const *p) {
  PacketParser* instance = (PacketParser*)p;
  instance->send_worker();
}

void PacketParser::send_worker(void) {
  send_thread_.signal_wait(START_THREAD_FLAG);
  while(true) {
    osEvent evt = out_box_.get();
    if (evt.status == osEventMail) {
      tx_led_ = 1;
      out_pkt_ = (packet_union_t*)evt.value.p;
      out_pkt_->packet.header.start = 0;
      out_pkt_->packet.header.sequence = tx_sequence_++;
      uint8_t crc_value = calculate_crc8(out_pkt_->raw, out_pkt_->packet.header.length-1);
      out_pkt_->raw[out_pkt_->packet.header.length-1] = crc_value;
      pc_.dmaSend(out_pkt_->raw, out_pkt_->packet.header.length);
      tx_led_ = 0;
      send_thread_.signal_wait(DMA_COMPLETE_FLAG);
      tx_led_ = 1;
      send_thread_.signal_clr(DMA_COMPLETE_FLAG);
      out_box_.free(out_pkt_);
      out_pkt_ = NULL;
      tx_led_ = 0;
    }
    Thread::yield();
  }
}

void PacketParser::send_complete(MODSERIAL_IRQ_INFO *q) {
  tx_led_ = 1;
  if (out_pkt_ != NULL) {
    out_box_.free(out_pkt_);
    out_pkt_ = NULL;
  }
  tx_led_ = 0;
}

void PacketParser::receive_callback(MODSERIAL_IRQ_INFO *q) {
  rx_led_ = 1;
  MODSERIAL* serial = q->serial;

  if (in_pkt_ != NULL) {
    while(serial->readable()) {
      char c = serial->getc();

      // If we just received the second character, set packet length
      if (in_pkt_idx_ == 1) {
        in_pkt_len_ = c;
      }

      // If there has been a parse error, reset packet buffer
      if ((in_pkt_idx_ == 0 && c != 0) || in_pkt_len_ < sizeof(header_t)+1 ) {
        in_pkt_idx_ = 0;
        in_pkt_len_ = MAX_PACKET_LENGTH;
        in_pkt_crc_ = 0;

      // Store byte in packet buffer and update CRC
      } else {
        in_pkt_->raw[in_pkt_idx_++] = c;
        in_pkt_crc_ = update_crc8(in_pkt_crc_, c);
      }

      // If we just received the last character, put valid packets in mailbox
      // and reset packet buffer
      if (in_pkt_idx_ == in_pkt_len_) {
        if (in_pkt_crc_ == 0) {
          in_box_.put(in_pkt_);
          in_pkt_ = (packet_union_t*)in_box_.alloc();
        }
        in_pkt_idx_ = 0;
        in_pkt_len_ = MAX_PACKET_LENGTH;
        in_pkt_crc_ = 0;
      }
    }
  } else {
    in_pkt_ = (packet_union_t*)in_box_.alloc();
  }

  rx_led_ = 0;
}

