/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <inttypes.h>

#include "Arduino.h"

typedef uint8_t byte;

enum I2C_STATES {
    SLAVE_IDLE =0,
    SLAVE_ADDRESS_RECEIVE,
    SLAVE_NOTMY_ADDRESS,
    SLAVE_DATA_RECEIVE,
    SLAVE_DATA_TRANSMIT,
};

typedef void (*Handle)();
typedef void (*Handleb)(byte);

class Soft_I2C {
public:
    Soft_I2C( const int scl_pin, const int sda_pin);
    ~Soft_I2C();

    static void begin(byte address, Handle xmit, Handle recv, Handleb done=NULL);
    static void end();

    static void write(byte *addr, byte len);
    static void read(byte *addr, byte len);

    static void isr();

private:

    static          byte  scl_bit;
    static volatile byte *scl_ddr_port;
    static volatile byte *scl_in_port;
    static          byte  sda_bit;
    static volatile byte *sda_ddr_port;
    static volatile byte *sda_in_port;
    
    static byte scl_front;
    static byte sda_front;

    static byte rising_edge_counter;
    static byte falling_edge_counter;
    static bool restart;

    static byte _address;
    static bool _readwrite;
    static byte _data;
    static enum I2C_STATES state;
    static byte _tmp; // shift register
    static bool _bit;
    static bool repeat_start;
    
    static byte _register;

// data management
    static byte *_buffer;
    static byte _ptr;
    static byte _size;


    static Handle xmit_cb;
    static Handle recv_cb;
    static Handleb done_cb;

// pin change causes interrupt on both edges so we need to manually filter out wrong edges
    static inline void set_scl_falling_intr(){
        scl_front=false;
    }
    static inline void set_scl_rising_intr(){
        scl_front=true;
    }
    
    static inline void clear_scl_intr(){}
    static inline void disable_scl_intr(){
        PCMSK1 &= ~scl_bit;
    }
    static inline void enable_scl_intr(){
        PCMSK1 |= scl_bit;
    }

    static inline void set_sda_falling_intr(){
        sda_front=false;
    }
    static inline void set_sda_rising_intr(){
        sda_front=true;
    }
    static inline void clear_sda_intr(){}
        
    static inline void disable_sda_intr(){
        PCMSK1 &= ~sda_bit;
    }
    static inline void enable_sda_intr(){
        PCMSK1 |= sda_bit;
    }


    static void send_byte();
    static void send_bit();

    static void address_low_high1to9(); // just set interrupt direction - nothing else at high SCL
    static void address_high_low1to7();
    static void address_high_low8();    // write / read
    static void address_high_low9();    // ACK
    static void notmyaddress_high_low1to9(); // just set interrupts
    static void receive_high_low1to7();
    static void receive_high_low8();
    static void receive_high_low9();
    static void transmit_high_low1to7();
    static void transmit_high_low8();
    static void transmit_high_low9();


    static void SCL_ISR();
    static void SDA_ISR();
};

