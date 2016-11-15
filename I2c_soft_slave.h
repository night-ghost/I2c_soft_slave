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
#include <stm32f4xx.h>
#include <hal.h>
#include "gpiopins.h"
#include "systick.h"
#include "Scheduler.h"

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

    static void begin(byte address, Handle xmit, Handle recv, Handleb done);
    static void end();

    static uint32_t writeBuffer( uint8_t addr_, uint8_t reg_, uint8_t len_, const uint8_t *data);
    static uint32_t write( uint8_t addr_, uint8_t reg, uint8_t data);
    static uint32_t read( uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
    static uint32_t transfer(uint8_t  addr, uint8_t  send_len, const uint8_t *send, uint8_t len, uint8_t *buf);
    

private:

    static          byte  scl_bit;
    static volatile byte *scl_ddr_port;
    static volatile byte *scl_in_port;
    static          byte  sda_bit;
    static volatile byte *sda_ddr_port;
    static volatile byte *sda_in_port;
    
    static byte scl_front;
    static byte sda_front;

    static byte _address;
    static byte _data;
    static byte state;
    static byte _tmp; // shift register
    
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
};

