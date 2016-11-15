// https://www.google.ru/url?sa=t&rct=j&q=&esrc=s&source=web&cd=61&cad=rja&uact=8&ved=0ahUKEwi5jPDlgqjQAhVBOywKHT7CBgA4PBAWCBowAA&url=https%3A%2F%2Fwww.thinkmind.org%2Fdownload.php%3Farticleid%3Dicons_2011_1_10_20003&usg=AFQjCNGhWB0irTCLzuV0aU8J-nNCk7sppw&sig2=eg0sxI0yh127EH2iIYTtBw

#include "I2c_soft_slave.h"

#define NINTH_BIT 9
#define BYTE_LENGTH 8
#define SEVENTH_BIT 7



#define SCL_H      {*scl_ddr_port &= ~scl_bit; }
#define SCL_L      {*scl_ddr_port |=  scl_bit; }

#define SDA_H      {*sda_ddr_port &= ~sda_bit; }
#define SDA_L      {*sda_ddr_port |=  sda_bit; }

#define SCL_read   ((*scl_in_port & scl_bit)!=0)
#define SDA_read   ((*sda_in_port & sda_bit)!=0)



// we need only PC pins so only pcint8-15 interrupts, PCIE1 flag and PCMSK1 register


         byte  Soft_I2C::scl_bit;
volatile byte *Soft_I2C::scl_ddr_port;
volatile byte *Soft_I2C::scl_in_port;
         byte  Soft_I2C::sda_bit;
volatile byte *Soft_I2C::sda_ddr_port;
volatile byte *Soft_I2C::sda_in_port;

byte Soft_I2C::scl_front;
byte Soft_I2C::sda_front;

byte Soft_I2C::rising_edge_counter=0;
byte Soft_I2C::falling_edge_counter=0;
bool Soft_I2C::restart=false;


byte Soft_I2C::_address;
bool Soft_I2C::_readwrite;
byte Soft_I2C::_data;
byte Soft_I2C::_tmp; // shift register
bool Soft_I2C::_bit;
bool Soft_I2C::repeat_start;


byte Soft_I2C::_register;

// data management
byte *Soft_I2C::_buffer;
byte Soft_I2C::_ptr;
byte Soft_I2C::_size;


Handle Soft_I2C::xmit_cb;
Handle Soft_I2C::recv_cb;
Handleb Soft_I2C::done_cb;
    


enum I2C_STATES  Soft_I2C::state; // states relative to SLAVE so we send in TRANSMIT state


void Soft_I2C::write(byte *addr, byte len){ // just store data pointer and size
    _buffer=addr;
    _ptr=0;
    _size=len;
}

void Soft_I2C::read(byte *addr, byte len){
    _buffer=addr;
    _ptr=0;
    _size=len;
}

// prepare but don't touch pins
Soft_I2C::Soft_I2C(const int scl_pin, const int sda_pin)
{
    uint8_t port;


    port = digitalPinToPort(scl_pin);
    scl_bit  = digitalPinToBitMask(scl_pin);
//    scl_out_port = portOutputRegister(port);
    scl_in_port = portInputRegister(port);
    scl_ddr_port = portModeRegister(port);


    port = digitalPinToPort(sda_pin);
    sda_bit  = digitalPinToBitMask(sda_pin);
//    sda_out_port = portOutputRegister(port);
    sda_in_port = portInputRegister(port);
    sda_ddr_port = portModeRegister(port); // 1 -> output

    SDA_H;
    SCL_H;

    digitalWrite(scl_pin, 0);
    digitalWrite(sda_pin, 0);
}


void Soft_I2C::begin(byte address, Handle recv, Handle xmit, Handleb done){
    SDA_H;
    SCL_H;

    _address=address;
    xmit_cb = xmit;
    recv_cb = recv;
    done_cb = done;
    
    set_scl_rising_intr();
    set_sda_falling_intr();
    clear_scl_intr();
    enable_scl_intr();
    clear_sda_intr();
    enable_sda_intr();
    _data=0;
    _tmp=0;
    state=SLAVE_IDLE;
    
    PCICR |= PCIE1; // enable PinChange interrupt for port C
    
}

void Soft_I2C::end(){
    disable_scl_intr();
    disable_sda_intr();
}

Soft_I2C::~Soft_I2C(){
    end();
}


ISR(PCINT1_vect){
    Soft_I2C::isr();
}

void Soft_I2C::isr(){
    bool sda=SDA_read; // read current values
    bool scl=SDA_read;
    
    if(scl_front) { // we want rising interrupt
        if(scl)  SCL_ISR(); // yes we got!
    } else {        // we need falling edge
        if(!scl) SCL_ISR(); // yes we got!
    }

    if(sda_front) { // we want rising interrupt
        if(sda)  SDA_ISR(); // yes we got!
    } else {        // we need falling edge
        if(!sda) SDA_ISR(); // yes we got!
    }
}



void Soft_I2C::SCL_ISR(void) // SCL
{
    if(SCL_read) {  // Low to High (Rising Edge)    
        rising_edge_counter++;
        switch(state){
        case SLAVE_ADDRESS_RECEIVE:
        case SLAVE_DATA_RECEIVE:
        case SLAVE_DATA_TRANSMIT: // yes we need ACK bit on transmit!
            _bit=SDA_read; // lock data bit on raising edge
            // no break!

        case SLAVE_NOTMY_ADDRESS:
            address_low_high1to9(); // just set interrupt direction - nothing else at high SCL
            break;
        default:
            break;
        }
        if(rising_edge_counter==NINTH_BIT)
            rising_edge_counter=0;

    }else {   // SCL High to Low (Falling Edge)
        falling_edge_counter++;

        switch(state){

        case SLAVE_ADDRESS_RECEIVE:
            if(falling_edge_counter<=SEVENTH_BIT) 
                address_high_low1to7();
            else if(falling_edge_counter==BYTE_LENGTH)
                address_high_low8();    // write / read
            else
                address_high_low9();    // ACK
            break;

        case SLAVE_NOTMY_ADDRESS:
            notmyaddress_high_low1to9(); // just set interrupts
            break;

        case SLAVE_DATA_RECEIVE:
            if(falling_edge_counter<= SEVENTH_BIT)
                receive_high_low1to7();
            else if(falling_edge_counter==BYTE_LENGTH)
                receive_high_low8();
            else
                receive_high_low9();
            break;
        
        case SLAVE_DATA_TRANSMIT:
            if(falling_edge_counter<=SEVENTH_BIT)
                transmit_high_low1to7();
            else if(falling_edge_counter==BYTE_LENGTH)
                transmit_high_low8();
            else
                transmit_high_low9();
        default:
            break;
        }
        if(falling_edge_counter==NINTH_BIT)
            falling_edge_counter=0;
    }
}

void Soft_I2C::SDA_ISR(void) // SDA
{
    if(! SCL_read) {          // SCL Low - glitch
        clear_sda_intr();    // Clear SDA interrupt flag
    } else {
        if(!SDA_read){  // start
            rising_edge_counter=0;
            falling_edge_counter=0; /*Initialize all counters*/
            
            /*Reload timers*/
            disable_sda_intr();   // Disable SDA interrupt
            if(state==SLAVE_DATA_RECEIVE){
                restart=true;                 /* Set repeated start condition */
                _register=_buffer[0]; // received byte is register number
            }
            state=SLAVE_ADDRESS_RECEIVE;
            _tmp=0;
            _data=0;

        } else {  // Stop

            SCL_H;     // (set SCL as input)
            SDA_H;     // (set SDA as input)
            set_sda_falling_intr(); 
            clear_sda_intr();    
            set_scl_rising_intr(); 
            clear_scl_intr();    
        /*Clear timer*/
            restart=0;
            state=SLAVE_IDLE;
            
            _buffer=0; // no more
            
            if(done_cb) done_cb(_ptr-1); // send data size to finish proc
        }
    }
}


// just adjust interrupts - we shouldn't do anything at high SCL
void Soft_I2C::address_low_high1to9(){ // SCL // Low to High (Rising Edge) all modes
    if( SDA_read )          // Set SDA interrupt edge
        set_sda_falling_intr(); // SCL for 1->0 interrupt edge
    else
        set_sda_rising_intr(); // SCL for 1->0 interrupt edge
    
    set_scl_falling_intr(); // Set SCL for 1->0 interrupt edge
    clear_scl_intr();    // Clear SCL interrupt flag
    clear_sda_intr();    // Clear SDA interrupt flag
    enable_sda_intr();   // Enable SDA interrupt
}



void Soft_I2C::address_high_low1to7(){ // // SCL High to Low (Falling Edge) on address receive, data bits
    SCL_L;      // extend clock
    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();

    SCL_H; // release SCL

    _tmp <<= 1;
    if(_bit) _tmp |= 1; // data bits to shift register
}

void Soft_I2C::address_high_low8(){ // got read/write bit
    SCL_L; // extend clock

    if(_address == _tmp) {  // we got correct address
        _readwrite = (_bit != 0);
        SDA_L; // confirm ACK
    }else{
        state=SLAVE_NOTMY_ADDRESS;;
        SDA_H; // NACK
    }

    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();
    SCL_H; // release SCL
}


void Soft_I2C::send_bit(){
    if(_data & 0x80){    SDA_H; }
    else            {    SDA_L; }
    
    _data <<= 1; // Left shift transmit data
}


void Soft_I2C::send_byte(){
    if(!_buffer) return;
    
    /*store the transmit data bitwise to SDA line*/    
    _data=_buffer[_ptr] ; // get byte of data to send
    
    if(_ptr<_size)
        _ptr++; 
        
    send_bit();
}

void Soft_I2C::address_high_low9(){
    SCL_L; // extend clock

// if we come here then address is correct - else state changes to SLAVE_NOTMY_ADDRESS
    if(_readwrite) {
        state=SLAVE_DATA_TRANSMIT;

        if(xmit_cb) xmit_cb(); // callback for transmitting data
        
        send_byte();

    }else{
        SDA_H;  // release bus
        state= SLAVE_DATA_RECEIVE;
    }

    if(repeat_start==1){
        rising_edge_counter=0; /*all counters = 0, */
        falling_edge_counter=0; 
        /*  repeat_start =0  at stop */
    }

    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();
    SCL_H; // release SCL
}



void Soft_I2C::notmyaddress_high_low1to9(){
    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();
}



void Soft_I2C::receive_high_low1to7(){ // receiving byte
    SCL_L; // extend clock
    set_scl_rising_intr();
    clear_scl_intr();

    disable_sda_intr();
    _tmp <<= 1;      /*Left shift carry bit into I2C val*/
    if(_bit) _tmp |= 1; // data bits to shift register
    SCL_H;  // release SCL
}


void Soft_I2C::receive_high_low8(){ // last bit of byte is come - we should confirm it
    SCL_L;    // extend clock
    SDA_L;      // ACK
    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();

    _tmp <<= 1;
    if(_bit) _tmp |= 1; // data bits to shift register
    _data = _tmp;    // store the byte value

    SCL_H;  // release SCL

}

void Soft_I2C::receive_high_low9(){  // ACK done, release bus
    SCL_L; // extend clock
    SDA_H;      // end of ACK
    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();

//    state=SLAVE_DATA_RECEIVE;
    /*Stop Timer*/
    
    if(recv_cb) recv_cb(); // callback for receiving data

    if(_buffer) {
        _buffer[_ptr] = _data; // store received data
        if(_ptr<_size)
            _ptr++; 
    }
    _tmp=0; /*reset i2c data, and reload timer*/
    
    SCL_H;  // release SCL
}


void Soft_I2C::transmit_high_low1to7(){  // transmit byte
    SCL_L; // extend clock
    set_scl_rising_intr();
    clear_scl_intr();
    send_bit();
    
    disable_sda_intr();
    SCL_H;  // release SCL
}


void Soft_I2C::transmit_high_low8(){ // last bit of byte is transmitted - read ACK
    SCL_L; // extend clock
    SDA_H;
    set_scl_rising_intr();
    clear_scl_intr();
    disable_sda_intr();
    SCL_H;  // release SCL
}


void Soft_I2C::transmit_high_low9(){
    SCL_L;

    if(_bit) { // we got NACK and should stop transmitting
        state=SLAVE_NOTMY_ADDRESS; // just skip the rest

    }else { // we got ACK and should transmit the next byte
//        state = SLAVE_DATA_TRANSMIT;
        send_byte(); // next byte
        
        set_scl_rising_intr();
        clear_scl_intr();
        /*Reload timer*/
        disable_sda_intr();
    }
    SCL_H;
}


