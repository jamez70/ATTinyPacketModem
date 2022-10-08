


// ==== defs
#define WELCOME_MSG      "\r\nTNCTiny v0.01\r\n\r\n"
#define MIN_PACKET_LEN   10
#define PACKET_SIZE      200
#define INBUF_SIZE (200) // Educated guess for a good buffer size
#define AX25_MARK        0
#define AX25_SPACE       1
#define MAX_SYNC_ERRS    5
// T3TOP was 1212, but we are at 10mhz so came up with 758 <- thid
#define T3TOP 758
// For a better sine wave, we can load the timer differently - 19200 is
#define TRANSMIT_TOP 521
#define READY_TO_SEND (USART0.STATUS & (0x20))
#define DITDLY 50

// Defines

#define BIT_DELAY 8333 // 189 Delay for 0.833 ms (189 for 14.7456 MHz) 205 for 16mhz
#define BIT_MS_DELAY 10000
#define SPACE (283)    // gives us 2228hz
#define MARK (520)     // gives us 1200.98hz close enough with 16mhz clock
#define SPACE_PWM (2272)
#define MARK_PWM (4166)
#define	TRUE 	(1)
#define	FALSE 	(0)

//#define OPERMODE_MX614
#define OPERMODE_ADC

/* Port setup
PA0 - UPDI
PA1 - TXD / MOSI
PA2 - RXD / MISO
PA3 - B0  / SCK
PA4 - DET / CS
PA5 - PTT/M0
PA6 - M1 / LED2
PA7 - Stat LED
PB0 - TX PWM Out (WO0)
PB1 - RX Analog (AN10)
PB2 - TXD Serial
PB3 - RXD Serial

*/
#define SET_DDRA    (PORTA.DIR   = 0xFA)
#define SET_DDRB    (PORTB.DIR   = 0x05)
#define HB_ON       (PORTA.OUTSET = 0x40)
#define HB_OFF      (PORTA.OUTCLR = 0x40)
#define DCD_ON      (PORTA.OUTSET = 0x80)
#define DCD_OFF     (PORTA.OUTCLR = 0x80)
#ifdef OPERMODE_MX614
#define M0_ON       (PORTA.OUTSET = 0x20)
#define M0_OFF      (PORTA.OUTCLR = 0x20)
#define M1_ON       (PORTA.OUTSET = 0x40)
#define M1_OFF      (PORTA.OUTCLR = 0x40)
#else
#define M0_ON       ;
#define M0_OFF      ;
#define M1_ON       ;
#define M1_OFF      ;
#endif
#define SS_LOW      (PORTA.OUTCLR = 0x10)
#define SS_HIGH     (PORTA.OUTSET = 0x10)
#define TX_MARK     (PORTA.OUTSET = 0x02)
#define TX_SPACE    (PORTA.OUTCLR = 0x02)
#define TX_INVERT   (PORTA.OUTTGL = 0x02)
#define RX_BIT      ((PORTA.IN & 0x04) >> 2)
#define HB_TOGGLE   (PORTA.OUTTGL = 0x40)
#define LED_TOGGLE  (PORTA.OUTTGL = 0x40)
#define TX_ON       (PORTA.OUTSET = 0x20)
#define TX_OFF      (PORTA.OUTCLR = 0x20)


// ==== includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/eeprom.h>

#include "params.h"

// ==== protos
ISR(TCA0_OVF_vect);
void SendRawByte(uint8_t val);
void decode_ax25(void) ;
void send_serial_str( const char * inputstr ) ;
void serprintf(const char *format, ...);
void serprintln(const char *format, ...);
const char *GetOnOff(uint8_t val);
void mainDelay(void);
void ax25crcBit(int lsb_int);
void ax25sendHeader(void);
void ax25sendFooter(void);
void local_delay(uint16_t delay_ticks);
void ax25sendByte(uint8_t txbyte);
void mainTransmit(void);
void mainReceive(void);

typedef void(*callback_t)(char*);

typedef struct
{
    callback_t  cb;
    char *Cmd;
    char *Usage;
} cmdList_t;


// ==== vars
volatile int8_t adcval; // zero-biased ADC input
volatile int16_t mult_sum;   // sum of mult_cb values

volatile uint8_t since_last_chg;       // samples since the last MARK/SPACE symbol change
volatile uint8_t phase_err;            // symbol transition timing error; in samples
volatile uint8_t current_symbol;       // MARK or SPACE
volatile uint8_t last_symbol;          // MARK or SPACE from previous reading
volatile uint8_t last_symbol_inaframe; // MARK or SPACE from one bit-duration ago
volatile uint8_t inaframe;             // rec'd start-of-frame marker
volatile uint8_t bittimes;             // bit durations that have elapsed
volatile uint8_t bitqlen;              // number of rec'd bits in bitq
volatile uint8_t popbits;              // number of bits to pop (drop) off the end
volatile uint8_t byteval;              // rec'd byte; read from bitq
uint8_t msg[PACKET_SIZE + 1]; // rec'd data
uint8_t msgcopy[PACKET_SIZE + 1]; // rec'd data
uint8_t TxFrame[PACKET_SIZE + 1]; // transmit frame
uint8_t TxSize;
//uint8_t txframe_pos;              // frame position
//volatile uint8_t TxReady;     // Flag showing transmission is ready
uint8_t msg_pos;              // bytes rec'd; next array element to fill
volatile uint8_t OutputPacketSize;// bytes rec'd; next array element to fill
uint8_t x;                    // misc counter
uint8_t test;                 // temp variable
uint8_t decode_state;         // section of rec'd data being parsed

uint8_t sync_err_cnt; // number of sync errors in this frame (so far)
uint8_t bit_timer;    // countdown one bit duration
uint8_t thesebits;    // number of elapsed bit durations
volatile uint8_t prevdata;     // Previous parsed input data
uint8_t flagprev;     // flag if not type 0x00
uint8_t hexwidth;     // width of output
uint8_t MonitorFlag;  // Monitoring Enabled flag
#ifndef OPERMODE_MX614
volatile uint8_t currentTone;
int16_t mult_cb[7]; // circular buffer for adc*delay values
int16_t bias_sum;   // sum of last 128 ADC readings
uint8_t rawadc;               // value read directly from ADCH register
uint8_t bias_cnt;             // number of ADC samples collected (toward goal of 128)
uint8_t adc_bias;             // center-value for ADC, self-adjusting
uint8_t cb_pos;               // position within the circular buffer
uint32_t accumulator;
#endif
int8_t     adc_delay[6] ;             // delay line for adc readings

uint32_t        bitq ;                     // rec'd bits awaiting grouping into bytes
volatile uint8_t kissMode = 0;
volatile uint8_t ctrlcCount = 0;
uint32_t BeaconTimer = 0;

#ifndef OPERMODE_MX614
#define ACCUMULATOR_BITS 24 // This is 2^10 bits used from accum
const uint16_t sinetable[256] = {
    379,388,397,406,416,425,434,443,452,461,470,479,488,497,506,515,
    523,532,540,549,557,565,573,581,589,596,604,611,619,626,633,639,
    646,653,659,665,671,677,683,688,693,698,703,708,712,717,721,725,
    728,732,735,738,741,743,746,748,750,751,753,754,755,756,757,757,
    757,757,757,756,755,754,753,751,750,748,746,743,741,738,735,732,
    728,725,721,717,712,708,703,698,693,688,683,677,671,665,659,653,
    646,639,633,626,619,611,604,596,589,581,573,565,557,549,540,532,
    523,515,506,497,488,479,470,461,452,443,434,425,416,406,397,388,
    379,369,360,351,341,332,323,314,305,296,287,278,269,260,251,242, // 128-143
    234,225,217,208,200,192,184,176,168,161,153,146,138,131,124,118, // 144-159
    111,104,98,92,86,80,74,69,64,59,54,49,45,40,34,32, // 160-175
    29,25,22,19,16,14,11,9,7,6,4,3,2,1,0,0, // 176-189
    0,0,0,1,2,3,4,6,7,9,11,14,16,19,22,25, // 190 - 207
    29,32,36,40,45,49,54,59,64,69,74,80,86,92,98,104, // 208-223
    111,118,124,131,138,146,153,161,168,176,184,192,200,208,217,225, // 224-239
    234,242,251,260,269,278,287,296,305,314,323,332,341,351,360,369 // 240-255
};


#define REFCLK 13200
static const unsigned long toneStep[2] = {
    715827882,
    390451572
    //268435456,
    //492131669
};

//  pow(2,32)*2200.0/REFCLK,
//  pow(2,32)*1200.0/REFCLK

static uint8_t sine_index;  // Index for the D-to-A sequence

#endif
static uint8_t IsTransmitting;    // Keeps track of TX/RX state

volatile uint16_t txtone;   // Used in main.c SIGNAL(SIG_OVERFLOW2)
volatile uint8_t maindelay; // State of mainDelay function

static uint8_t InputBuffer[INBUF_SIZE + 1]; // USART input buffer array
volatile uint8_t InputBeg;               // USART input buffer head pointer
volatile uint8_t InputEnd;               // USART input buffer tail pointer
static unsigned short crc;      // CRC value
volatile uint32_t m_w = 1;        // Random Seed 1
volatile uint32_t m_z = 2;        // Random Seed 2
volatile uint16_t RTC_Overflow;   // RTC Overflow Counter
Packet_Params_t params;
volatile uint8_t CmdModeState;    // command mode state machine index
const char *Prompt = "cmd:";
const char *ON =     "ON";
const char *OFF =    "OFF";
const char *hexdata = "0123456789abcdef";
char strBuffer[128];
char frameBuffer[128];

enum {
    CMD_MODE_KISS,
    CMD_MODE_SHOW_PROMPT,
    CMD_MODE_WAIT_INPUT,
    CMD_MODE_HANDLE_COMMAND,
    CMD_MODE_CLEAR_COMMAND
};

void handleUsage(char *str);
void handleMyCall(char *str);
void handleUnproto(char *str);
void handleBroadcast(char *str);
void handleDigi1Call(char *str);
void handleDigi2Call(char *str);
void handleMon(char *str);
void handleCW(char *str);
void handleTxDelay(char *str);
void SendPacket(uint8_t *buf, uint16_t len);
void DecodeMorse(char *str);
void handleKiss(char *str);
void handleStartMode(char *str);
void handleMHeard(char *str);
void handleBeaconText(char *str);
void handleBeaconInt(char *str);
uint32_t getRTCCount(void);

const cmdList_t cmdList[] = {
    {handleUsage,     "help",     "help"},
    {handleMyCall,    "mycall",   "Set your callsign"},
    {handleUnproto,   "unproto",  "Set Destination call"},
    {handleDigi1Call, "digi1",    "Set Digipeater 1"},
    {handleDigi2Call, "digi2",    "Set Digipeater 2"},
    {handleBroadcast, "bcast",    "Broadcast a Message"},
    {handleMon,       "monitor",  "Monitor incoming packets (ON/OFF)"},
    {handleCW,        "cw",       "Send morse code"},
    {handleTxDelay,   "txdelay",  "Set/Get TX delay"},
    {handleKiss,      "kiss",     "Enter KISS mode"},
    {handleStartMode, "startmode","Startup mode (cmd/kiss)"},
    {handleMHeard,    "mheard",   "List heard callsigns"},
    {handleBeaconText,"btext",   "ID text"},
    {handleBeaconInt, "beacon",   "Beacon Interval"},
    {NULL, NULL, NULL}

};


const uint16_t CWTable[] = {
    'A' | (2 << 7) | ((0b010000) << 10),
    'B' | (4 << 7) | ((0b100000) << 10),
    'C' | (4 << 7) | ((0b101000) << 10),
    'D' | (3 << 7) | ((0b100000) << 10),
    'E' | (1 << 7) | ((0b000000) << 10),
    'F' | (4 << 7) | ((0b001000) << 10),
    'G' | (3 << 7) | ((0b110000) << 10),
    'H' | (4 << 7) | ((0b000000) << 10),
    'I' | (2 << 7) | ((0b000000) << 10),
    'J' | (4 << 7) | ((0b011100) << 10),
    'K' | (3 << 7) | ((0b101000) << 10),
    'L' | (4 << 7) | ((0b010000) << 10),
    'M' | (2 << 7) | ((0b110000) << 10),
    'N' | (2 << 7) | ((0b100000) << 10),
    'O' | (3 << 7) | ((0b111000) << 10),
    'P' | (3 << 7) | ((0b110000) << 10),
    'Q' | (4 << 7) | ((0b110100) << 10),
    'R' | (3 << 7) | ((0b010000) << 10),
    'S' | (3 << 7) | ((0b000000) << 10),
    'T' | (1 << 7) | ((0b100000) << 10),
    'U' | (3 << 7) | ((0b001000) << 10),
    'V' | (4 << 7) | ((0b000100) << 10),
    'W' | (3 << 7) | ((0b011000) << 10),
    'X' | (4 << 7) | ((0b100100) << 10),
    'Y' | (4 << 7) | ((0b101100) << 10),
    'Z' | (4 << 7) | ((0b110000) << 10),
    '1' | (5 << 7) | ((0b011110) << 10),
    '2' | (5 << 7) | ((0b001110) << 10),
    '3' | (5 << 7) | ((0b000110) << 10),
    '4' | (5 << 7) | ((0b000010) << 10),
    '5' | (5 << 7) | ((0b000000) << 10),
    '6' | (5 << 7) | ((0b100000) << 10),
    '7' | (5 << 7) | ((0b110000) << 10),
    '8' | (5 << 7) | ((0b111000) << 10),
    '9' | (5 << 7) | ((0b111100) << 10),
    '0' | (5 << 7) | ((0b111110) << 10),
    '.' | (6 << 7) | ((0b010101) << 10),
    '?' | (6 << 7) | ((0b001100) << 10),
    ',' | (6 << 7) | ((0b110011) << 10),
    '!' | (6 << 7) | ((0b010100) << 10),
    0
};



void toneset(uint8_t val)
{
    M1_OFF;
    if (val)
    {
        local_delay(DITDLY*3);
    }
    else
    {
        local_delay(DITDLY);
    }
    M1_ON;
    local_delay(DITDLY);
}

void handleKiss(char *str)
{
    // Enter kiss mode
    serprintln("%s","Entering KISS mode");
    kissMode = 1;
    
}

void UpdateBeaconTimer(void)
{
    BeaconTimer = params.IdInterval;
    if (BeaconTimer != 0)
    {
        BeaconTimer *= 60;
        BeaconTimer <<= 10;
        BeaconTimer += getRTCCount();
    }
}

void handleStartMode(char *str)
{
    // Handle startup mode
    if ((str != NULL) && (strlen(str) > 0))
    {
        if (atoi(str) != 0)
        {
            params.StartMode = 1;
        }
        else
        {
            params.StartMode = 0;
        }
    }
    serprintln("Startmode is %d", params.StartMode);
    SaveEepromContents(&params);
}

void handleMHeard(char *str)
{

}

void DecodeMorse(char *str)
{
    char ch;
    uint16_t i,j;
    uint16_t tval;
    char wrd[8];
    uint8_t mlen;

    local_delay(50);
    while (*str != 0)
    {
        ch = *str++;
        i=0;
        mlen = 0;
        while (CWTable[i] != 0)
        {
            if ((CWTable[i] & 0x7f) == toupper(ch))
            {
                tval = CWTable[i];
                mlen = (tval >> 7) & 0x07;
                for (j=0; j<mlen; j++)
                {
                    if (tval & 0x8000)
                    {
                        wrd[j] = '-';
                        toneset(1);
                    }
                    else
                    {
                        wrd[j] = '.';
                        toneset(0);
                    }
                    tval <<= 1;
                }
                wrd[mlen] = 0;
                serprintln("%c - %-8s",ch,wrd);
                break;
            }
            i++;
        }
        if (CWTable[i] == 0)
        {
            switch(ch)
            {
                case 32:
                local_delay(DITDLY*3);
                break;
                default:
                serprintln("%c - UNK",ch);
            }
        }
        local_delay(DITDLY*3);
    }
}

void serprintf(const char *format,...)
{
    va_list ap;
    int size;

    size=0;
    va_start(ap, format);
    size=vsnprintf(strBuffer, sizeof(strBuffer), format, ap);
    if (size > 0)
    {
        send_serial_str(strBuffer);
    }
}

void serprintln(const char *format,...)
{
    va_list ap;
    int size;

    size=0;
    va_start(ap, format);
    size=vsnprintf(strBuffer, sizeof(strBuffer), format, ap);
    if (size > 0)
    {
        send_serial_str(strBuffer);
        send_serial_str("\r\n");
    }
}

// An example of a simple pseudo-random number generator is the
// Multiply-with-carry method invented by George Marsaglia.
// two initializers (not null)
unsigned long getRandom()
{
    m_z = 36969L * (m_z & 65535L) + (m_z >> 16);
    m_w = 18000L * (m_w & 65535L) + (m_w >> 16);
    return (m_z << 16) + m_w;  /* 32-bit result */
}


void SetupRTC(void)
{
    RTC_Overflow=0;
    RTC.CTRLA = RTC_PRESCALER_DIV32_gc  // 32
    | 1 << RTC_RTCEN_bp     // Enable: enabled
    | 1 << RTC_RUNSTDBY_bp; // Run In Standby: enabled
    RTC.DBGCTRL = 1 << RTC_DBGRUN_bp; // Run in debug: enabled
    RTC.INTCTRL = 0 << RTC_CMP_bp    // Compare Match Interrupt enable: disabled
    | 1 << RTC_OVF_bp; // Overflow Interrupt enable: enabled
    //  HB_TOGGLE;

}

uint32_t getRTCCount(void)
{
    uint32_t cnt;
    cnt = (uint32_t)RTC_Overflow << 16;
    cnt |= RTC.CNT;
    return cnt;
}

void local_delay(uint16_t delay_ticks)
{
    uint32_t dlyamt;

    dlyamt = getRTCCount();
    dlyamt += delay_ticks;
    while (getRTCCount() < dlyamt)
    ;

}

ISR(RTC_CNT_vect)
{
    //  HB_TOGGLE;

    ++RTC_Overflow;
    
    /* Overflow interrupt flag has to be cleared manually */
    RTC.INTFLAGS = RTC_OVF_bm;
}


ISR(TCA0_OVF_vect)
{
    TCA0.SINGLE.INTFLAGS = 0x01;
    HB_TOGGLE;
    if (IsTransmitting) // if transmitting output our sine tone
    {
        #ifdef OPERMODE_MX614
        TCA0.SINGLE.CMP0 = txtone;     // Preload counter based on freq.
        #else
        accumulator += toneStep[currentTone];
        uint8_t phAng = (accumulator >> ACCUMULATOR_BITS);
        TCA0.SINGLE.CMP0 = sinetable[phAng];
        #endif
    }
    else
    {
        // calculate ADC bias (average of last 128 ADC samples)
        // this input is decoulped from the receiver with a capacitor,
        // and is re-biased to half of the Arduino regulated +3.3V bus
        // with a voltage divider. therefore the net bias should equal
        // (more or less) the settle point of the voltage divider.
        // doing this in software also means that the calculated bias
        // will re-center automatically if the resistors are not
        // perfectly matched, etc.
        
        
        //=========================================================
        // Seguine math
        //    for details, see http://www.cypress.com/?docID=2328
        //=========================================================
        #ifdef OPERMODE_MX614
        adcval = RX_BIT ;
        // rotate the delay
        for ( x=4 ; x>=1 ; x-- )
        {
            adc_delay[x] = adc_delay[x-1] ;
        }
        adc_delay[0] = adcval ;
        mult_sum = 0;
        for (x=0; x<5; x++)
        {
            mult_sum += adc_delay[x];
        }
        if (adcval)
        {
            current_symbol = AX25_MARK ;
        }
        else
        {
            current_symbol = AX25_SPACE ;
        }
        
        #else
        rawadc = ADC0.SAMPLE;
        bias_sum += rawadc;
        if (++bias_cnt == 128)
        {
            adc_bias = bias_sum >> 7;
            bias_cnt = 0;
            bias_sum = 0;
        }

        //=========================================================
        // Seguine math
        //    for details, see http://www.cypress.com/?docID=2328
        //=========================================================

        adcval = rawadc - adc_bias;

        // circle buffer is just 7 elements (about half of a bit duration)
        if (++cb_pos == 7)
        {
            cb_pos = 0;
        }

        // back out old value from sum
        mult_sum -= mult_cb[cb_pos];

        // multiply the latest ADC value by the value ~1/2 lo-freq duration ago. if the
        // incoming audio is the 1200 Hz MARK tone, the two samples will (usually) have
        // opposite phases/signs, so multiplying the two values will give a negative result.
        // If the incoming audio is 2200 Hz, the two samples usu. have the same phase/sign,
        // so multiplying them will give a positve result.
        // subscript 5 = six samples ago-------v
        mult_cb[cb_pos] = adcval * adc_delay[5];

        // add this result to get the sum of the last 7 multipliers (1st LPF)
        mult_sum += mult_cb[cb_pos];

        // force a definitive answer with a hysteresis zone around zero (2nd LPF)
        if (mult_sum >= 100)
        {
            current_symbol = AX25_SPACE;
        }
        else if (mult_sum <= -100)
        {
            current_symbol = AX25_MARK;
        }
        else
        {
            ;
        } // inconclusive - dont change
        #endif
        


        // increment # of samples since last symbol change, enforce a max
        if (++since_last_chg > 200)
        {
            since_last_chg = 200;
        }
        thesebits = 0 ;
        
        #ifndef OPERMODE_MX614
        // rotate the delay
        for (x = 5; x >= 1; x--)
        {
            adc_delay[x] = adc_delay[x - 1];
        }
        adc_delay[0] = adcval;
        #endif

        //=============================
        //   clock and bit recovery
        //=============================
        
        
        // the in-a-frame and seeking-frame-start modes require different strategies
        // let's split them up here
        
        if ( inaframe )
        {
            //================================
            // clock recovery within a frame
            //================================
            
            // check symbol state only once per bit time (3rd LPF)
            bit_timer-- ;
            
            if ( current_symbol != last_symbol )
            {

                
                // save the new symbol
                last_symbol = current_symbol ;
                // reset counter
                since_last_chg = 0 ;
                
                // Ideally, frequency transitions will occur on a since-last-change
                // count that is an exact bit duration at the 1200 Hz signaling
                // rate - that is, an exact multiple of 11 samples (11 samples = 1 bit,
                // 22 = 2 bits, 33 = 3, etc). To give some extra settle time, we
                // don't attempt to read the symbol value at the exact moment of
                // transition; instead, we give it 4 extra beats. Thus as bit_timer
                // is counting down to the next symbol check, its value should ideally
                // be 4 when the symbol change actually takes place. If the symbol
                // change is a little early or late, we can tweak the bit_timer to
                // tolerate some drift and still keep sync.
                // By those rules, an SLC of 4 is perfect, 2 through 6 are fine and
                // need no adjustment, 7,8 and 0,1 need adjustment, 9,10 are timing
                // errors - we can accept a certain number of those before aborting.
                
                if ( ( bit_timer == 7 ) || ( bit_timer == 8 ) )
                {
                    // other station is slow or we're fast. nudge timer back.
                    bit_timer -= 1 ;
                }
                else if ( ( bit_timer == 0 ) || ( bit_timer == 1 ) )
                {
                    // they're fast or we're slow - nudge timer forward
                    bit_timer += 1 ;
                }
                else if ( ( bit_timer == 9 ) || ( bit_timer == 10 ) )
                {
                    // too much error
                    if ( ++sync_err_cnt > MAX_SYNC_ERRS )
                    {
                        sync_err_cnt = 0 ;
                        msg_pos = 0 ;
                        inaframe = 0 ;
                        bitq = 0 ;
                        bitqlen = 0 ;
                        bit_timer = 0 ;
                        bittimes = 0 ;
                        // turn off DCD light
                        DCD_OFF ;
                        return ;
                    }
                    else
                    {
                    }
                }
            } // end bit_timer cases

            
            //=============================
            // bit recovery within a frame
            //=============================
            
            if ( bit_timer == 0 )
            {
                // time to check for new bits
                
                // reset timer for the next bit
                bit_timer = 11 ;
                // another bit time has elapsed
                bittimes++ ;
                
                // wait for a symbol change decide what bits to clock in,
                // and how many
                if ( current_symbol != last_symbol_inaframe )
                {
                    // add one as ready-to-decode flag
                    thesebits = bittimes + 1 ;
                    bittimes = 0 ;
                    last_symbol_inaframe = current_symbol ;
                }
                
            } // end if bit_timer==0
            
        } // end if inaframe
        
        else
        {

            //=================
            // not in a frame
            //=================
            
            // housekeeping
            // phase_err = since_last_change MOD 11, except that the "%" operator is =very slow=
            phase_err = since_last_chg ;
            while (phase_err >= 11)
            {
                phase_err -= 11;
            }
            
            // elapsed bittimes = round (since_last_chg / 11)
            bittimes = 0 ;
            test = since_last_chg + 5 ;
            while (test > 11)
            {
                test -= 11;
                bittimes++;
            }
            thesebits = 0 ;
            
            //====================================
            // clock recovery NOT within a frame
            //====================================
            
            // our bit clock is not synced yet, so we will need a symbol transition
            // to either establish sync or to clock in bits (no transition? exit ISR)
            if ( current_symbol == last_symbol )
            
            // symbol change
            
            {

                return;
            }
            
            // save the new symbol, reset counter
            last_symbol = current_symbol ;
            since_last_chg = 0 ;
            
            // check bit sync
            if ( ( phase_err >= 4 ) && ( phase_err <= 7 ) )
            {
                // too much error
                bitq = 0 ;
                bitqlen = 0 ;
                // turn off the DCD light
                DCD_OFF ;
            }
            
            // save these bits
            thesebits = bittimes + 1 ;
            
        } // end else ( = not inaframe)
        
        
        //========================================
        //   bit recovery, in or out of a frame
        //========================================
        
        
        // if no bit times have elapsed, nothing to do
        if (thesebits == 0)
        {
            return;
        }
        else
        {
            thesebits--;
        } // remove the "ready" flag
        
        // determine incoming bit values based on how many bit times have elapsed.
        // whatever the count was, the last bit involved a symbol change, so must be zero.
        // all other elapsed bits must be ones. AX.25 is transmitted LSB first, so in
        // adding bits to the incoming bit queue, we add them right-to-left (ie, new bits
        // go on the left). this lets us ready incoming bytes directly from the lowest
        // eight bits of the bit queue (once we have that many bits).
        
        // the act of adding bits to the queue is in two parts - (a) OR in any one bits,
        // shifting them to the left as required prior to the OR operation, and (b) update
        // the number of bits stored in the queue. with zero bits, there's nothing to
        // OR into place, so they are taken care of when we update the queue length, and
        // when we shift the queue to the right as bytes are popped off the end later on.
        
        switch ( thesebits )
        {
            case 1:
            break; // no ones to add ------> binary       "0"
            
            case 2:
            bitq |= (0x01 << bitqlen); // binary      "01"
            break ;
            
            case 3:
            bitq |= (0x03 << bitqlen); // binary     "011"
            break ;
            
            case 4:
            bitq |= (0x07 << bitqlen); // binary    "0111"
            break ;
            
            case 5:
            bitq |= (0x0F << bitqlen); // binary   "01111"
            break ;
            
            // "six" is a special case ("bitstuff"): drop the zero, and only add the 5 one bits
            case 6:
            bitq |= (0x1F << bitqlen); // binary   "11111"
            thesebits = 5 ;
            break ;
            
            // "seven" is another special case - only legal for an HDLC byte
            case 7:                                   // binary "0111111"
            if ( ( bitqlen == 1 ) && ( bitq == 0 ) )
            {
                // only one bit pending, and it's a zero
                // this is the ideal situation to receive a "seven".
                // complete the HDLC byte
                bitq = 0x7E ;
                bitqlen = 8 ;
            }
            else if ( bitqlen < 4 )
            {
                // 0-3 bits still pending, but not the ideal single-zero.
                // this is kinda ugly. let's dump whatever is pending,
                // and close the frame with a tidy right-justified HDLC.
                bitq = 0x7E ;
                bitqlen = 8 ;
            }
            else if ( bitqlen >= 4 )
            {
                // also kinda ugly - half or more of an unfinished byte.
                // lets hang onto the pending bits by fill out this
                // unfinished byte with zeros, and append the full HDLC
                // char, so that we can close the frame neatly.
                bitq = ( bitq & 0xFF ) | 0x7E00 ;
                bitqlen = 16 ;
            }
            else
            {
                // huh?!? ok, well, let's clean up
                bitq = 0x7E ;
                bitqlen = 8 ;
            }
            
            // we've already made the necessary adjustments to bitqlen,
            // so do not add anything else (below)
            thesebits = 0 ;
            break ;
            
            default:
            // less than a full bit, or more than seven have elapsed
            // clear buffers
            msg_pos = 0 ;
            inaframe = 0 ;
            bitq = 0 ;
            bitqlen = 0 ;
            // do not add to bitqlen (below)
            thesebits = 0 ;
            // turn off DCD light
            DCD_OFF ;
            break ;
            
        } // end switch
        
        // how many bits did we add?
        bitqlen += thesebits ;
        
        
        //===================
        //   byte recovery
        //===================
        
        
        // got our bits in a row. now let's talk about bytes.
        
        // check the bit queue, more than once if necessary
        while ( bitqlen >= 8 )
        {
            // take the bottom 8 bits to make a byte
            byteval = bitq & 0xFF ;
            
            // special case - HDLC frame marker
            if ( byteval == 0x7E )
            {

                if ( inaframe == 0 )
                {
                    // marks start of a new frame
                    inaframe = 1 ;
                    last_symbol_inaframe = current_symbol ;
                    sync_err_cnt = 0 ;
                    bit_timer = 15 ;
                    bittimes = 0 ;
                    // pop entire byte (later)
                    popbits = 8 ;
                }
                
                else if ( msg_pos < MIN_PACKET_LEN )
                {
                    // We are already in a frame, but have not rec'd any/enough data yet.
                    // AX.25 preamble is sometimes a series of HDLCs in a row, so
                    // let's assume that's what this is, and just drop this byte.
                    popbits = 8 ;
                }
                
                else
                {
                    // in a frame, and have some data, so this HDLC is probably
                    // a frame-ender (and maybe also a starter)

                    
                    if ( msg_pos > 0 )
                    {   /*
                    printf( "Message was:" ) ;
                    for ( x=0 ; x < msg_pos ; x++ )
                    {
                    printf( " %02X", msg[x] ) ;
                    }
                    printf( "\n" ) ;
                    printf( "Which decodes as:\n" ) ;
                    */
                    // send frame data out the serial port
                    memcpy(msgcopy,msg,msg_pos);
                    OutputPacketSize = msg_pos;
                    //decode_ax25();
                }
                
                // stay in inaframe-mode, in case a new frame is starting
                msg_pos = 0 ;
                sync_err_cnt = 0 ;
                bittimes = 0 ;
                // pop entire byte (later)
                popbits = 8 ;
                
            }  // end else for "if not inaframe"
            
        }  // end if byteval = 0x7E
        
        else if ( inaframe == 1 )
        {
            // not an HDLC frame marker, but =is= a data character

            
            // add it to the incoming message
            msg[ msg_pos ] = byteval ;
            msg_pos++ ;
            
            // is this good enough of a KISS frame to turn on the carrier-detect light?
            // we know we have an HDLC (because we're in a frame);
            // check for end-of-header marker
            if (byteval == 0x03)
            {
                DCD_ON;
            }
            
            // pop entire byte (later)
            popbits = 8 ;
        }
        
        else
        {
            // not already in a frame, and this byte is not a frame marker.
            // It is possible (likely) that when an HDLC byte arrives, its 8 bits will not
            // align perfectly with the 8 we just checked. So instead of dropping all 8 of
            // these random bits, let's just drop one, and re-check the rest again later.
            // This increases our chances of seeing the HDLC byte in the incoming bitstream
            // amid noise (esp. if we're running open squelch).
            popbits = 1 ;
        }
        
        // pop the used bits off the end
        // (i know, not really a "pop", more of a "drop")
        bitq = ( bitq >> popbits ) ;
        bitqlen -= popbits ;
        
    } // end while bitqueue >= 8
    
    // debug: check timing
    //end_time = TCNT3 ;
    
    sei() ;
}
return ;
}  // end timerX interrupt

/**********************************************************************
*  Set up a generic timer that we can call (only enabled in transmit mode) 
*  that we use as a symbol timer so that it ticks at ~1200hz 
*
**********************************************************************/
SIGNAL(TCB0_INT_vect)
{
  maindelay = FALSE;					// Clear condition holding up mainDelay
  TCB0.INTFLAGS = 3; // clear the interrupt
  //test to see if we toggle fast enough....
  //TCNT2 = 255 - BIT_DELAY;
  //txtone = (txtone == MARK)? SPACE : MARK;
  
}

/******************************************************************************/
SIGNAL(USART0_RXC_vect)
/*******************************************************************************
* ABSTRACT:	Called by the receive ISR (interrupt). Saves the next serial
*				byte to the head of the RX buffer.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
  uint8_t rx;
  rx = USART0.RXDATAL;
  if (CmdModeState == CMD_MODE_KISS)
  {
    InputBuffer[InputEnd] = rx; // Transfer the byte to the input buffer
    if (++InputEnd == INBUF_SIZE)
      InputEnd = 0;                   // Advance and wrap buffer pointer
  }
  else
  {
    switch(rx)
    {
      case 0x08:    // backspace
        if (InputEnd > 0)
        {
          --InputEnd;
          send_serial_str("\b \b"); 
        }
        
        break;
      case 13:  
        CmdModeState = CMD_MODE_HANDLE_COMMAND;
        InputBuffer[InputEnd++] = 0;
        if (++InputEnd == INBUF_SIZE)
          InputEnd = 0;        
        SendRawByte(13);
        SendRawByte(10);
        break;
      default:
        if (InputEnd < INBUF_SIZE)
        {
          InputBuffer[InputEnd++] = rx;
          SendRawByte(rx);
        }
        break;
    }
  }
        
	return;

}		// End SIGNAL(SIG_UART_RECV)



/*******************************************************************************
* MsgHandler(char data) 
*  Takes in a byte at a time and determins what we should do with it from the 
*  serial port. This is what translates kiss data and spits it out the modem 
*  taking care of special characters 
*
*******************************************************************************/
void MsgHandler(uint8_t data)
{
   if ((data == 0x03) && (CmdModeState == CMD_MODE_KISS))
   {
       if (IsTransmitting == FALSE)
       {
           ++ctrlcCount;
           // 3 ^C's, get out of KISS mode
           if (ctrlcCount >= 3)
           {
               ctrlcCount = 0;
               CmdModeState = CMD_MODE_SHOW_PROMPT;
           }
       }
   }
   else
   {
       ctrlcCount = 0;
   }
   
   if (0xC0 == prevdata)
   {
     switch(data)
     {
      case 0x00:
        flagprev = 0;
        mainTransmit();
        break;
      default:
        // Set prev flag so we know to look at the next byte
        flagprev = 1;
        break;
     }
   }
   else if ((flagprev != 0))
   {
    flagprev = 0;
    switch(prevdata)
    {
      case 0x01: // txdelay
        params.TxDelay = data;
        break;
      case 0x02: // persistance
        params.Persistence = data;
        break;
      case 0x03: // persistance
        params.SlotTime = data;
        break;
      case 0x04: // persistance
        params.TxTail = data;
        break;
      case 0x05: // persistance
        params.FullDuplex = data;
        break;
      case 0x06: // persistance
        params.SetHardware = data;
        break;
    }
   }
  else if ((0xC0 == data) && (IsTransmitting == TRUE)) // now we have the end of a message
   {
    DCD_OFF;
//     HB_TOGGLE;  
     mainReceive();
   }
   else if(0xDC == data) // we may have an escape byte
   {  
        if(0xDB == prevdata)
        {
            ax25sendByte(0xC0);
        }
   }
   else if(0xDD == data) // we may have an escape byte
   { 
        if(0xDB == prevdata)
        {
            ax25sendByte(0xDB);
        }
   }  
  else if (TRUE == IsTransmitting)
    ax25sendByte(data); // if we are transmitting then just send the data
  
  prevdata = data; // copy the data for our state machine 
}

void SendRawByte(uint8_t val)
{
      while (!READY_TO_SEND)
        ;
    USART0.TXDATAL = val;
}

void SendHex8(uint8_t val, uint8_t withspace)
{
 

  SendRawByte(hexdata[val >> 4]);
  SendRawByte(hexdata[val & 0x0f]);
  hexwidth++;
  if (hexwidth > 15)
  {
    hexwidth = 0;
    SendRawByte(13);
    SendRawByte(10);
  }
  else
  {
    if (withspace)
    {
      SendRawByte(' ');
    }
  }
}

void SendHex16(uint16_t val)
{
  SendHex8(val >> 8,0);
  SendHex8(val,1);
}


void SendDataByte(uint8_t val)
{
  if (params.SetHardware & 1)
  {
    SendHex8(val,1);
  }
  else
  {
      SendRawByte(val);
  }
}

void decode_ax25 (void)
{
    // Take the data in the msg array, and send it out the serial port.
  uint8_t cp;
    uint8_t x;
    
    x = 0 ;
    decode_state = 0 ;     // 0=just starting, 1=header, 2=got 0x03, 3=got 0xF0 (payload data)

    //debug( "Decode routine - rec'd " . length($pkt) . " bytes." ) ;

    // lop off last 2 bytes (FCS checksum, which we're not sending to the PC)
  for (x = 0; x < (OutputPacketSize - 2); x++)
    {
        switch ( decode_state )  
        {
        // note the goofy order!!
        case 0:  
            // just starting
          hexwidth = 0;
          SendDataByte(0xc0);
          SendDataByte(0x00);
          SendDataByte(msgcopy[x]);
          decode_state = 1 ;
           break ;
        
        case 2: 
            // got the 0x03, waiting for the 0xF0
      if (msgcopy[x] == 0xF0)
            { 
              SendDataByte(msgcopy[x]);
                decode_state = 3 ; 
            }
            else 
            {
              SendDataByte(13);
              SendDataByte(10);
     	        return ;
            } 
            break ;
            
        case 1: 
            // in the header
      if (msgcopy[x] == 0x03)
            { 
              SendDataByte(msgcopy[x]);
                decode_state = 2 ; 
                break ; 
            }
            // else fall through
        
        default:
            // payload or header
      if (msgcopy[x] == 0xC0)
      {
        SendDataByte(0xdb);
      }
      SendDataByte(msgcopy[x]);
      if (msgcopy[x] == 0xDB)
      {
        SendDataByte(0xdd);
      }
            break ;
        }	

    } // end for	

  SendDataByte(0xc0);
  if (params.SetHardware & 1)
  {
    cp = msgcopy[x];
    SendHex8(cp,0);
    cp = msgcopy[x+1];
    SendHex8(cp,1);
    SendRawByte(13);
    SendRawByte(10);
  }
    SendRawByte(13);
    SendRawByte(10);

}


void send_serial_str(const char * str) 
{
    uint16_t x;
    for ( x=0 ; x < strlen( str ) ; x++ ) 
    {
    if (str[x] == 0)
    {
      return;
    }
    while (!READY_TO_SEND)
      ;
    USART0.TXDATAL = str[x];
    }
}


/******************************************************************************/
void mainTransmit(void)
/*******************************************************************************
* ABSTRACT:	Do all the setup to transmit.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
  DCD_ON;
#ifdef OPERMODE_MX614

  M0_ON;
  M1_OFF;
  txtone = MARK;  //set up the bits so we always start with the same tone for the header (otherwise it alternates) 
  TX_MARK;
#else
  sine_index = 0; // set our transmitter so it always starts the sine generator from 0
  txtone = MARK_PWM;  // set up the bits so we always start with the same tone for the header (otherwise it alternates)
  currentTone = 0;

#endif
  
	// enable overflow interrupt
  TCB0.CNT = 0;
  TCB0.INTCTRL = 3;
  TCB0.CTRLA = 3;
  IsTransmitting = TRUE;  // Enable the transmitter
  TX_ON;
	ax25sendHeader();	// Send APRS header
	return;

}		// End mainTransmit(void)


/******************************************************************************/
void mainReceive(void)
/*******************************************************************************
* ABSTRACT:	Do all the setup to receive or wait.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
  uint8_t dly;
	ax25sendFooter();		// Send APRS footer
  IsTransmitting = FALSE;  // Disable transmitter
  DCD_OFF;
#ifdef OPERMODE_MX614
  M0_OFF;
  M1_ON;
  TX_SPACE;
#endif

  // Change timer into 1ms for the following
//  TCA0.SINGLE.CMP0 = BIT_MS_DELAY;
  for (dly=0; dly < params.TxTail; dly++)
  {
    mainDelay();
  }
  TX_OFF;
  TCA0.SINGLE.CMP0 = T3TOP;
  TCB0.INTCTRL = 0; // DIsable interrupt
  HB_OFF;
#ifndef OPERMODE_MX614
  sine_index = 0; // set the sine back to 0 (redundant)
#endif

}		// End mainReceive(void)


/******************************************************************************/
void mainDelay(void)
/*******************************************************************************
* ABSTRACT:	This function sets "maindelay", programs the desired delay,
*			
*
* INPUT:		None
* OUT:	None
*/
{
	maindelay = TRUE;					// Set the condition variable
                    //  TCB0.CNT = 65536 - timeout;
	while(maindelay)
	{
            //asm("nop");//do something TODO process serial data here... 
	}
        //TCNT2 = (255 - BIT_DELAY);
        
	return;

}		// End mainDelay(unsigned int timeout)



/******************************************************************************/
void ax25sendHeader(void)
/*******************************************************************************
* ABSTRACT:	This function keys the transmitter, sends the source and
*				destination address, and gets ready to send the actual data.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
	static uint8_t	loop_delay;
	crc = 0xFFFF;							// Initialize the crc register
	// Transmit the Flag field to begin the UI-Frame
	// Adjust length for txdelay (each one takes 6.7ms)
	for (loop_delay = 0 ; loop_delay < params.TxDelay ; loop_delay++)
	{
		(ax25sendByte(0x7E));      //send the sync header byte
	}
	return;
}		// End ax25sendHeader(void)

/******************************************************************************/
void ax25sendFooter(void)
/*******************************************************************************
* ABSTRACT:	This function closes out the packet with the check-sum and a
*				final flag.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
	static uint8_t	crchi;
	crchi = (crc >> 8)^0xFF;
	ax25sendByte(crc^0xFF); 				// Send the low byte of the crc
	ax25sendByte(crchi); 					// Send the high byte of the crc
	ax25sendByte(0x7E);			  			// Send a flag to end the packet
	return;
}		// End ax25sendFooter(void)
/******************************************************************************/
void ax25sendByte(uint8_t txbyte)
/*******************************************************************************
* ABSTRACT:	This function sends one byte by toggling the "tone" variable.
*
* INPUT:		txbyte	The byte to transmit
* OUTPUT:	None
* RETURN:	None
*/
{
	static char	mloop;
	static char	bitbyte;
	static int	bit_zero;
	static uint8_t	sequential_ones;
    
	bitbyte = txbyte;							// Bitbyte will be rotated through

	for (mloop = 0 ; mloop < 8 ; mloop++)	// Loop for eight bits in the byte
	{
		bit_zero = bitbyte & 0x01;			// Set aside the least significant bit

		if (txbyte == 0x7E)					// Is the transmit character a flag?
		{
			sequential_ones = 0;				// it is immune from sequential 1's
		}
		else										// The transmit character is not a flag
		{
			(ax25crcBit(bit_zero));			// So modify the checksum
		}

		if (!(bit_zero))						// Is the least significant bit low?
		{
			sequential_ones = 0;				// Clear the number of ones we have sent
#ifdef OPERMODE_MX614			
      TX_INVERT;
#else
      currentTone = (currentTone == 0) ? 1 : 0;
      txtone = (txtone == MARK_PWM) ? SPACE_PWM : MARK_PWM; // Toggle transmit tone
#endif
		}
		else										// Else, least significant bit is high
		{
			if (++sequential_ones == 5)	// Is this the 5th "1" in a row?
			{
				mainDelay();		// Go ahead and send it
                                
#ifdef OPERMODE_MX614			
      TX_INVERT;
#else
      currentTone = (currentTone == 0) ? 1 : 0;
      txtone = (txtone == MARK_PWM) ? SPACE_PWM : MARK_PWM; // Toggle transmit tone
#endif
				sequential_ones = 0;			// Clear the number of ones we have sent
			}

		}

		bitbyte >>= 1;							// Shift the reference byte one bit right
		mainDelay();				// Pause for the bit to be sent
	}
	return;
}		// End ax25sendByte(uint8_t txbyte)

/*******************************************************************************/
void ax25crcBit(int lsb_int)
/*******************************************************************************
* ABSTRACT:	This function takes the latest transmit bit and modifies the crc.
*
* INPUT:		lsb_int	An integer with its least significant bit set of cleared
* OUTPUT:	None
* RETURN:	None
*/
{
	static unsigned short	xor_int;

	xor_int = crc ^ lsb_int;				// XOR lsb of CRC with the latest bit
	crc >>= 1;						// Shift 16-bit CRC one bit to the right

	if (xor_int & 0x0001)					// If XOR result from above has lsb set
	{
		crc ^= 0x8408;					// Shift 16-bit CRC one bit to the right
	}

	return;
}		// End ax25crcBit(int lsb_int)

uint8_t Crc16(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint8_t j;
    uint16_t crclocal;
    uint16_t crcframe;
    
    crcframe = buf[len-1];
    crcframe <<= 8;
    crcframe |= buf[len-2];
    
    crclocal = 0xffff;
    for (i=0; i<len-2; i++)
    {
        crclocal ^= buf[i];
        for (j=0; j<8; j++)
        {
          
            if (crclocal & 1)
                crclocal = (crclocal >> 1) ^ 0x8408; //0xa001;
            else
                crclocal = (crclocal >> 1);
        }
    }    
    crclocal ^= 0xffff;
    if (crcframe == crclocal)
        return 0;
    else
        return 1;
        
}

void SendPacket(uint8_t *buf, uint16_t len)
{
  uint16_t i;
  if (len == 0) 
    return;
  mainTransmit();
  for (i=0; i<len; i++)
  {
    ax25sendByte(buf[i]);
  }
  mainReceive();
}

uint8_t LenOrToDash(char *str)
{
  uint8_t i;
  i=0;
  while ((str[i] != 0) && (str[i] != '-'))
    i++;
  return i;
}

uint8_t GetID(char *str)
{
  char *trail;
  uint8_t id;

  strtok_r(str,"-", &trail);
  id = 0;
  if (trail != NULL)
  id = atoi(trail);

  return id;
}

uint8_t DoSetCall(const char *CallType, char *str, char *callLoc, uint8_t *id)
{
  uint16_t i;
  uint16_t len;
  char bnew[7];
  char ch;
  uint8_t isNone;

  isNone = 0;
  if (str == NULL)
  {
      serprintln("%s is %s",CallType, callLoc);
    return 1;
  }    
  if (id == NULL)
    return -1;
  len = LenOrToDash(str);
  if (len > 6)
  {
    serprintln("arg is too long");
    return 1;
  }
  else
  {
    if ((len >=4) && (strncasecmp(str,"none",4) == 0))
      isNone = 1;
    i=0;
    while (i<len)
    {
      if (isNone)
        ch = ' ';
      else
        ch = toupper(str[i]);
      callLoc[i] = ch;
      bnew[i] = callLoc[i];
      i++;
    }
    while (i<6)
    {
      callLoc[i] = ' ';
      bnew[i] = ' ';
      i++;
    }
    bnew[6] = 0;
    *id = GetID(str);
    serprintln("Setting %s to '%s'-%d",CallType, bnew,*id);
    SaveEepromContents(&params);
  }
  return 0;
}

void handleMyCall(char *str)
{
    DoSetCall("MyCall",str, (char*)params.MyCall, (uint8_t*)&params.MyId);
}

void handleUnproto(char *str)
{
  DoSetCall("Unproto",str, (char*)params.DestCall, (uint8_t*)&params.DestId);
}

void handleDigi1Call(char *str)
{
  DoSetCall("Digi1",str, (char*)params.Digi1, (uint8_t*)&params.Digi1Id);
}

void handleDigi2Call(char *str)
{
  DoSetCall("Digi1",str,(char*) params.Digi2, (uint8_t*)&params.Digi2Id);
}

void handleCW(char *str)
{
  DecodeMorse(str);
}

void handleTxDelay(char *str)
{
  uint16_t val;
  if ((str == NULL) || (strlen(str) < 1))
  {
    serprintln("TXDelay is %d",params.TxDelay);
    return;
  }
  serprintln("Processing '%s'",str);
  val = atoi(str);
  params.TxDelay = val;
  serprintln("TXDelay set to %d",params.TxDelay);
  SaveEepromContents(&params);

}
// 86 A2 40 40 40 40 60  AE 72 94 82 94 40 60  88 92 8E 92 62 40 60  88 92 8E 92 64 40 61  03 F0 63 6B 0D 
uint8_t GetCallsign(char *csbuf, uint8_t *buf)
{
  uint8_t i;
  uint8_t val;
  uint8_t ssid;
  i=0;
  while (i<6)
  {
    csbuf[i] = buf[i]>>1;
    i++;
  }
  while ((i>0) && (csbuf[i-1] == ' '))
  {
    csbuf[i-1] = 0;
    --i;
  }    
  val = buf[6];
  ssid = (val & 0x1e) >> 1;
  if (ssid != 0)
  {
    csbuf[i] = '-';
    sprintf(&csbuf[i],"%d",ssid);
  }
  val &= 0x01;
  //if (val == 1)
//    val = 1;

  return val;
}

void DumpFrame(uint8_t *buf, uint16_t len)
{
  uint16_t left;
  uint8_t ret;
  uint8_t *p;
  uint8_t val;
  uint8_t showAscii;
  static char callsign[12];
  uint8_t crcMatch;
  
  if (len < 2)
    return;

  memset(frameBuffer,0,sizeof(frameBuffer));
  left = len-2;

  crcMatch = Crc16(buf, len);
  if (crcMatch)
  {
    frameBuffer[0] = '*';
  }

  
  p=buf;
  ret = 0;
  if (left > 14)
  {
    ret=GetCallsign(callsign,p+7);
    strcat(frameBuffer,callsign);
    strcat(frameBuffer,">");
    GetCallsign(callsign,p);
    strcat(frameBuffer,callsign);
    p+=14;
    left -= 14;
  }
  // Are there digipeaters?
 
  if (ret == 0)
  {
    strcat(frameBuffer," (");
    if (left > 7)
    {
      ret=GetCallsign(callsign,p);
      p+=7;
      left -= 7;
    }
    strcat(frameBuffer,callsign);
    if (ret == 0)
    {
      strcat(frameBuffer,",");
      if (left > 7)
      {
        ret=GetCallsign(callsign,p);
        strcat(frameBuffer,callsign);
        p+=7;
        left -= 7;
      }
    }
    strcat(frameBuffer,") ");
  }
    strcat(frameBuffer,": ");
    // Get control (03)
    val = *p;
    p++;
    if ((val & 1) == 1)
    {
        // Unnumbered frame
        sprintf(callsign, "<UI>:");
        showAscii=1;
    }
    else
    {
        showAscii=0;
        if ((val & 2)==0)        
        {
            sprintf(callsign,"<S:%d,%d>:",(val & 0xe0)>>5, (val & 0x0c)>>2 );
        }
        else
        {
            sprintf(callsign,"<I:%d,%d>:",(val & 0xe0)>>5, (val & 0x0e)>>1 );
        }
    }
    strcat(frameBuffer,callsign);
    // 0xF0 here
    p++;
    serprintln("%s",frameBuffer);
    hexwidth=0;
    frameBuffer[0] = 0;
    if (showAscii == 0)
    {
        
        while (left > 0)
        {
          callsign[0] = hexdata[(*p) >> 4];
          callsign[1] = hexdata[(*p++) & 0x0f];
          callsign[2] = ' ';
          callsign[3] = 0;

          strcat(frameBuffer, callsign);
          ++hexwidth;
          if (hexwidth>15)
          {
            hexwidth=0;
            serprintln("%s",frameBuffer);
            frameBuffer[0] = 0;
          }
          --left;
        }
    }
    else
    {
        if (left > 0)
        {
            strncat(frameBuffer,(char*)p,left-2);
            p+=left;
        }
    }        
    if (strlen(frameBuffer) > 0)
    {
        serprintln("%s",frameBuffer);
    }

}

void handleBroadcast(char *str)
{
  // Construct a packet to broadcast
  uint8_t *p;
  uint16_t i;
  uint8_t id;
  
  p=TxFrame;
  
  for (i=0; i<6; i++)
  {
    *p++ = params.DestCall[i] << 1;
  }
  *p++ = 0x60 + (params.DestId << 1);
  for (i=0; i<6; i++)
  {
    *p++ = params.MyCall[i] << 1;
  }
  if (params.Digi1[0] == ' ')
    id = 0x61;
  else
    id = 0xe0;
  *p++ = id + (params.DestId << 1);
  if (params.Digi1[0] != ' ')
  {
    for (i=0; i<6; i++)
    {
      *p++ = params.Digi1[i] << 1;
    }
    if (params.Digi2[0] == ' ')
      id = 0x61;
    else
      id = 0x60;
    *p++ = id + (params.Digi1Id << 1);
    if (params.Digi2[0] != ' ')
    {
      for (i=0; i<6; i++)
      {
        *p++ = params.Digi2[i] << 1;
      }
      id = 0x61;
      *p++ = id + (params.Digi2Id << 1);
    }
  }

  *p++ = 0x03;
  *p++ = 0xf0;
  while (*str != 0)
  {
    *p++ = *str++;
  }
  TxSize = (p - TxFrame);
  DumpFrame(TxFrame,  TxSize);
  SendPacket(TxFrame,  TxSize);
  TxSize = 0;
}

uint8_t OnOff(char *str)
{
  uint8_t ret;
  ret = 0;
  if (strcasecmp(str,"ON") == 0)
    ret = 1;
  return ret;
}



const char *GetOnOff(uint8_t val)
{
  if (val != 0)
    val = 1;
  if (val)
    return ON;
  return OFF;
}

void handleMon(char *str)
{
  uint16_t val;
  if ((str == NULL) || (strlen(str) < 1))
  {
    serprintln("Monitor is %s",GetOnOff(params.Monitor));
    return;
  }

  val = OnOff(str);
  params.Monitor = val;
  serprintln("Monitor set to %s", GetOnOff(val));
  SaveEepromContents(&params);
}

void handleUsage(char *str)
{
  uint16_t i;

  i=0;
  while (cmdList[i].Cmd != NULL)
  {
    serprintln("%-10s - %s",cmdList[i].Cmd, cmdList[i].Usage);
    i++;
  }
}

void handleBeaconText(char *str)
{
    if ((str == NULL) || (strlen(str) == 0))
    {
        serprintln("Beacon Text: %s", params.IdText);
    }
    else
    {
        strncpy((char*)params.IdText,str, sizeof(params.IdText)-1);
    }
    SaveEepromContents(&params);
}

void handleBeaconInt(char *str)
{
    uint8_t val;
    if ((str == NULL) || (strlen(str) == 0))
    {
        serprintln("Beacon every %d min", params.IdInterval);
    }    
    else
    {
        val = atoi(str);
        if (val > 60)
        {
            val = 60;
        }
        params.IdInterval = val;
        SaveEepromContents(&params);
    }
}

/******************************************************************************/
void	ProcessCmd(void)
/*******************************************************************************
* ABSTRACT: We have input -- go process it now
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
  char *last;
  char *command;
  uint16_t i;

  command = strtok_r((char*)InputBuffer, " ", &last);
  if (command != NULL)
  {
//    serprintln("command is %s, last is %s", command, last);
    i=0;
    while (cmdList[i].Cmd != NULL)
    {
      if (strncasecmp(command, cmdList[i].Cmd, strlen(command)) == 0)
      {
        (*cmdList[i].cb)(last);
        break;
      }
      i++;
    }
    // If command not found show the usage message
    if (cmdList[i].Cmd == NULL)
    {
        handleUsage(NULL);
    }
  }
}

/******************************************************************************/
void	Serial_Processes(void)
/*******************************************************************************
* ABSTRACT:	Called by main.c during idle time. Processes any waiting serial
*				characters coming in or going out both serial ports.
*
* INPUT:		None
* OUTPUT:	None
* RETURN:	None
*/
{
  if (InputEnd != InputBeg) // If there are incoming bytes pending
  {
    while (InputEnd != InputBeg)
    {
      MsgHandler(InputBuffer[InputBeg++]); // And pass it to a handler
      if (InputBeg == INBUF_SIZE)
        InputBeg = 0;              // Advance and wrap pointer
    }
  }
  else
  {
    switch(CmdModeState)
    {
      case CMD_MODE_KISS:
        break;
      case CMD_MODE_SHOW_PROMPT:
        send_serial_str(Prompt);
        CmdModeState =  CMD_MODE_WAIT_INPUT;
        break;
      case CMD_MODE_WAIT_INPUT:
        // Nothing to do here
        break;
      case CMD_MODE_HANDLE_COMMAND:
        // We have a command - go process it
        ProcessCmd();
        CmdModeState=CMD_MODE_CLEAR_COMMAND;
        break;
      case CMD_MODE_CLEAR_COMMAND:
        InputBeg = 0;
        InputEnd = 0;
        if (kissMode != 0)
        {
            CmdModeState = CMD_MODE_KISS;        
        }
        else
        {
            CmdModeState = CMD_MODE_SHOW_PROMPT;
        }
        break;
      default:
        CmdModeState = CMD_MODE_SHOW_PROMPT;
        break;
    }

  }
  if (OutputPacketSize != 0)
  {
    DCD_ON;
    if (CmdModeState == CMD_MODE_KISS)
    {
        decode_ax25();
    }
    else
    {
        DumpFrame(msgcopy,OutputPacketSize);
    }        
    DCD_OFF;
    OutputPacketSize = 0;
  }
}		// End Serial_Processes(void)


void SPIWrite(uint8_t *buf, uint16_t len)
{
    uint16_t i;
     SPI0.CTRLB = (SPI0.CTRLB & 0xfc) | 0;
    SS_LOW;    
    for (i=0; i<len; i++)
    {
        // Wait for the transfer to complete
        SPI0.DATA = buf[i];
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
            ;
    }
    SS_HIGH;
}

void SPIXfer(uint8_t *buf, uint16_t len, uint8_t mode)
{
    uint16_t i;
    SPI0.CTRLB = (SPI0.CTRLB & 0xfc) | mode;
    for (i=0; i<len; i++)
    {
        // Wait for the transfer to complete
        SPI0.DATA = buf[i];
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
        // Put the read data into the buffer
        buf[i] = SPI0.DATA;
    }
}

void ReadSPI(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    SS_LOW;
    for (i=0; i<len; i++)
    {
        // Wait for the transfer to complete
        SPI0.DATA = buf[i];
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
        // Put the read data into the buffer
        buf[i] = SPI0.DATA;
    }
    SS_HIGH;
}

void SpiWriteRead(uint8_t *buf, uint16_t readsize, uint8_t txnum)
{
    uint16_t i;
    SS_LOW;
    // Write in mode 2, Read in mode 3
     SPI0.CTRLB = (SPI0.CTRLB & 0xfc) | 0;
    for (i=0; i<txnum; i++)
    {
        // Wait for the transfer to complete
        SPI0.DATA = buf[i];
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
        // Put the read data into the buffer
        buf[i] = SPI0.DATA;
    }
    // Write in mode 2, Read in mode 3
    SPI0.CTRLB = (SPI0.CTRLB & 0xfc) | 1;
    for (i=0; i<readsize; i++)
    {
        // Clock out 0s
        SPI0.DATA = 0;
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
        // Put the read data into the buffer
        buf[i] = SPI0.DATA;
    }
    SS_HIGH;
    
}

void SpiReadMemory(uint32_t addr, uint8_t *buf, uint16_t len)
{
    buf[0] = 0x02;
    buf[1] = (addr >> 16) & 0xff;
    buf[2] = (addr >> 8) & 0xff;
    buf[3] = (addr & 0xff);
    SpiWriteRead(buf, len, 4);
}

void SpiWriteMemory(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t tbuf[4];
    uint16_t i;
    
     tbuf[0] = 1;
     tbuf[1] = (addr >> 16) & 0xff;
     tbuf[2] = (addr >> 8) & 0xff;
     tbuf[3] = (addr & 0xff);   
     
    SPI0.CTRLB = (SPI0.CTRLB & 0xfc) | 0;
    SS_LOW;
    for (i=0; i<4; i++)
    {
        // Wait for the transfer to complete
        SPI0.DATA = tbuf[i];
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
    }
    for (i=0; i<4; i++)
    {
        SPI0.DATA = 0;
        // Wait for the transfer to complete
        while ((SPI0.INTFLAGS & SPI_IF_bm) == 0)
        ;
        buf[i] = SPI0.DATA;
    }    
    SS_HIGH;    
}

uint8_t SpiBuffer[32];

void SetupRAM(void)
{
    static uint16_t j=0;
    memset(SpiBuffer,0,sizeof(SpiBuffer));
    
    SpiBuffer[0] = 2; // Write memory
    SpiBuffer[2] = 1;
    SpiBuffer[4] = 0x22+j;
    SpiBuffer[5] = 0x33+j;
    SpiBuffer[6] = 0x44+j;
    SpiBuffer[7] = 0x55+j;
    j++;
    SPIWrite(SpiBuffer,8);
    
    memset(SpiBuffer,0,sizeof(SpiBuffer));
    SpiBuffer[0] = 3; // Mode register
    SpiBuffer[2] = 1;
    SpiWriteRead(SpiBuffer,4,4);
    serprintln("SPI buffer contains %02x %02x %02x %02x",SpiBuffer[0], SpiBuffer[1], SpiBuffer[2], SpiBuffer[3]);

}

void CheckTimers(void)
{
    if (BeaconTimer != 0)
    {
        if (getRTCCount() > BeaconTimer)
        {
            UpdateBeaconTimer();
            handleBroadcast((char*)params.IdText);
        }
    }
}

void setup(void)
{
  // hardware USART0 (MEGA or Due)
  // timer value of 19.2kbps serial output to match bootloader
  PORTMUX.USARTROUTEA = 0;
  USART0.BAUD = 4166; // 2083 for 38400, 4166 for 19200; //  USART0_BAUD_RATE(BAUD_RATE) ((float)(16000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)
  //    UCSR0A |= (1<<U2X0) ;
  USART0.CTRLB |= 0xc0; // (1<<TXEN0) | (1<<RXEN0) ;
  USART0.CTRLC = 0x03;  // 8 data bits
  USART0.CTRLA |= 0x80; // enable rx interrupt
#ifndef OPERMODE_MX614
//  PORTMUX.TCAROUTEA = 1;
  ADC0.CTRLB = 7;         // pre-scale 16
  ADC0.CTRLC = (20 << 3); // Internal reference Vcc, timebase 20
  ADC0.CTRLF = 0x30;      // Free run, left adjust
  ADC0.CTRLE = 0x1b;      // Sample Duration
  ADC0.MUXPOS = 0x0a;     // Ain10
  ADC0.MUXNEG = 0x30;     // Default (Ground)
  ADC0.DBGCTRL = 1;       // Debug run
//  PORTA.PIN3CTRL = 4;     // Disable digital pin PA3
  PORTB.PIN1CTRL = 4;     // Disable digital pin PB0
  PORTA.PIN2CTRL = 8;     // pull up on SIO pin
  ADC0.CTRLA = 0xa1; // Run standby, Enable
  ADC0.COMMAND = 1;  // Start a conversion
  currentTone = 0;  
#endif  
  SET_DDRB;
    PORTA.OUT = 0;
    SS_HIGH;
//  PORTB.OUT = 0x02;
//  ADC0.CTRLA = 0xa1; // Run standby, Enable
//  ADC0.COMMAND = 1;  // Start a conversion
  
  OutputPacketSize = 0; // No frames to send
  MonitorFlag = 0; // No monitoring 
  // use 16-bit timer to check the ADC at 13200 Hz.
  // this is 11x the 1200Hz MARK freq, and 6x the 2200Hz SPACE freq.
  // Configure the SPI bus
  // Mode 0, maybe check SPI_BUFEN_bm
  SPI0.CTRLB = 2 | SPI_SSD_bm;
  // MSB first, Enable bus, double clock speed SPI_CLK2X_bm
  SPI0.CTRLA = SPI_ENABLE_bm | SPI_PRESC_1_bm |  SPI_MASTER_bm;
  SPI0.DATA = 1;
  
  // use Timer1 on Arduino Duemilanove (ATmega328P)
  TCA0.SINGLE.CTRLD = 0;
  TCA0.SINGLE.CTRLA = 0;
  TCA0.SINGLE.INTCTRL = 1;
#ifdef OPERMODE_MX614
  TCA0.SINGLE.CTRLB = 0x01; // FREQ mode - may be incorrect
  TCA0.SINGLE.EVCTRL = 0;
  TCA0.SINGLE.CMP0 = T3TOP;
  TCA0.SINGLE.CMP0 = T3TOP;
  TCA0.SINGLE.CMP0 = T3TOP;
  TCA0.SINGLE.CTRLA = 3;
#else
  TCA0.SINGLE.CTRLB = 0x15; // FREQ mode - may be incorrect
  TCA0.SINGLE.EVCTRL = 0;
  TCA0.SINGLE.PER = T3TOP;
  TCA0.SINGLE.CMP0 = T3TOP/2;
  TCA0.SINGLE.CTRLA = 1;
#endif  
  TCB0.INTCTRL = 3; // Overflow interrupt
  TCB0.CCMP = BIT_DELAY;
  TCB0.CTRLB = 0;
  TCB0.CTRLA = 3; // prescaler of 2, enable
  // use blinky on "pin 13" as DCD light, use "pin 12" as sample clock heartbeat
  // the port/bit designation for these Arduino pins varies with the chip used
  // see the chip-specific DEFINEs above for details
  SetupRTC();
  CmdModeState=CMD_MODE_SHOW_PROMPT;
  SET_DDRA;
  PORTA.OUT = 0;
#if 0
  for (x=0; x<100; x++)
  {  
    SetupRAM();
    local_delay(50);
  }  
#endif    
    // Turn on M0, M1
#ifdef OPERMODE_MX614	
    M0_OFF;
    M1_ON;   // TX mode  for M1
#else    
    TX_OFF;
#endif	
    ReadEepromContents(&params);
    if (params.StartMode != 0)
    {
        CmdModeState=CMD_MODE_KISS;
    }
    else
    {
        send_serial_str( WELCOME_MSG ) ;
    }
    
    UpdateBeaconTimer();
    InputEnd = 0;
    InputBeg = 0;
    // enable interrupts
    sei() ; 
    
    // pause again for ADC bias to settle
    local_delay(10);
}

void loop (void)
{
    Serial_Processes();
    CheckTimers();
}
/*
int main(void)
{
    setup();
    while (1)
    {
        loop();
    }
    return 0;
}
// end of file
*/