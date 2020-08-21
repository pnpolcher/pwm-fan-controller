#define _XTAL_FREQ 16000000

#include <pic16f1503.h>
#include <xc.h>

#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF
#pragma config WRT = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config LVP = ON


#define TMP_MAX_SAMPLES     16
#define TMP_1_REG           0x00
#define TMP_2_REG           0x01
#define TMP_1_RAW_REG       0x02
#define TMP_2_RAW_REG       0x03
#define WRSET_STATUS_REG    0x04

#define TEMP_LUT_SIZE   101

#define CMD_READ_REGISTER   0x01
#define CMD_WRITE_REGISTER  0x81
#define CMD_READ_SETTING    0x02
#define CMD_WRITE_SETTING   0x82

#define FSM_IDLE            0x00
#define FSM_CMD_SETUP       0x01
#define FSM_CMD_REG         0x02
#define FSM_CMD_ARG         0x03
#define FSM_CMD_ARMED       0x04

typedef unsigned char byte;

volatile byte command = 0;
volatile byte cmd_reg = 0;
volatile byte cmd_arg = 0;
volatile byte register_map[16] = {0,};
volatile byte i2c_byte_count = 0;

volatile unsigned short adc_tmp_1 = 0;
volatile unsigned short adc_tmp_2 = 0;
volatile byte tmp_1_cnt = 0;
volatile byte tmp_2_cnt = 0;

volatile unsigned short row_buffer[16];

volatile byte fsm_state = FSM_IDLE;

const unsigned short pref_addrs[2] = {
    0x780,
    0x782
};

const unsigned short temp_lut[TEMP_LUT_SIZE] = {
    941, 938, 935, 932, 928, 925, 922, 918, 914, 910, // 0..9
    906, 902, 898, 893, 888, 883, 878, 873, 867, 861, // 10..19
    856, 849, 843, 837, 830, 823, 816, 808, 801, 793, // 20..29
    785, 776, 768, 759, 750, 741, 731, 721, 712, 701, // 30..39
    691, 681, 670, 659, 648, 637, 625, 614, 602, 590, // 40..49
    579, 567, 554, 542, 530, 518, 505, 493, 481, 468, // 50..59
    456, 444, 432, 420, 407, 395, 384, 372, 360, 349,
    337, 326, 315, 304, 294, 283, 273, 263, 253, 244,
    235, 225, 217, 208, 200, 191, 183, 176, 168, 161,
    154, 147, 141, 135, 129, 123, 117, 112, 106, 101,
    96
};

short get_temp(unsigned short code)
{
    short left = 0;
    short right = TEMP_LUT_SIZE-1;
    
    while (left < right)
    {
        short middle = (left + right) / 2;
        if (temp_lut[middle] > code)
        {
            left = middle + 1;
        }
        
        else if (temp_lut[middle] < code)
        {
            right = middle - 1;
        }
        else
        {
            return middle;
        }
    }
    
    return right;
}

int eeprom_read(unsigned short addr)
{
    byte gie_status = INTCONbits.GIE;
    INTCONbits.GIE = 0;

    PMADR = addr;
    PMCON1bits.CFGS = 0;
    PMCON1bits.RD = 1;
    NOP();
    NOP();

    INTCONbits.GIE = gie_status;
    return PMDAT;
}

void eeprom_erase(unsigned short addr)
{
    byte gie_status = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    PMADRL = (addr & 0xff);
    PMADRH = ((addr & 0xff00) >> 8);
    PMCON1bits.CFGS = 0;
    PMCON1bits.FREE = 1;
    PMCON1bits.WREN = 1;
    
    PMCON2 = 0x55;
    PMCON2 = 0xAA;
    PMCON1bits.WR = 1;
    NOP();
    NOP();
        
    PMCON1bits.WREN = 0;
    INTCONbits.GIE = gie_status;
}

void eeprom_write(unsigned short addr)
{
    byte addr_lo = (byte)(addr & 0xff);
    byte addr_hi = (byte)((addr & 0xff00) >> 8);

    byte gie_status = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    PMCON1bits.CFGS = 0;
    PMCON1bits.WREN = 1;
    PMCON1bits.LWLO = 1;
    
    for (byte pos = 0; pos < 16; pos++)
    {
        PMADRL = addr_lo;
        PMADRH = addr_hi;
        
        PMDATL = row_buffer[pos];
        PMDATH = ((row_buffer[pos] & 0xff00) >> 8);

        if (pos == 15)
        {
            PMCON1bits.LWLO = 0;
        }
        
        PMCON2 = 0x55;
        PMCON2 = 0xAA;
        PMCON1bits.WR = 1;
        NOP();
        NOP();

        addr_lo++;
        if (addr_lo == 0)
        {
            addr_hi++;
        }

        CLRWDT();
    }

    PMCON1bits.WREN = 0;
    INTCONbits.GIE = gie_status;
}

void write_preference(byte preference, byte data)
{
    unsigned short addr = 0x780;
    register_map[WRSET_STATUS_REG] = 0x00;
    
    INTCONbits.GIE = 0;
    for (byte i = 0; i < 16; i++)
    {
        row_buffer[i] = eeprom_read(addr);
        addr += 2;
        CLRWDT();
    }
    
    eeprom_erase(0x780);
    row_buffer[preference] = (unsigned short)data;
    eeprom_write(0x780);
    register_map[WRSET_STATUS_REG] = 0xaa;
    INTCONbits.GIE = 1;
}

byte process_command()
{
    byte result;
    
    switch (command)
    {
        case CMD_READ_REGISTER:
            result = register_map[cmd_reg % sizeof(register_map)];
            break;
            
        case CMD_WRITE_REGISTER:
            register_map[cmd_reg % sizeof(register_map)] = cmd_arg;
            break;
            
        case CMD_READ_SETTING:
            result = eeprom_read(pref_addrs[cmd_reg % sizeof(pref_addrs)]);
            break;
            
        case CMD_WRITE_SETTING:
            write_preference(cmd_reg % sizeof(pref_addrs), cmd_arg);
            result = 0xc0;
            break;
            
        default:
            result = 0xfe;
            break;
    }
    
    fsm_state = FSM_IDLE;
    
    return result;
}

void __interrupt() isr(void)
{
    byte b;
    
    if (PIR1bits.SSP1IF == 1)
    {
        SSPCONbits.CKP = 0;
        
        // If there was an overflow or a write collision.
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            b = SSP1BUF;
            SSPCONbits.SSPOV = 0;
            SSPCONbits.WCOL = 0;
            SSPCONbits.CKP = 1;
        }
        
        // If the received byte is the I2C address.
        if (!SSP1STATbits.D_nA)
        {
            i2c_byte_count = 0;
            
            // Discard slave address.
            b = SSP1BUF;
            
            // If the master wants to read.
            if (SSP1STATbits.R_nW)
            {
                if (fsm_state == FSM_CMD_ARMED)
                {
                    SSP1BUF = process_command();
                }
            }
        }
        else // Data bytes are being send/requested by the master.
        {
            // Increment bytes processed count.
            i2c_byte_count++;
            
            if (i2c_byte_count == 1)
            {
                fsm_state = FSM_CMD_SETUP;
            }
            
            // Reset BF.
            b = SSP1BUF;
            
            // Master wants to read data.
            if (SSP1STATbits.R_nW)
            {
                // Load the data (to send to the master) into the transmit
                // buffer.
                if (fsm_state == FSM_CMD_ARMED)
                {
                    SSP1BUF = process_command();
                }
            }
            else
            {
                switch (fsm_state)
                {
                    case FSM_CMD_SETUP:
                        fsm_state = FSM_CMD_REG;
                        command = b;
                        break;
                        
                    case FSM_CMD_REG:
                        cmd_reg = b;
                        fsm_state = FSM_CMD_ARG;
                        break;
                        
                    case FSM_CMD_ARG:
                        cmd_arg = b;
                        fsm_state = FSM_CMD_ARMED;
                        if ((command & 0x80) == 0x80)
                        {
                            process_command();
                        }
                        
                        break;
                        
                    default:
                        fsm_state = FSM_IDLE;
                        break;
                }
            }
        }
        
        // Clear interrupt flag.
        PIR1bits.SSP1IF = 0;
        
        // Release the clock (clock stretch, SEN, enabled).
        SSPCONbits.CKP = 1;
    }
    
    if (PIR1bits.ADIF == 1)
    {
        
        if (ADCON0bits.CHS == 0b00110)
        {
            if (tmp_1_cnt == TMP_MAX_SAMPLES)
            {
                unsigned short code = (adc_tmp_1 / TMP_MAX_SAMPLES);
                register_map[TMP_1_REG] = get_temp(code);
                register_map[TMP_1_RAW_REG] = (byte)(code >> 2);
                adc_tmp_1 = 0;
                tmp_1_cnt = 0;
            }
            else
            {
                adc_tmp_1 += ((ADRESH & 0x03) << 8) + ADRESL;
                tmp_1_cnt++;
            }
            
            // Switch to RA4/AN3.
            ADCON0bits.CHS = 0b00011;
        }
        else
        {
            if (tmp_2_cnt == TMP_MAX_SAMPLES)
            {
                register_map[TMP_2_REG] = (byte)((adc_tmp_2 / TMP_MAX_SAMPLES) >> 2);
                adc_tmp_2 = 0;
                tmp_2_cnt = 0;
            }
            else
            {
                adc_tmp_2 += ((ADRESH & 0x03) << 8) + ADRESL;
                tmp_2_cnt++;
            }
            
            // Switch to RC2/AN6.
            ADCON0bits.CHS = 0b00110;
        }
        
        PIR1bits.ADIF = 0;
    }
}

void main(void)
{
    // 16 MHz speed.
    OSCCONbits.IRCF = 0xf;
    OSCCONbits.SCS = 0x3;
//    OSCCON = 0b01111011;
    PORTA = 0;
    PORTC = 0;
    LATA = 0;
    LATC = 0;
    ANSELA = 0;
    ANSELC = 0;
    TRISA = 0b00111111;
    TRISC = 0b00111111;
    
    PWM1CON = 0;
    PWM2CON = 0;
    PR2 = 0x7c;
    PWM1DCH = 0;
    PWM1DCL = 0;
    PWM2DCH = 0;
    PWM2DCL = 0;

    // Set up I2C module as a 7-bit slave on address 0x30.
    SSP1STAT = 0x80; // Slew rate control disabled.
    SSP1ADD = 0x30 << 1;
    SSP1CON1 = 0b00110110;
    SSP1CON2 = 0b00000001; // Enable clock stretching.
    
    // Set up Timer 0.
    OPTION_REGbits.nWPUEN = 1;
    OPTION_REGbits.TMR0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b111;
    
    // Set up the ADC module.
    ADCON2bits.TRIGSEL = 0b0011; // Triggered by Timer0 overflow.
    ADCON1 = 0b11110000; // FRC
    ANSELAbits.ANSA4 = 1;
    ANSELCbits.ANSC2 = 1;
    ADCON0bits.CHS = 0b00110; // Select AN6/RC2.
    ADCON0bits.ADON = 1;
    
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b01;
    T2CONbits.TMR2ON = 1;
    while (PIR1bits.TMR2IF == 0);
    
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    
    PWM1DCH = 0x1f;
    PWM2DCH = 0x44;

    PWM1CONbits.PWM1OE = 1;
    PWM2CONbits.PWM2OE = 1;
    
    PWM1CON = 0b11000000;
    PWM2CON = 0b11000000;
  
//    PWM1DCH = 0b10001000;
//    PWM1DCL = 0b10000000;
//    PWM2DCH = 0b01000100;
//    PWM2DCL = 0b01000000;
    
    PIR1bits.SSP1IF = 0;     // Clear MSSP interrupt request flag.
    PIE1bits.SSP1IE = 1;     // Enable MSSP interrupt.
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.GIE = 1;     // Global Interrupt Enable bit on.
    INTCONbits.PEIE = 1;    // Peripheral Interrupt Enable bit on.

    // Enable ADC conversion.
    ADCON0bits.ADGO = 1;

    while(1) {
        __delay_ms(10);
        // PORTCbits.RC4 ^= 1;
    }
}

