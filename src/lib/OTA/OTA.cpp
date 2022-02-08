/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

#include "OTA.h"

static inline uint8_t ICACHE_RAM_ATTR HybridWideNonceToSwitchIndex(uint8_t nonce)
{
    // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
    // 0, 1, 2, 3, 4, 5, 6, 7,
    // 1, 2, 3, 4, 5, 6, 7, 0
    // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
    // this makes sure each of the 8 values is sent at least once every 16 packets
    // regardless of the TLM ratio
    // Index 7 also can never fall on a telemetry slot
    return ((nonce & 0b111) + ((nonce >> 3) & 0b1)) % 8;
}

#if TARGET_TX or defined UNIT_TEST

// Current ChannelData generator function being used by TX
PackChannelData_t PackChannelData;

static void ICACHE_RAM_ATTR PackChannelDataHybridCommon(volatile uint8_t* Buffer, CRSF *crsf)
{
    Buffer[0] = RC_DATA_PACKET & 0b11;
    Buffer[1] = ((crsf->ChannelDataIn[0]) >> 3);
    Buffer[2] = ((crsf->ChannelDataIn[1]) >> 3);
    Buffer[3] = ((crsf->ChannelDataIn[2]) >> 3);
    Buffer[4] = ((crsf->ChannelDataIn[3]) >> 3);
    Buffer[5] = ((crsf->ChannelDataIn[0] & 0b110) << 5) |
                            ((crsf->ChannelDataIn[1] & 0b110) << 3) |
                            ((crsf->ChannelDataIn[2] & 0b110) << 1) |
                            ((crsf->ChannelDataIn[3] & 0b110) >> 1);
}

/**
 * Hybrid switches packet encoding for sending over the air
 *
 * Analog channels are reduced to 10 bits to allow for switch encoding
 * Switch[0] is sent on every packet.
 * A 3 bit switch index and 3-4 bit value is used to send the remaining switches
 * in a round-robin fashion.
 *
 * Inputs: crsf.ChannelDataIn, crsf.currentSwitches
 * Outputs: Radio.TXdataBuffer, side-effects the sentSwitch value
 */
// The next switch index to send, where 0=AUX2 and 6=AUX8
static uint8_t Hybrid8NextSwitchIndex;
#if defined(UNIT_TEST)
void OtaSetHybrid8NextSwitchIndex(uint8_t idx) { Hybrid8NextSwitchIndex = idx; }
#endif


void ICACHE_RAM_ATTR GenerateChannelDataHybrid8(volatile uint8_t* Buffer, CRSF *crsf, bool TelemetryStatus, uint8_t nonce, uint8_t tlmDenom)
{
    PackChannelDataHybridCommon(Buffer, crsf);

    // Actually send switchIndex - 1 in the packet, to shift down 1-7 (0b111) to 0-6 (0b110)
    // If the two high bits are 0b11, the receiver knows it is the last switch and can use
    // that bit to store data
    uint8_t bitclearedSwitchIndex = Hybrid8NextSwitchIndex;
    uint8_t value;
    // AUX8 is High Resolution 16-pos (4-bit)
    if (bitclearedSwitchIndex == 6)
        value = CRSF_to_N(crsf->ChannelDataIn[6 + 1 + 4], 16);
    else
    {
        // AUX2-7 are Low Resolution, "7pos" 6+center (3-bit)
        // The output is mapped evenly across 6 output values (0-5)
        // with a special value 7 indicating the middle so it works
        // with switches with a middle position as well as 6-position
        const uint16_t CHANNEL_BIN_COUNT = 6;
        const uint16_t CHANNEL_BIN_SIZE = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN) / CHANNEL_BIN_COUNT;
        uint16_t ch = crsf->ChannelDataIn[bitclearedSwitchIndex + 1 + 4];
        // If channel is within 1/4 a BIN of being in the middle use special value 7
        if (ch < (CRSF_CHANNEL_VALUE_MID-CHANNEL_BIN_SIZE/4)
            || ch > (CRSF_CHANNEL_VALUE_MID+CHANNEL_BIN_SIZE/4))
            value = CRSF_to_N(ch, CHANNEL_BIN_COUNT);
        else
            value = 7;
    } // If not 16-pos

    Buffer[6] =
        TelemetryStatus << 7 |
        // switch 0 is one bit sent on every packet - intended for low latency arm/disarm
        CRSF_to_BIT(crsf->ChannelDataIn[4]) << 6 |
        // tell the receiver which switch index this is
        bitclearedSwitchIndex << 3 |
        // include the switch value
        value;

    // update the sent value
    Hybrid8NextSwitchIndex = (bitclearedSwitchIndex + 1) % 7;
}

/**
 * Return the OTA value respresentation of the switch contained in ChannelDataIn
 * Switches 1-6 (AUX2-AUX7) are 6 or 7 bit depending on the lowRes parameter
 */
static uint8_t ICACHE_RAM_ATTR HybridWideSwitchToOta(CRSF *crsf, uint8_t switchIdx, bool lowRes)
{
    uint16_t ch = crsf->ChannelDataIn[switchIdx + 4];
    uint8_t binCount = (lowRes) ? 64 : 128;
    ch = CRSF_to_N(ch, binCount);
    if (lowRes)
        return ch & 0b111111; // 6-bit
    else
        return ch & 0b1111111; // 7-bit
}

/**
 * HybridWide switches packet encoding for sending over the air
 *
 * Analog channels are reduced to 10 bits to allow for switch encoding
 * Switch[0] is sent on every packet.
 * A 6 or 7 bit switch value is used to send the remaining switches
 * in a round-robin fashion.
 *
 * Inputs: crsf.ChannelDataIn, crsf.LinkStatistics.uplink_TX_Power
 * Outputs: Radio.TXdataBuffer
 **/
void ICACHE_RAM_ATTR GenerateChannelDataHybridWide(volatile uint8_t* Buffer, CRSF *crsf, bool TelemetryStatus, uint8_t nonce, uint8_t tlmDenom)
{
    PackChannelDataHybridCommon(Buffer, crsf);

    uint8_t telemBit = TelemetryStatus << 6;
    uint8_t nextSwitchIndex = HybridWideNonceToSwitchIndex(nonce);
    uint8_t value;
    // Using index 7 means the telemetry bit will always be sent in the packet
    // preceding the RX's telemetry slot for all tlmDenom >= 8
    // For more frequent telemetry rates, include the bit in every
    // packet and degrade the value to 6-bit
    // (technically we could squeeze 7-bits in for 2 channels with tlmDenom=4)
    if (nextSwitchIndex == 7)
    {
        value = telemBit | crsf->LinkStatistics.uplink_TX_Power;
    }
    else
    {
        bool telemInEveryPacket = (tlmDenom < 8);
        value = HybridWideSwitchToOta(crsf, nextSwitchIndex + 1, telemInEveryPacket);
        if (telemInEveryPacket)
            value |= telemBit;
    }

    Buffer[6] =
        // switch 0 is one bit sent on every packet - intended for low latency arm/disarm
        CRSF_to_BIT(crsf->ChannelDataIn[4]) << 7 |
        // include the switch value
        value;
}


/**
 * 16 Channels mode packet encoding for sending over the air
 *
 * Each frame contains 4 channels with 11 bits resolution so it is 44 bits
 * It remains 4 bits which become:
 *   1 bit fot Switch[0] ; is sent on every packet after the 44 bits.
 *   2 bits to identifies the set of channels in the frame  (0b00 = channels 1/4, 0b01= 5/8, ... 0b11 = 12/16
 *   1 bit for the telemetry status 
 * 
 * Inputs: crsf.ChannelDataIn , TelemetryStatus
 *     nonce and tlmDenom are still part of the parameters in this draft version but could be remove later on.
 * Outputs: Radio.TXdataBuffer
 **/
void ICACHE_RAM_ATTR GenerateChannelDataHybrid16(volatile uint8_t* Buffer, CRSF *crsf, bool TelemetryStatus, uint8_t nonce, uint8_t tlmDenom)
{
    static uint8_t Full16ChannelsIdx = 0 ;
    uint8_t Idx = Full16ChannelsIdx << 2;
    Buffer[0] = RC_DATA_PACKET & 0b11;
    Buffer[1] = (crsf->ChannelDataIn[Idx] >> 3);
    Buffer[2] = (((crsf->ChannelDataIn[Idx] & 0b00000111) << 5) | (crsf->ChannelDataIn[Idx + 1] >> 6) );
    Buffer[3] = (((crsf->ChannelDataIn[Idx + 1] & 0b00111111 ) << 2) | (crsf->ChannelDataIn[Idx + 2] >> 9) );
    Buffer[4] = (crsf->ChannelDataIn[Idx + 2] >> 1 );
    Buffer[5] = (((crsf->ChannelDataIn[Idx + 2] & 0b00000001 ) << 7) | (crsf->ChannelDataIn[Idx + 3] >> 4) );
    Buffer[6] = (((crsf->ChannelDataIn[Idx + 3] & 0b00001111 ) << 4) ) ; // first main bits contains remaining part of the last channel
    Buffer[6] = Buffer[6] | (CRSF_to_BIT(crsf->ChannelDataIn[4]) << 3) ; // keep switch 0;  is one bit sent on every packet - intended for low latency arm/disarm
    Buffer[6] = Buffer[6] | ((Full16ChannelsIdx & 0b11) << 1) ; // add 2 bits to identify the groups of channels in each frame
    Buffer[6] = Buffer[6] | (TelemetryStatus & 0b1) ; // add 1 bits to identify the groups of channels 
    
    Full16ChannelsIdx = (Full16ChannelsIdx + 1) & 0b11 ; // keep index in range 0...3 (because there are 4 groups)
}

/**
 * 10 Channels mode packet encoding for sending over the air
 * We send 2 frames with 6 bytes (+ header and CRC)
 * Each frame contains 4 channels with 10 bits resolution so it is 40 bits (channel 1/4 or channels 6/9)
 * It remains 1 byte in  the packet which becomes:
 *   5 bits for one low resolution channel (channel 10 or 11) 
 *   1 bit fot Switch[0] based on chanel 5 ; it is sent on every packet
 *   1 bits to identifies the set of channels in the frame  (0b0 = channels 1/4 + 10, 0b1= 6/9 + 11)
 *   1 bit for the telemetry status 
 * 
 * Inputs: crsf.ChannelDataIn , TelemetryStatus
 *     nonce and tlmDenom are still part of the parameters in this draft version but could be remove later on.
 * Outputs: Radio.TXdataBuffer
 **/
void ICACHE_RAM_ATTR GenerateChannelDataHybrid10(volatile uint8_t* Buffer, CRSF *crsf, bool TelemetryStatus, uint8_t nonce, uint8_t tlmDenom)
{
    static uint8_t Full10ChannelsIdx = 0 ;
    uint8_t Idx = ( Full10ChannelsIdx ) ? 5 : 0 ; // index = channel -1
    uint8_t IdxLowResolution = (Idx)  ? 10 : 9; // index = channel -1
    Buffer[0] = RC_DATA_PACKET & 0b11;
    Buffer[1] = (crsf->ChannelDataIn[Idx] >> 3);
    Buffer[2] = (((crsf->ChannelDataIn[Idx] & 0b00000110) << 5) | (crsf->ChannelDataIn[Idx + 1] >> 5) );
    Buffer[3] = (((crsf->ChannelDataIn[Idx + 1] & 0b00011110 ) << 3) | (crsf->ChannelDataIn[Idx + 2] >> 7) );
    Buffer[4] = (((crsf->ChannelDataIn[Idx + 2] & 0b01111110 ) << 1) | (crsf->ChannelDataIn[Idx + 3] >> 9) );
    Buffer[5] = (crsf->ChannelDataIn[Idx + 3] >> 1 );
    Buffer[6] = (crsf->ChannelDataIn[IdxLowResolution] >> 6 ) ; // first main bits contains 5 bits of channels
    Buffer[6] = Buffer[6] | (CRSF_to_BIT(crsf->ChannelDataIn[4]) << 1) ; // keep switch 0;  is one bit sent on every packet - intended for low latency arm/disarm
    Buffer[6] = Buffer[6] | ((Full10ChannelsIdx & 0b1) << 1) ; // add 1 bits to identify the groups of channels in each frame
    Buffer[6] = Buffer[6] | (TelemetryStatus & 0b1) ; // add 1 bits for telemetry acknowlegment 
    
    Full10ChannelsIdx = (Full10ChannelsIdx + 1) & 0b1 ; // keep index in range 0, 1 (because there are only 2 groups)
}


#endif // end of TARGET_TX or defined UNIT_TEST

#if TARGET_RX or defined UNIT_TEST

// Current ChannelData unpacker function being used by RX
UnpackChannelData_t UnpackChannelData;

static void ICACHE_RAM_ATTR UnpackChannelDataHybridCommon(volatile uint8_t* Buffer, CRSF *crsf)
{
    // The analog channels
    crsf->PackedRCdataOut.ch0 = (Buffer[1] << 3) | ((Buffer[5] & 0b11000000) >> 5);
    crsf->PackedRCdataOut.ch1 = (Buffer[2] << 3) | ((Buffer[5] & 0b00110000) >> 3);
    crsf->PackedRCdataOut.ch2 = (Buffer[3] << 3) | ((Buffer[5] & 0b00001100) >> 1);
    crsf->PackedRCdataOut.ch3 = (Buffer[4] << 3) | ((Buffer[5] & 0b00000011) << 1);
}

/**
 * Hybrid switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 *
 * Input: Buffer
 * Output: crsf->PackedRCdataOut
 * Returns: TelemetryStatus bit
 */
bool ICACHE_RAM_ATTR UnpackChannelDataHybridSwitch8(volatile uint8_t* Buffer, CRSF *crsf, uint8_t nonce, uint8_t tlmDenom)
{
    const uint8_t switchByte = Buffer[6];
    UnpackChannelDataHybridCommon(Buffer, crsf);

    // The low latency switch
    crsf->PackedRCdataOut.ch4 = BIT_to_CRSF((switchByte & 0b01000000) >> 6);

    // The round-robin switch, switchIndex is actually index-1
    // to leave the low bit open for switch 7 (sent as 0b11x)
    // where x is the high bit of switch 7
    uint8_t switchIndex = (switchByte & 0b111000) >> 3;
    uint16_t switchValue = SWITCH3b_to_CRSF(switchByte & 0b111);

    switch (switchIndex) {
    case 0:
        crsf->PackedRCdataOut.ch5 = switchValue;
        break;
    case 1:
        crsf->PackedRCdataOut.ch6 = switchValue;
        break;
    case 2:
        crsf->PackedRCdataOut.ch7 = switchValue;
        break;
    case 3:
        crsf->PackedRCdataOut.ch8 = switchValue;
        break;
    case 4:
        crsf->PackedRCdataOut.ch9 = switchValue;
        break;
    case 5:
        crsf->PackedRCdataOut.ch10 = switchValue;
        break;
    case 6:   // Because AUX1 (index 0) is the low latency switch, the low bit
    case 7:   // of the switchIndex can be used as data, and arrives as index "6"
        crsf->PackedRCdataOut.ch11 = N_to_CRSF(switchByte & 0b1111, 15);
        break;
    }

    // TelemetryStatus bit
    return switchByte & (1 << 7);
}

/**
 * HybridWide switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 1 bits for the low latency switch[0]
 * 6 or 7 bits for the round-robin switch
 * 1 bit for the TelemetryStatus, which may be in every packet or just idx 7
 * depending on TelemetryRatio
 *
 * Output: crsf.PackedRCdataOut, crsf.LinkStatistics.uplink_TX_Power
 * Returns: TelemetryStatus bit
 */

bool ICACHE_RAM_ATTR UnpackChannelDataHybridWide(volatile uint8_t* Buffer, CRSF *crsf, uint8_t nonce, uint8_t tlmDenom)
{
    static bool TelemetryStatus = false;
    const uint8_t switchByte = Buffer[6];
    UnpackChannelDataHybridCommon(Buffer, crsf);

    // The low latency switch (AUX1)
    crsf->PackedRCdataOut.ch4 = BIT_to_CRSF((switchByte & 0b10000000) >> 7);

    // The round-robin switch, 6-7 bits with the switch index implied by the nonce
    uint8_t switchIndex = HybridWideNonceToSwitchIndex(nonce);
    bool telemInEveryPacket = (tlmDenom < 8);
    if (telemInEveryPacket || switchIndex == 7)
          TelemetryStatus = (switchByte & 0b01000000) >> 6;
    if (switchIndex == 7)
    {
        crsf->LinkStatistics.uplink_TX_Power = switchByte & 0b111111;
    }
    else
    {
        uint8_t bins;
        uint16_t switchValue;
        if (telemInEveryPacket)
        {
            bins = 63;
            switchValue = switchByte & 0b111111; // 6-bit
        }
        else
        {
            bins = 127;
            switchValue = switchByte & 0b1111111; // 7-bit
        }

        switchValue = N_to_CRSF(switchValue, bins);
        switch (switchIndex) {
            case 0:
                crsf->PackedRCdataOut.ch5 = switchValue;
                break;
            case 1:
                crsf->PackedRCdataOut.ch6 = switchValue;
                break;
            case 2:
                crsf->PackedRCdataOut.ch7 = switchValue;
                break;
            case 3:
                crsf->PackedRCdataOut.ch8 = switchValue;
                break;
            case 4:
                crsf->PackedRCdataOut.ch9 = switchValue;
                break;
            case 5:
                crsf->PackedRCdataOut.ch10 = switchValue;
                break;
            case 6:
                crsf->PackedRCdataOut.ch11 = switchValue;
                break;
        }
    }

    return TelemetryStatus;
}

/******* this is a first draft to unpack 16 channels in 11 bits resolution
 * Unpacking 16 Channels 
 *
 * Each frame OTA contains only 4 channels with 11 bits resolution so it is 44 bits
 * 4 remaining bits are :
 *   1 bit fot Switch[0] ; is sent on every packet after the 44 bits.
 *   2 bits to identifies the set of channels in the frame  (0b00 = channels 1/4, 0b01= 5/8, ... 0b11 = 12/16
 *   1 bit for the telemetry status 
 * 
 * Inputs: crsf.ChannelDataIn , TelemetryStatus
 *     nonce and tlmDenom are still part of the parameters in this draft version but could be remove later on.
 * Outputs: Radio.TXdataBuffer ; the function returns the telemetry bit 
 **/
bool ICACHE_RAM_ATTR UnpackChannelDataHybrid16(volatile uint8_t* Buffer, CRSF *crsf, uint8_t nonce, uint8_t tlmDenom)
{
    static bool TelemetryStatus = Buffer[6] & 0b1; // last bit is always the telemetry acknowledgment 
    const uint8_t ChannelGroup = (Buffer[6] & 0b00000110 ) >> 1; // 2 bits identify the group of channels
    
    const uint16_t Channel1of4 = ( ((uint16_t) Buffer[1]) << 3) | ((Buffer[2] & 0b11100000) >> 5);
    const uint16_t Channel2of4 = ((((uint16_t) Buffer[2]) & 0b00011111 ) << 6) | ((Buffer[3] & 0b11111100) >> 2);
    const uint16_t Channel3of4 = ((((uint16_t) Buffer[3]) & 0b00000011 ) << 9) | (((uint16_t) Buffer[4]) << 1) | ((Buffer[5] & 0b10000000) >> 7);
    const uint16_t Channel4of4 = ((((uint16_t) Buffer[5]) & 0b01111111 ) << 4) | ((Buffer[6] & 0b11110000) >> 4) ;
    
    switch (ChannelGroup) {
        case 0:
            crsf->PackedRCdataOut.ch0 = Channel1of4 ;
            crsf->PackedRCdataOut.ch1 = Channel2of4 ;
            crsf->PackedRCdataOut.ch2 = Channel3of4 ;
            crsf->PackedRCdataOut.ch3 = Channel4of4 ;
            break ;
        case 1:
            crsf->PackedRCdataOut.ch4 = Channel1of4 ;
            crsf->PackedRCdataOut.ch5 = Channel2of4 ;
            crsf->PackedRCdataOut.ch6 = Channel3of4 ;
            crsf->PackedRCdataOut.ch7 = Channel4of4 ;
            break ;
        case 2:
            crsf->PackedRCdataOut.ch8 = Channel1of4 ;
            crsf->PackedRCdataOut.ch9 = Channel2of4 ;
            crsf->PackedRCdataOut.ch10 = Channel3of4 ;
            crsf->PackedRCdataOut.ch11 = Channel4of4 ;
            break ;
        case 3:
            crsf->PackedRCdataOut.ch12 = Channel1of4 ;
            crsf->PackedRCdataOut.ch13 = Channel2of4 ;
            crsf->PackedRCdataOut.ch14 = Channel3of4 ;
            crsf->PackedRCdataOut.ch15 = Channel4of4 ;
            break ;
    }    
    // The low latency switch (AUX1) is present in each packet (for compatibility reason with existing code - at least at this stage)
    crsf->PackedRCdataOut.ch4 = BIT_to_CRSF((Buffer[6] & 0b1000) >> 3);
    return TelemetryStatus;
}

/**
 * 10 Channels mode packet encoding for sending over the air
 * We send 2 frames with 6 bytes (+ header and CRC)
 * Each frame contains 4 channels with 10 bits resolution so it is 40 bits (channel 1/4 or channels 6/9)
 * It remains 1 byte in  the packet which becomes:
 *   5 bits for one low resolution channel (channel 10 or 11) 
 *   1 bit fot Switch[0] based on chanel 5 ; it is sent on every packet
 *   1 bits to identifies the set of channels in the frame  (0b0 = channels 1/4 + 10, 0b1= 6/9 + 11)
 *   1 bit for the telemetry status 
 * 
 * Inputs: crsf.ChannelDataIn , TelemetryStatus
 *     nonce and tlmDenom are still part of the parameters in this draft version but could be remove later on.
 * Outputs: Radio.TXdataBuffer
 **/
bool ICACHE_RAM_ATTR UnpackChannelDataHybrid10(volatile uint8_t* Buffer, CRSF *crsf, uint8_t nonce, uint8_t tlmDenom)
{
    static bool TelemetryStatus = Buffer[6] & 0b1; // last bit is always the telemetry acknowledgment 
    const uint8_t ChannelGroup = (Buffer[6] & 0b00000010 ) >> 1; // 1 bit identifies the group of channels
    // The low latency switch (AUX1) is present in each packet (for compatibility reason with existing code - at least at this stage)
    crsf->PackedRCdataOut.ch4 = BIT_to_CRSF((Buffer[6] & 0b100) >> 2);
    
    const uint16_t Channel1of6 = ( ((uint16_t) Buffer[1]) << 3) | ((Buffer[2] & 0b11000000) >> 5);
    const uint16_t Channel2of6 = ((((uint16_t) Buffer[2]) & 0b00111111 ) << 2) | ((Buffer[3] & 0b11110000) >> 3);
    const uint16_t Channel3of6 = ((((uint16_t) Buffer[3]) & 0b00001111 ) << 7) | ((Buffer[4] & 0b11111100) >> 1);
    const uint16_t Channel4of6 = ((((uint16_t) Buffer[4]) & 0b00000011 ) << 9) | ((Buffer[5] & 0b11111111) << 1) ;
    const uint16_t Channel6of6 = ((((uint16_t) Buffer[6]) & 0b11111000 ) << 5)  ;
    
    switch (ChannelGroup) {
        case 0:
            crsf->PackedRCdataOut.ch0 = Channel1of6 ;
            crsf->PackedRCdataOut.ch1 = Channel2of6 ;
            crsf->PackedRCdataOut.ch2 = Channel3of6 ;
            crsf->PackedRCdataOut.ch3 = Channel4of6 ;
            crsf->PackedRCdataOut.ch9 = Channel6of6 ;
            break ;
        case 1:
            crsf->PackedRCdataOut.ch5 = Channel1of6 ;
            crsf->PackedRCdataOut.ch6 = Channel2of6 ;
            crsf->PackedRCdataOut.ch7 = Channel3of6 ;
            crsf->PackedRCdataOut.ch8 = Channel4of6 ;
            crsf->PackedRCdataOut.ch10 = Channel6of6 ;
            break ;
    }    
    return TelemetryStatus;
}


#endif  // end of TARGET_RX or defined UNIT_TEST

OtaSwitchMode_e OtaSwitchModeCurrent;
void OtaSetSwitchMode(OtaSwitchMode_e switchMode)
{
    switch(switchMode)
    {
    case sm1Bit:
    case smHybrid:
    default:
        #if defined(TARGET_TX) || defined(UNIT_TEST)
            #if (!defined USE_HYBRID_MODE_FOR_10_CHANNELS)
        PackChannelData = &GenerateChannelDataHybrid8;
            #else
        PackChannelData = &GenerateChannelDataHybrid16;
            #endif
        #endif
        #if defined(TARGET_RX) || defined(UNIT_TEST)
            #if (!defined USE_HYBRID_MODE_FOR_10_CHANNELS)
        UnpackChannelData = &UnpackChannelDataHybridSwitch8;
            #else
        UnpackChannelData = &UnpackChannelDataHybrid16;
            #endif
        #endif
        break;
    case smHybridWide:
        #if defined(TARGET_TX) || defined(UNIT_TEST)
            #if (!defined USE_HYBRIDWIDE_MODE_FOR_10_CHANNELS)
        PackChannelData = &GenerateChannelDataHybridWide;
            #else
        PackChannelData = &GenerateChannelDataHybrid16;
            #endif
        #endif
        #if defined(TARGET_RX) || defined(UNIT_TEST)
            #if (!defined USE_HYBRIDWIDE_MODE_FOR_10_CHANNELS)
        UnpackChannelData = &UnpackChannelDataHybridWide;
            #else
        UnpackChannelData = &UnpackChannelDataHybrid16;
            #endif    
        #endif
        break;
    }

    OtaSwitchModeCurrent = switchMode;
}