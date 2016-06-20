/*
 * mac.h
 *
 * Created: 4/1/2016 4:27:10 PM
 *  Author: psammand
 */
#ifndef MAC_H_
#define MAC_H_

#define aBaseSlotDuration           60
#define aBaseSuperframeDuration     aBaseSlotDuration * aNumSuperframeSlots
#define aGTSDescPersistenceTime     4
#define aMaxBeaconOverhead          75
#define aMaxBeaconPayloadLength     aMaxPHYPacketSize–aMaxBeaconOverhead
#define aMaxLostBeacons             4
#define aMaxMACSafePayloadSize      aMaxPHYPacketSize–aMaxMPDUUnsecuredOverhead
#define aMaxMACPayloadSize          aMaxPHYPacketSize–aMinMPDUOverhead
#define aMaxMPDUUnsecuredOverhead   25
#define aMaxSIFSFrameSize           18
#define aMinCAPLength               440
#define aMinMPDUOverhead            9
#define aNumSuperframeSlots         16
#define aUnitBackoffPeriod          20

typedef struct
{
    uint64_t    macExtendedAddress;
    uint32_t    macAckWaitDuration;
    bool        macAssociatedPANCoord;
    bool        macAssociationPermit;
    bool        macAutoRequest;
    bool        macBattLifeExt;
    uint32_t    macBattLifeExtPeriods;
    uint8_t*    macBeaconPayload;
    uint32_t    macBeaconPayloadLength;
    uint8_t     macBeaconOrder;
    uint32_t    macBeaconTxTime;
    uint8_t     macBSN;
    uint64_t    macCoordExtendedAddress;
    uint16_t    macCoordShortAddress;
    uint8_t     macDSN;
    bool        macGTSPermit;
    uint8_t     macMaxBE;
    uint8_t     macMaxCSMABackoffs;
    uint32_t    macMaxFrameTotalWaitTime;
    uint8_t     macMaxFrameRetries;
    uint8_t     macMinBE;
    uint16_t    macPANId;
    bool        macPromiscuousMode;
    uint8_t     macResponseWaitTime;
    bool        macRxOnWhenIdle;
    bool        macSecurityEnabled;
    uint16_t    macShortAddress;
    uint8_t     macSuperframeOrder;

} mac_pib_t;

typedef struct
{
    uint8_t mode;
    union
    {
        uint16_t    short_addr;
        uint64_t    ext_addr;
    };
} addr_mode_t;

typedef union
{
    uint16_t    short_addr;
    uint64_t    ext_addr;
} device_addr_t;

typedef struct
{
    device_addr_t uint8_t   channel_no;
    uint8_t                 channel_page;
    uint8_t                 coord_addr_mode;
    uint16_t                coord_pan_id;
    uint8_t                 capability_info;
    uint8_t                 security_level;
} mlme_associate_req_t;

typedef struct
{
    device_addr_t   device_addr;
    uint8_t         capability_info;
    uint8_t         security_level;
} mlme_associate_ind_t;

typedef struct
{
    device_addr_t   device_addr;
    uint16_t        assoc_short_addr;
    uint8_t         status;
    uint8_t         security_level;
} mlme_associate_res_t;

typedef struct
{
    uint16_t        assoc_short_addr;
    uint8_t         status;
    uint8_t         security_level;
} mlme_associate_confirm_t;

void    mlme_associate_req(void);
void    mlme_disassociate_req(void);
void    mlme_beacon_notify(void);
void    mlme_comm_staus(void);
void    mlme_get(void);
void    mlme_orphan(void);
void    mlme_reset(void);
void    mlme_rx_enable(void);
void    mlme_scan(void);
void    mlme_set(void);
void    mlme_start(void);

#endif /* MAC_H_ */
