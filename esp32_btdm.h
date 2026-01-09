#ifndef ESP32_BTDM_H
#define ESP32_BTDM_H

#include <stdint.h>

#define BTDM_BASE_ADDR 0x3FF71000
#define EM_BASE_ADDR   0x3FFB0000

/* --- Exchange Memory Structures --- */

struct em_buf_tx_desc {
    uint16_t txptr;      // Pointer to the actual data buffer in EM
    uint16_t txheader;   // BLE Packet Header (Type, Length)
    uint16_t txdataptr;  // Data plane offset
    uint16_t txdle;      // Data Length Extension control
};

struct co_list_hdr {
    uint32_t next;       // Pointer to next element in list
};

struct em_desc_node {
    struct co_list_hdr hdr;
    uint16_t idx;
    uint16_t buffer_idx;
    uint16_t buffer_ptr;
    uint8_t  llid;
    uint8_t  length;
};

/* Interrupt Acknowledgment Register Bit Positions */
#define CSCNTINTACK_POS         (0U)
#define RXINTACK_POS            (1U)
#define SLPINTACK_POS           (2U)
#define EVENTINTACK_POS         (3U)
#define CRYPTINTACK_POS         (4U)
#define ERRORINTACK_POS         (5U)
#define GROSSTGTIMINTACK_POS    (6U)
#define FINETGTIMINTACK_POS     (7U)
#define EVENTAPFAINTACK_POS     (8U)

/* Interrupt Acknowledgment Register Bit Masks */
#define CSCNTINTACK_MASK        (1U << CSCNTINTACK_POS)     /* 625μs base time reference interrupt ack */
#define RXINTACK_MASK           (1U << RXINTACK_POS)        /* Packet RX interrupt ack */
#define SLPINTACK_MASK          (1U << SLPINTACK_POS)       /* Sleep mode interrupt ack */
#define EVENTINTACK_MASK        (1U << EVENTINTACK_POS)     /* End of event interrupt ack */
#define CRYPTINTACK_MASK        (1U << CRYPTINTACK_POS)     /* Encryption engine interrupt ack */
#define ERRORINTACK_MASK        (1U << ERRORINTACK_POS)     /* Error interrupt ack */
#define GROSSTGTIMINTACK_MASK   (1U << GROSSTGTIMINTACK_POS)/* Gross target timer interrupt ack */
#define FINETGTIMINTACK_MASK    (1U << FINETGTIMINTACK_POS) /* Fine target timer interrupt ack */
#define EVENTAPFAINTACK_MASK    (1U << EVENTAPFAINTACK_POS) /* End of event/anticipated pre-fetch abort interrupt ack */

/**
 * @brief BTDM Register Layout (ESP32 + generic mapping)
 */
typedef struct {
    /* --- BR/EDR (Classic) Section --- */
    volatile uint32_t BTCNTL;           // 0x000: BR/EDR control
    
    union {
        struct {
            uint32_t BUILD : 8;
            uint32_t UPG   : 8;
            uint32_t REL   : 8;
            uint32_t TYP   : 8;
        };
        volatile uint32_t val;
    } BTVERSION;                        // 0x004: BR/EDR version

    uint32_t _reserved0[1];
    volatile uint32_t BTINTCNTL;        // 0x00C
    volatile uint32_t BTINTSTAT;        // 0x010
    volatile uint32_t BTINTRAWSTAT;     // 0x014
    volatile uint32_t BTINTACK;         // 0x018

    uint32_t _reserved1[121];           // Offset to BLE block 

    /* --- BLE Section (Core) --- */
    
    // BB_RWBBCNTL
    union {
        struct {
            uint32_t SYNCERR           : 3;
            uint32_t _res0             : 1;
            uint32_t RXWINSZDEF        : 4;
            uint32_t RWBLE_EN          : 1;  // RW-BLE Core Exchange Table pre-fetch
            uint32_t ADVERTFILT_EN     : 1;
            uint32_t _res1             : 6;
            uint32_t HOP_REMAP_DSB     : 1;
            uint32_t CRC_DSB           : 1;
            uint32_t WHIT_DSB          : 1;
            uint32_t CRYPT_DSB         : 1;
            uint32_t NESN_DSB          : 1;
            uint32_t SN_DSB            : 1;
            uint32_t MD_DSB            : 1;
            uint32_t _res2             : 1;
            uint32_t SCAN_ABORT        : 1;
            uint32_t ADVERT_ABORT      : 1;
            uint32_t RFTEST_ABORT      : 1;
            uint32_t _res3             : 1;
            uint32_t SWINT_REQ         : 1;
            uint32_t REG_SOFT_RST      : 1;
            uint32_t MASTER_TGSOFT_RST : 1;
            uint32_t MASTER_SOFT_RST   : 1;  // RW-BLE Master reset
        };
        volatile uint32_t val;
    } BLECNTL;                          // 0x200 (RW Offset 0x00)

    // BB_VERSION
    union {
        struct {
            uint32_t BUILD : 8;
            uint32_t UPG   : 8;
            uint32_t REL   : 8;
            uint32_t TYP   : 8;         // BLE Core Type
        };
        volatile uint32_t val;
    } BLEVERSION;                       // 0x204 (RW Offset 0x04)

    // BB_RWBLEBCONF
    volatile uint32_t BLECONF;          // 0x208 (RW Offset 0x08)

    // BB_INTCNTL
    union {
        struct {
            uint32_t CSCNTINTMSK      : 1; // 625μs Base Time
            uint32_t RXINTMSK         : 1;
            uint32_t SLPINTMSK        : 1;
            uint32_t EVENTINTMSK      : 1; // End of Event (Adv/Scan/Conn)
            uint32_t CRYPTINTMSK      : 1;
            uint32_t ERRORINTMSK      : 1;
            uint32_t GROSSTGTIMINTMSK : 1;
            uint32_t FINETGTIMINTMSK  : 1;
            uint32_t EVENTAPFAINTMSK  : 1;
            uint32_t SWINTMSK         : 1;
            uint32_t _res             : 22;
        };
        volatile uint32_t val;
    } BLEINTCNTL;                       // 0x20C (RW Offset 0x0C)

    volatile uint32_t BLEINTSTAT;       // 0x210 (RW Offset 0x10)
    volatile uint32_t BLEINTRAWSTAT;    // 0x214 same bits as BLEINTCNTL
    volatile uint32_t BLEINTACK;        // 0x218 same bits as BLEINTCNTL

    // BB_BASETIMECNT
    union {
        struct {
            uint32_t BASETIMECNT : 27;
            uint32_t _res        : 4;
            uint32_t SAMP        : 1;
        };
        volatile uint32_t val;
    } BLEBASETIMECNT;                   // 0x21C (RW Offset 0x1C)

    // BB_FINETIMECNT
    union {
        struct {
            uint32_t FINECNT : 10;
            uint32_t _res    : 22;
        };
        volatile uint32_t val;
    } BLEFINETIMECNT;                   // 0x220 (RW Offset 0x20)

    volatile uint32_t BLEBDADDRL;       // 0x224 (RW Offset 0x24)

    // BB_BDADDRU
    union {
        struct {
            uint32_t BDADDRU  : 16;
            uint32_t PRIV_NPUB : 1;     // public = 0
            uint32_t _res     : 15;
        };
        volatile uint32_t val;
    } BLEBDADDRU;                       // 0x228 (RW Offset 0x28)

    // BB_ET_CURRENTRXDESCPTR
    union {
        struct {
            uint32_t CURRENTRXDESCPTR : 15;
            uint32_t _res             : 1;
            uint32_t ETPTR            : 15; // Pointer to Exchange Table in EM
            uint32_t _res2            : 1;
        };
        volatile uint32_t val;
    } BLECURRENTRXDESCPTR;              // 0x22C 

    uint32_t _reserved2[8];

    volatile uint32_t BLEDIAGCNTL;      // 0x250 (BB_DIAGCNTL)
    volatile uint32_t BLEDIAGSTAT;      // 0x254
    
    uint32_t _reserved3[2];
    volatile uint32_t BLEERRORTYPESTAT; // 0x260

    uint32_t _reserved4[3];
    volatile uint32_t BLERADIOCNTL0;    // 0x270 (BB_RADIOCNTL0)
    volatile uint32_t BLERADIOCNTL1;    // 0x274 (BB_RADIOCNTL1)

    uint32_t _reserved5[2];
    volatile uint32_t BLERADIOPWRUPDN;  // 0x280 (BB_RADIOPWRUPDN)

    uint32_t _reserved6[3];
    // BB_ADVCHMAP
    union {
        struct {
            uint32_t CHAN37 : 1;
            uint32_t CHAN38 : 1;
            uint32_t CHAN39 : 1;
            uint32_t _res   : 29;
        };
        volatile uint32_t val;
    } BLEADVCHMAP;                      // 0x290 (RW Offset 0xB0 shifted)

    uint32_t _reserved7[3];
    volatile uint32_t BLEADVTIM;        // 0x2A0 (BB_ADVTIM)

    uint32_t _reserved8[3];
    volatile uint32_t BLEWLPUBADDRPTR;  // 0x2B0 (BB_WLPUBADDPTR)
    volatile uint32_t BLEWLPRIVADDRPTR; // 0x2B4
    volatile uint32_t BLEWLNBDEV;       // 0x2B8

    uint32_t _reserved9[1];
    // BB_AESCNTL
    union {
        struct {
            uint32_t AES_START : 1;
            uint32_t AES_MODE  : 1;
            uint32_t _res      : 30;
        };
        volatile uint32_t val;
    } BLEAESCNTL;                       // 0x2C0 (RW Offset 0xE0 shifted)

    volatile uint32_t BLEAESKEY[4];     // 0x2C4 - 0x2D0
    volatile uint32_t BLEAESPTR;        // 0x2D4 (BB_AESPTR)

    uint32_t _reserved10[2];
    // BB_RFTESTCNTL
    union {
        struct {
            uint32_t TXLENGTH    : 9;
            uint32_t _res0       : 2;
            uint32_t TXPKTCNTEN  : 1;
            uint32_t TXPLDSRC    : 1;
            uint32_t PRBSTYPE    : 1;
            uint32_t TXLENGTHSRC : 1;
            uint32_t INFINITETX  : 1;
            uint32_t _res1       : 11;
            uint32_t RXPKTCNTEN  : 1;
            uint32_t _res2       : 3;
            uint32_t INFINITERX  : 1;
        };
        volatile uint32_t val;
    } BLERFTESTCNTL;                    // 0x2E0 (RW Offset 0x100 shifted)

    volatile uint32_t BLERFTESTTXSTAT;  // 0x2E4
    volatile uint32_t BLERFTESTRXSTAT;  // 0x2E8

    uint32_t _reserved11[1];
    // BB_TIMGENCNTL
    union {
        struct {
            uint32_t PREFETCH_TIME      : 9;
            uint32_t _res0              : 7;
            uint32_t PREFETCHABORT_TIME : 10;
            uint32_t _res1              : 5;
            uint32_t APFM_EN            : 1;
        };
        volatile uint32_t val;
    } BLETIMGENCNTL;                    // 0x2F0 (RW Offset 0x110 shifted)

    uint32_t _reserved12[3];
    volatile uint32_t BLECOEXIFCNTL0;   // 0x300 (BB_COEXIFCNTL0)

    uint32_t _reserved13[7];
    volatile uint32_t BLERALPTR;        // 0x320 (BB_RALPTR) Resolve Address List pointer
    volatile uint32_t BLERALNBDEV;      // 0x324

} btdm_dev_t;

/* --- Peripheral and Memory Access --- */

#define BTDM_PTR ((void*)(BTDM_BASE_ADDR))

// Exchange Memory Pointers
#define EM_EXCH_TABLE       ((uint16_t*)(EM_BASE_ADDR + 0x0000))
#define EM_FREQ_TABLE       ((uint16_t*)(EM_BASE_ADDR + 0x0040))
#define EM_CONTROL_STRUCT   ((uint8_t*)(EM_BASE_ADDR + 0x00B8))
#define EM_TX_DESC_BASE     ((struct em_buf_tx_desc*)(EM_BASE_ADDR + 0x05AC))
#define EM_RX_DESC_BASE     ((struct em_buf_tx_desc*)(EM_BASE_ADDR + 0x0934))
#define EM_TX_DATA_BUF      ((uint8_t*)(EM_BASE_ADDR + 0x0B82))

#endif // ESP32_BTDM_H