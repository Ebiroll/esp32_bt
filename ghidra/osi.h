


typedef unsigned int uint;
typedef unsigned char byte;

struct ghidra_ip_t {
    void (*r_bt_util_buf_init)(void);                               /* 0x00 */
    int* (*r_bt_util_buf_lmp_tx_alloc)(void);                      /* 0x04 */
    uint (*r_bt_util_buf_lmp_tx_free)(uint *buff);                 /* 0x08 */
    int* (*r_bt_util_buf_acl_rx_alloc)(void);                      /* 0x0c */
    uint (*r_bt_util_buf_acl_rx_free)(uint bufix);                 /* 0x10 */
    int* (*r_bt_util_buf_acl_tx_alloc)(void);                      /* 0x14 */
    uint (*r_bt_util_buf_acl_tx_free)(uint buffix);                /* 0x18 */
    uint* (*r_bt_util_buf_sync_init)(uint p1, uint p2, uint p3, uint p4, uint p5); /* 0x1c */
    int* (*r_bt_util_buf_sync_clear)(uint param_1);                /* 0x20 */
    int** (*r_bt_util_buf_sync_tx_alloc)(byte ix, byte param_2);   /* 0x24 */
    uint (*r_bt_util_buf_sync_tx_free)(uint p1, uint p2);          /* 0x28 */
    int* (*r_bt_util_buf_sync_rx_alloc)(uint p1, byte p2);         /* 0x2c */
    uint* (*r_bt_util_buf_sync_rx_free)(uint p1, uint p2);         /* 0x30 */
    
    // Crypto / Security Functions
    unsigned char* (*r_E1)(uint p1, uint p2, uint p3, uint p4, uint p5, uint p6); /* 0x34 */
    uint* (*r_E21)(uint p1, uint p2, uint p3, uint p4, uint p5, uint p6, char *p7); /* 0x38 */
    uint* (*r_E22)(uint p1, uint p2, uint p3, uint p4, uint p5, uint p6); /* 0x3c */
    uint (*r_KPrimC)(uint p1, uint p2, uint p3, uint p4, uint p5, uint *p6); /* 0x40 */
    uint (*r_XorKey)(uint p1, uint p2, uint p3, uint p4);          /* 0x44 */
        
    /* LM (Link Manager) Utility Functions */
    int (*r_LM_MakeRandVec)(int param_1);
    uint (*r_lmp_pack)(uint *param_1, char *param_2);
    uint (*r_lmp_unpack)(uint *param_1, byte *param_2, byte *param_3);
    uint (*r_lm_n_is_zero)(uint param_1);  
    uint* (*r_lm_sp_n_one)(void *param_1, int param_2);
    
    /* SHA-256 and HMAC (Used for Secure Simple Pairing) */
    byte* (*r_lm_sp_sha256_calculate)(byte *param_1, uint *param_2, unsigned short param_3);
    uint (*r_lm_sp_n192_convert_wnaf)(int param_1, int param_2, uint *param_3);
    uint (*r_lm_sp_p192_point_to_inf)(int param_1);
    void (*r_lm_sp_p192_point_jacobian_to_affine)(int *param_1);
    void (*r_lm_sp_p192_points_jacobian_to_affine)(int param_1);
    void (*r_lm_sp_pre_compute_points)(uint *param_1);
    
    /* Elliptic Curve P-192 Math (The heart of BT Pairing) */
    int (*r_lm_sp_p192_dbl)(int *param_1, uint *param_2);
    uint (*r_lm_sp_p192_add)(int *param_1, uint *param_2, uint *param_3);
    uint (*r_lm_sp_p192_invert)(int param_1);
    
    /* F-series Functions (Key derivation for BLE/BR-EDR) */
    uint* (*r_lm_f1)(int param_1, byte *param_2, uint param_3, uint param_4, int param_5);
    uint* (*r_lm_f2)(void *p1, void *p2, void *p3, void *p4, int p5, byte *p6);
    uint (*r_lm_g)(void *p1, void *p2, char p3, int p4, uint *p5); 
    uint* (*r_lm_f3)(void *p1, void *p2, void *p3, uint p4, void *p5, void *p6, int p7, byte *p8);
    
    /* DHKey (Diffie-Hellman) calculation */
    void* (*r_lm_get_nonce)(void *param_1);
    uint (*r_lm_dhkey_calc_init)(int param_1); 
    void (*r_lm_dhkey_compare)(int param_1);  
    
    /* SHA-256 / HMAC / SHA Utility */
    byte* (*r_F1_256)(int p1, int p2, int p3, byte *p4, int p5);
    uint* (*r_HMAC)(int p1, int p2, int p3, uint p4);
    int (*r_G_256)(int p1, int p2, int p3, int p4, uint *p5); 
    uint (*r_SHA_256)(int p1, uint p2, byte *p3);
};

typedef struct {
    uint16_t txctrl;       /* 0x00: Bit 15 = Ready */
    uint16_t bt_header;    /* 0x02 */
    uint16_t acl_header;   /* 0x04 */
    uint16_t txdataptr;    /* 0x06 */
    uint16_t mic;          /* 0x08 */
    uint16_t txrate;       /* 0x0a */
    uint16_t txstat;       /* 0x0c */
    uint16_t txheaderptr;  /* 0x0e: Pointer to separate header buffer (if used) */
    uint32_t timestamp;    /* 0x10: Precise TX time scheduled */
} em_bt_txdesc_20_t;



#if 0

/* Event Arbiter (Real-time Scheduling) */
    uint* (*r_ea_elt_cancel)(int **param_1);
    uint (*r_ea_time_get_slot_rounded)(void);
    uint (*r_ea_init)(void);
    unsigned short* (*r_ea_elt_create)(uint param_1);
    uint (*r_ea_elt_insert)(uint *param_1);
    uint (*r_ea_time_get_halfslot_rounded)(void);
    uint (*r_ea_elt_remove)(int *param_1);
    unsigned short* (*r_ea_interval_create)(void);
    uint* (*r_ea_interval_insert)(uint *param_1);
    int* (*r_ea_interval_remove)(int *param_1);
    int* (*r_ea_interval_delete)(int *param_1);
    int (*r_ea_finetimer_isr)(void);
    int* (*r_ea_sw_isr)(void);
    uint (*r_ea_offset_req)(int p1, unsigned short *p2);
    uint (*r_ea_sleep_check)(uint *p1, uint p2); 
    uint (*r_ea_interval_duration_req)(unsigned short *p1, unsigned short *p2); 
    uint* (*r_ea_alarm_set)(uint *param_1);
    uint (*r_ea_alarm_clear)(int *param_1);
    
    /* Buffer / EM Init */
    uint (*r_em_buf_init)(void);
    uint (*r_em_buf_rx_free)(uint param_1);

#endif

struct ghidra_modules_t {
  void *co_list_init;
  void *co_list_pool_init;
  void *co_list_push_back;
  void *co_list_push_front;  
  void *co_list_pop_front;
  void *co_list_extract;    
  void *co_list_extract_after;
};

struct co_list_hdr {
    struct co_list_hdr *next;
};

struct co_list {
    struct co_list_hdr *first;
    struct co_list_hdr *last;
    uint32_t cnt;
    uint32_t max_cnt;
    uint32_t min_free;
};

typedef struct {
    uint32_t    _version;
    uint32_t(*_set_isr)(int n, xt_handler f, void *arg);
    void (*_ints_on)(unsigned int mask);
    void (*_interrupt_disable)(void);
    void (*_interrupt_restore)(void);
    void (*_task_yield)(void);
    void (*_task_yield_from_isr)(void);
    void *(*_semphr_create)(uint32_t max, uint32_t init);
    void (*_semphr_delete)(void *semphr);
    int32_t (*_semphr_take_from_isr)(void *semphr, void *hptw);
    int32_t (*_semphr_give_from_isr)(void *semphr, void *hptw);
    int32_t (*_semphr_take)(void *semphr, uint32_t block_time_ms);
    int32_t (*_semphr_give)(void *semphr);
    void *(*_mutex_create)(void);
    void (*_mutex_delete)(void *mutex);
    int32_t (*_mutex_lock)(void *mutex);
    int32_t (*_mutex_unlock)(void *mutex);
    void *(* _queue_create)(uint32_t queue_len, uint32_t item_size);
    void (* _queue_delete)(void *queue);
    int32_t (* _queue_send)(void *queue, void *item, uint32_t block_time_ms);
    int32_t (* _queue_send_from_isr)(void *queue, void *item, void *hptw);
    int32_t (* _queue_recv)(void *queue, void *item, uint32_t block_time_ms);
    int32_t (* _queue_recv_from_isr)(void *queue, void *item, void *hptw);
    int32_t (* _task_create)(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id);
    void (* _task_delete)(void *task_handle);
    bool (* _is_in_isr)(void);
    int (* _cause_sw_intr_to_core)(int core_id, int intr_no);
    void *(* _malloc)(uint32_t size);
    void *(* _malloc_internal)(uint32_t size);
    void (* _free)(void *p);
    int32_t (* _read_efuse_mac)(uint8_t mac[6]);
    void (* _srand)(unsigned int seed);
    int (* _rand)(void);
    uint32_t (* _btdm_lpcycles_2_us)(uint32_t cycles);
    uint32_t (* _btdm_us_2_lpcycles)(uint32_t us);
    bool (* _btdm_sleep_check_duration)(uint32_t *slot_cnt);
    void (* _btdm_sleep_enter_phase1)(uint32_t lpcycles);  /* called when interrupt is disabled */
    void (* _btdm_sleep_enter_phase2)(void);
    void (* _btdm_sleep_exit_phase1)(void);  /* called from ISR */
    void (* _btdm_sleep_exit_phase2)(void);  /* called from ISR */
    void (* _btdm_sleep_exit_phase3)(void);  /* called from task */
    bool (* _coex_bt_wakeup_request)(void);
    void (* _coex_bt_wakeup_request_end)(void);
    int (* _coex_bt_request)(uint32_t event, uint32_t latency, uint32_t duration);
    int (* _coex_bt_release)(uint32_t event);
    int (* _coex_register_bt_cb)(coex_func_cb_t cb);
    uint32_t (* _coex_bb_reset_lock)(void);
    void (* _coex_bb_reset_unlock)(uint32_t restore);
    uint32_t _magic;
} ghidra_osi_funcs_t;


#if 0
    void *sleep_exit_p3;       /* 39 */
    void *bt_wakeup_req;       /* 40 */
    void *bt_wakeup_end;       /* 41 */
    void *coex_bt_req;         /* 42 */
    void *coex_bt_rel;         /* 43 */
    void *coex_reg_bt_cb;      /* 44 */
    void *coex_bb_lock;        /* 45 */
    void *coex_bb_unlock;      /* 46 */
    void *coex_schm_reg;       /* 47 */
    void *coex_stat_clr;       /* 48 */
    void *coex_stat_set;       /* 49 */
    void *coex_intv_get;       /* 50 */
    void *coex_period_get;     /* 51 */
    void *coex_phase_get;      /* 52 */
    void *wifi_chan_get;       /* 53 */
    void *wifi_chan_cb_reg;    /* 54 */
    void *set_int_handler;     /* 55 */
    void *int_l3_disable;      /* 56 */
    void *int_l3_restore;      /* 57 */
    void *cust_queue_create;   /* 58 */
    void *coex_version_get;    /* 59 */
    void *patch_apply;         /* 60 */
#endif
