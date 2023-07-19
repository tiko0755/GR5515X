#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define KE_IDX_GET(ke_task_id) (((ke_task_id) >> 8) & 0xFF)
#define TASK_NONE  0xFF
#define RW_ERR_HCI_TO_HL(err)     (((err) != 0) ? ((err) + 0x90) : (0))
#define GAP_BD_ADDR_LEN         (6)
#define GAP_KEY_LEN             (16)
#define BD_ADDR_LEN              6
#define TASK_ID_GAPC             13
#define TASK_FIRST_MSG(task) ((uint16_t)((task) << 8))
#define GAPC_DISCONNECT                   0x01
#define HCI_DISC_CMP_EVT_CODE             0x05
#define HCI_RD_REM_VER_INFO_CMP_EVT_CODE  0x0C
#define HCI_AUTH_PAYL_TO_EXP_EVT_CODE     0x57
#define HCI_ENC_CHG_EVT_CODE              0x08
#define HCI_ENC_KEY_REFRESH_CMP_EVT_CODE  0x30
    
typedef uint8_t ke_state_t;
typedef uint16_t ke_msg_id_t;
typedef uint16_t ke_task_id_t;
typedef uint16_t ke_task_id_t;

extern struct gapc_env_tag* gapc_env[10];
extern ke_state_t ke_state_get(ke_task_id_t const id);
extern ke_task_id_t gapc_get_dest_task(uint8_t conidx);
extern void gapc_send_error_evt(uint8_t conidx, uint8_t operation, const ke_task_id_t requester, uint8_t status);
extern void gapm_con_cleanup(uint8_t conidx, uint16_t conhdl, uint8_t reason);
extern void gapc_send_disconect_ind(uint8_t conidx,  uint8_t reason, uint8_t conhdl,
                              ke_task_id_t dest_id);

enum smpc_addr_src
{
    /// Local info.
    SMPC_INFO_LOCAL,
    /// Peer info.
    SMPC_INFO_PEER,
    /// Maximum info source.
    SMPC_INFO_MAX
};

enum ke_msg_status_tag
{
    KE_MSG_CONSUMED = 0, ///< consumed, msg and ext are freed by the kernel
    KE_MSG_NO_FREE,      ///< consumed, nothing is freed by the kernel
    KE_MSG_SAVED,        ///< not consumed, will be pushed in the saved queue
};

enum gapc_op_type
{
    /// Operation used to manage Link (update params, get peer info)
    GAPC_OP_LINK_INFO    = 0x00,

    /// Operation used to manage SMP
    GAPC_OP_SMP          = 0x01,

    /// Operation used to manage connection update
    GAPC_OP_LINK_UPD     = 0x02,

    /// Max number of operations
    GAPC_OP_MAX
};

enum gapc_state_id
{
    /// Connection ready state
    GAPC_READY,

    /// Link Operation on-going
    GAPC_LINK_INFO_BUSY     = (1 << GAPC_OP_LINK_INFO),
    /// SMP Operation  on-going
    GAPC_SMP_BUSY           = (1 << GAPC_OP_SMP),
    /// Update Operation  on-going
    GAPC_LINK_UPD_BUSY      = (1 << GAPC_OP_LINK_UPD),
    /// SMP start encryption on-going
    GAPC_ENCRYPT_BUSY       = (1 << GAPC_OP_MAX),

    /// Disconnection  on-going
    GAPC_DISC_BUSY          = 0x1F,
    /// Free state
    GAPC_FREE               = 0X3F,

    /// Number of defined states.
    GAPC_STATE_MAX
};

enum gapc_msg_id
{
    /* Default event */
    /// Command Complete event
    GAPC_CMP_EVT = TASK_FIRST_MSG(TASK_ID_GAPC),//!< GAPC_CMP_EVT

    /* Connection state information */
    /// Indicate that a connection has been established
    GAPC_CONNECTION_REQ_IND,                    //!< GAPC_CONNECTION_REQ_IND
    /// Set specific link data configuration.
    GAPC_CONNECTION_CFM,                        //!< GAPC_CONNECTION_CFM

    /// Indicate that a link has been disconnected
    GAPC_DISCONNECT_IND,                        //!< GAPC_DISCONNECT_IND

    /* Link management command */
    /// Request disconnection of current link command.
    GAPC_DISCONNECT_CMD,                        //!< GAPC_DISCONNECT_CMD

    /* Peer device info */
    /// Retrieve information command
    GAPC_GET_INFO_CMD,                          //!< GAPC_GET_INFO_CMD
    /// Peer device attribute DB info such as Device Name, Appearance or Slave Preferred Parameters
    GAPC_PEER_ATT_INFO_IND,                     //!< GAPC_PEER_ATT_INFO_IND
    /// Indication of peer version info
    GAPC_PEER_VERSION_IND,                      //!< GAPC_PEER_VERSION_IND
    /// Indication of peer features info
    GAPC_PEER_FEATURES_IND,                     //!< GAPC_PEER_FEATURES_IND
    /// Indication of ongoing connection RSSI
    GAPC_CON_RSSI_IND,                          //!< GAPC_CON_RSSI_IND

    /* Device Name Management */
    /// Peer device request local device info such as name, appearance or slave preferred parameters
    GAPC_GET_DEV_INFO_REQ_IND,                  //!< GAPC_GET_DEV_INFO_REQ_IND
    /// Send requested info to peer device
    GAPC_GET_DEV_INFO_CFM,                      //!< GAPC_GET_DEV_INFO_CFM
    /// Peer device request to modify local device info such as name or appearance
    GAPC_SET_DEV_INFO_REQ_IND,                  //!< GAPC_SET_DEV_INFO_REQ_IND
    /// Local device accept or reject device info modification
    GAPC_SET_DEV_INFO_CFM,                      //!< GAPC_SET_DEV_INFO_CFM

    /* Connection parameters update */
    /// Perform update of connection parameters command
    GAPC_PARAM_UPDATE_CMD,                      //!< GAPC_PARAM_UPDATE_CMD
    /// Request of updating connection parameters indication
    GAPC_PARAM_UPDATE_REQ_IND,                  //!< GAPC_PARAM_UPDATE_REQ_IND
    /// Master confirm or not that parameters proposed by slave are accepted or not
    GAPC_PARAM_UPDATE_CFM,                      //!< GAPC_PARAM_UPDATE_CFM
    /// Connection parameters updated indication
    GAPC_PARAM_UPDATED_IND,                     //!< GAPC_PARAM_UPDATED_IND

    /* Bonding procedure */
    /// Start Bonding command procedure
    GAPC_BOND_CMD,                              //!< GAPC_BOND_CMD
    /// Bonding requested by peer device indication message.
    GAPC_BOND_REQ_IND,                          //!< GAPC_BOND_REQ_IND
    /// Confirm requested bond information.
    GAPC_BOND_CFM,                              //!< GAPC_BOND_CFM
    /// Bonding information indication message
    GAPC_BOND_IND,                              //!< GAPC_BOND_IND

    /* Encryption procedure */
    /// Start Encryption command procedure
    GAPC_ENCRYPT_CMD,                           //!< GAPC_ENCRYPT_CMD
    /// Encryption requested by peer device indication message.
    GAPC_ENCRYPT_REQ_IND,                       //!< GAPC_ENCRYPT_REQ_IND
    /// Confirm requested Encryption information.
    GAPC_ENCRYPT_CFM,                           //!< GAPC_ENCRYPT_CFM
    /// Encryption information indication message
    GAPC_ENCRYPT_IND,                           //!< GAPC_ENCRYPT_IND

    /* Security request procedure */
    /// Start Security Request command procedure
    GAPC_SECURITY_CMD,                          //!< GAPC_SECURITY_CMD
    /// Security requested by peer device indication message
    GAPC_SECURITY_IND,                          //!< GAPC_SECURITY_IND

    /* Signature procedure */
    /// Indicate the current sign counters to the application
    GAPC_SIGN_COUNTER_IND,                      //!< GAPC_SIGN_COUNTER_IND

    /* Device information */
    /// Indication of ongoing connection Channel Map
    GAPC_CON_CHANNEL_MAP_IND,                   //!< GAPC_CON_CHANNEL_MAP_IND

    /* Deprecated */
    /// Deprecated messages
    GAPC_DEPRECATED_0,                          //!< GAPC_DEPRECATED_0
    GAPC_DEPRECATED_1,                          //!< GAPC_DEPRECATED_1
    GAPC_DEPRECATED_2,                          //!< GAPC_DEPRECATED_2
    GAPC_DEPRECATED_3,                          //!< GAPC_DEPRECATED_3
    GAPC_DEPRECATED_4,                          //!< GAPC_DEPRECATED_4
    GAPC_DEPRECATED_5,                          //!< GAPC_DEPRECATED_5
    GAPC_DEPRECATED_6,                          //!< GAPC_DEPRECATED_6
    GAPC_DEPRECATED_7,                          //!< GAPC_DEPRECATED_7
    GAPC_DEPRECATED_8,                          //!< GAPC_DEPRECATED_8
    GAPC_DEPRECATED_9,                          //!< GAPC_DEPRECATED_9

    /* LE Ping */
    /// Update LE Ping timeout value
    GAPC_SET_LE_PING_TO_CMD,                    //!< GAPC_SET_LE_PING_TO_CMD
    /// LE Ping timeout indication
    GAPC_LE_PING_TO_VAL_IND,                    //!< GAPC_LE_PING_TO_VAL_IND
    /// LE Ping timeout expires indication
    GAPC_LE_PING_TO_IND,                        //!< GAPC_LE_PING_TO_IND

    /* LE Data Length extension*/
    /// LE Set Data Length Command
    GAPC_SET_LE_PKT_SIZE_CMD,                   //!< GAPC_SET_LE_PKT_SIZE_CMD
    /// LE Set Data Length Indication
    GAPC_LE_PKT_SIZE_IND,                       //!< GAPC_LE_PKT_SIZE_IND

    /* Secure Connections */
    /// Request to inform the remote device when keys have been entered or erased
    GAPC_KEY_PRESS_NOTIFICATION_CMD,            //!< GAPC_KEY_PRESS_NOTIFICATION_CMD
    /// Indication that a KeyPress has been performed on the peer device.
    GAPC_KEY_PRESS_NOTIFICATION_IND,            //!< GAPC_KEY_PRESS_NOTIFICATION_IND

    /* PHY Management */
    /// Set the PHY configuration for current active link
    GAPC_SET_PHY_CMD,                           //!< GAPC_SET_PHY_CMD
    /// Active link PHY configuration. Triggered when configuration is read or during an update.
    GAPC_LE_PHY_IND,                            //!< GAPC_LE_PHY_IND

    /* Channel Selection Algorithm */
    /// Indication of currently used channel selection algorithm
    /// @see struct gapc_chan_sel_algo_ind
    GAPC_CHAN_SEL_ALGO_IND,                     //!< GAPC_CHAN_SEL_ALGO_IND

    /* Preferred Slave Latency */
    /// Set the preferred slave latency (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_LATENCY_CMD,            //!< GAPC_SET_PREF_SLAVE_LATENCY_CMD
    /// Set the preferred slave event duration (for slave only, with RW controller)
    GAPC_SET_PREF_SLAVE_EVT_DUR_CMD,            //!< GAPC_SET_PREF_SLAVE_EVT_DUR_CMD

    /// Indication to the task that sends the unknown message
    GAPC_UNKNOWN_MSG_IND,                       //!< GAPC_UNKNOWN_MSG_IND

    // ---------------------- INTERNAL API ------------------------
    /* Internal messages for timer events, not part of API*/
    /// Signature procedure
    GAPC_SIGN_CMD,                              //!< GAPC_SIGN_CMD
    /// Signature result
    GAPC_SIGN_IND,                              //!< GAPC_SIGN_IND

    /// Parameter update procedure timeout indication
    GAPC_PARAM_UPDATE_TO_IND,                   //!< GAPC_PARAM_UPDATE_TO_IND
    /// Pairing procedure timeout indication
    GAPC_SMP_TIMEOUT_TIMER_IND,                 //!< GAPC_SMP_TIMEOUT_TIMER_IND
    /// Pairing repeated attempts procedure timeout indication
    GAPC_SMP_REP_ATTEMPTS_TIMER_IND,            //!< GAPC_SMP_REP_ATTEMPTS_TIMER_IND


    /* Switch Role */
    /// Switch Role Command
    GAPC_SWITCH_ROLE_CMD,
    /// Switch Role Indication
    GAPC_SWITCH_ROLE_IND,

    /* Time Sync */
    /// Time Sync Command
    GAPC_TIME_SYNC_CMD,

    /* Create Sniffer */
    ///  Create Sniffer Command
    GAPC_CREATE_SNIFFER_CMD,
    GAPC_READ_EVENT_COUNT_CMD,
    GAPC_READ_EVENT_COUNT_IND,
};

typedef int (*ke_msg_func_t)(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);

struct ke_msg_handler
{
    /// Id of the handled message.
    ke_msg_id_t id;
    /// Pointer to the handler function for the msgid above.
    ke_msg_func_t func;
};

struct bd_addr
{
    ///6-byte array address value
    uint8_t  addr[BD_ADDR_LEN];
};

typedef struct
{
    ///6-byte array address value
    uint8_t  addr[GAP_BD_ADDR_LEN];
} bd_addr_t;

struct gap_bdaddr
{
    /// BD Address of device
    bd_addr_t addr;
    /// Address type of the device 0=public/1=private random
    uint8_t addr_type;
};

struct gap_sec_key
{
    /// Key value MSB -> LSB
    uint8_t key[GAP_KEY_LEN];
};

struct smpc_env
{
    /// SMPC temporary information
    union smpc_info
    {
        /**
         * Pairing Information - This structure is allocated at the beginning of a pairing
         * or procedure. It is freed when a disconnection occurs or at the end of
         * the pairing procedure. If not enough memory can be found, the procedure will fail
         *  with an "Unspecified Reason" error
         */
        struct smpc_pair_info *pair;

        /**
         * Signature Procedure Information - This structure is allocated at the beginning of a
         * signing procedure. It is freed when a disconnection occurs or at the end of
         * the signing procedure. If not enough memory can be found, the procedure will fail
         *  with an "Unspecified Reason" error.
         */
        struct smpc_sign_info *sign;
    } info;

    /// CSRK values (Local and remote)
    struct gap_sec_key csrk[SMPC_INFO_MAX];

    /// signature counter values (Local and remote)
    uint32_t sign_counter[SMPC_INFO_MAX];

    /// Repeated Attempt Timer value
    uint16_t rep_att_timer_val;

    /// Encryption key size
    uint8_t key_size;

    /**
     * Contains the current state of the two timers needed in the SMPC task
     *      Bit 0 - Is Timeout Timer running
     *      Bit 1 - Is Repeated Attempt Timer running
     *      Bit 2 - Has task reached a SMP Timeout
     */
    uint8_t timer_state;

    /// State of the current procedure
    uint8_t state;

    bool secure_connections_enabled;
};

struct gapc_env_tag
{
    /// Request operation Kernel message
    void* operation[GAPC_OP_MAX];
    /// Source task id of requested disconnection
    ke_task_id_t disc_requester;
    /// Destination task ID for asynchronous events not linked to an operation
    ke_task_id_t dest_task_id;

    /* Connection parameters to keep */

    /// Security Management Protocol environment variables
    struct smpc_env smpc;

    /// connection handle
    uint16_t conhdl;

    /// Configuration fields (@see enum gapc_fields)
    uint8_t fields;

    /// BD Address used for the link that should be kept
    struct gap_bdaddr src[SMPC_INFO_MAX];

    ///Peer address type - 0=public/1=random
    uint8_t             peer_addr_type;
    ///Peer address
    struct bd_addr      peer_addr;

    /// Relevant information of peer LE features 8-byte array
    uint8_t features;
    /// Channel Selection Algorithm
    uint8_t chan_sel_algo;

    /// sniffer flag
    bool sniffer_flag;

    /// rcv local rpa addr generated by controller flag 
    bool local_rpa_flag;
};

struct hci_disc_cmp_evt
{
    ///Status of received command
    uint8_t     status;
    ///Connection Handle
    uint16_t    conhdl;
    ///Reason for disconnection
    uint8_t     reason;
};

typedef struct hci_cmd_handler_tab_info
{
    struct ke_msg_handler * hci_handler_tab_p;
    uint32_t tab_size;
}hci_cmd_handler_tab_info_t;

extern hci_cmd_handler_tab_info_t hci_cmd_handler_tab_info[4];
uint8_t discon_not_report_flag=0;
static int hci_disc_cmp_evt_handler_patch(ke_msg_id_t const msgid,
        struct hci_disc_cmp_evt const *cmp_evt,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if(state != GAPC_FREE)
    {
        // send a disconnection message
        if(gapc_env[conidx]->disc_requester != TASK_NONE)
        {
            discon_not_report_flag=1;
        }
        gapc_send_disconect_ind(conidx, cmp_evt->reason, cmp_evt->conhdl, gapc_get_dest_task(conidx));

        // send completed event only if it's a disconnect request
        if(gapc_env[conidx]->disc_requester != TASK_NONE)
        {
            /* send disconnect command complete event */
            gapc_send_error_evt(conidx, GAPC_DISCONNECT, gapc_env[conidx]->disc_requester,
                    RW_ERR_HCI_TO_HL(cmp_evt->status));
        }

        // cleanup Allocated connection resources
        gapm_con_cleanup(conidx, cmp_evt->conhdl, cmp_evt->reason);
    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

extern int hci_rd_rem_ver_info_cmp_evt_handler(ke_msg_id_t const msgid, void const *cmp_evt,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int hci_auth_payl_to_exp_evt_handler(ke_msg_id_t const msgid, void const *event,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int hci_enc_chg_evt_handler(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int hci_enc_key_refr_evt_handler(ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id);
                                   
static struct ke_msg_handler hci_event_handler_tab_patch[] =
{
    { HCI_DISC_CMP_EVT_CODE,                       (ke_msg_func_t) hci_disc_cmp_evt_handler_patch },
    { HCI_RD_REM_VER_INFO_CMP_EVT_CODE,            (ke_msg_func_t) hci_rd_rem_ver_info_cmp_evt_handler },
    { HCI_AUTH_PAYL_TO_EXP_EVT_CODE,               (ke_msg_func_t) hci_auth_payl_to_exp_evt_handler },
    { HCI_ENC_CHG_EVT_CODE,                        (ke_msg_func_t) hci_enc_chg_evt_handler },
    { HCI_ENC_KEY_REFRESH_CMP_EVT_CODE,            (ke_msg_func_t) hci_enc_key_refr_evt_handler },
};

void app_disc_cmp_evt_replace(void)
{
    hci_cmd_handler_tab_info[2].hci_handler_tab_p = hci_event_handler_tab_patch;
    hci_cmd_handler_tab_info[2].tab_size = 5;
}
