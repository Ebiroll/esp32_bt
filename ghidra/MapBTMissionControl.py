# Map Bluetooth Stack: Descriptors, ACL Environments, and Global Driver State
# @category: ESP32_BT

from ghidra.program.model.data import StructureDataType, WordDataType, ArrayDataType, ByteDataType, UnsignedIntegerDataType, PointerDataType
from ghidra.program.model.util import CodeUnitInsertionException

def map_bt_stack_final():
    dtm = currentProgram.getDataTypeManager()

    # --- 1. em_bt_rx_desc_t (14 bytes) ---
    rx_desc = StructureDataType("em_bt_rx_desc_t", 14)
    rx_desc.add(WordDataType(), 2, "ctrl", "Bit 15: Done/Ready")
    rx_desc.add(WordDataType(), 2, "bt_header", "LT_ADDR, Type, etc.")
    rx_desc.add(WordDataType(), 2, "acl_header", "LLID, Length")
    rx_desc.add(WordDataType(), 2, "rxdataptr", "Offset to EM Data")
    rx_desc.add(WordDataType(), 2, "rxrfstat", "RSSI and Channel info")
    rx_desc.add(WordDataType(), 2, "rate", "Modulation info")
    rx_desc.add(WordDataType(), 2, "stat", "HW Status flags")
    rx_desc = dtm.addDataType(rx_desc, None)

    # --- 2. em_bt_tx_desc_t (20 bytes) ---
    tx_desc = StructureDataType("em_bt_tx_desc_t", 20)
    tx_desc.add(WordDataType(), 2, "txctrl", "Bit 15: Ready Flag")
    tx_desc.add(WordDataType(), 2, "bt_header", "Link Layer Header")
    tx_desc.add(WordDataType(), 2, "acl_header", "Data Layer Header")
    tx_desc.add(WordDataType(), 2, "txdataptr", "Offset to EM Data")
    tx_desc.add(WordDataType(), 2, "mic", "MIC/CRC Seed")
    tx_desc.add(WordDataType(), 2, "txrate", "Power and Rate")
    tx_desc.add(WordDataType(), 2, "txstat", "Hardware status update")
    tx_desc.add(WordDataType(), 2, "txheaderptr", "EM Header buffer offset")
    tx_desc.add(UnsignedIntegerDataType(), 4, "timestamp", "TX Target Clock")
    tx_desc = dtm.addDataType(tx_desc, None)

    # --- 3. bt_acl_env_t (252 bytes / 0xFC) ---
    # Based on r_ld_acl_start logic
    acl_env = StructureDataType("bt_acl_env_t", 252)
    acl_env.add(PointerDataType(), 4, "ea_ptr", "Element header")
    acl_env.add(UnsignedIntegerDataType(), 4, "unknown_p2", "")
    acl_env.add(UnsignedIntegerDataType(), 4, "conn_handle", "HCI Connection Handle")
    acl_env.add(UnsignedIntegerDataType(), 4, "anchor_point", "BT Clock Anchor")
    acl_env.add(WordDataType(), 2, "bit_offset", "Slot Bit Offset")
    acl_env.add(WordDataType(), 2, "conn_state", "State (e.g., 0x46A)")
    acl_env.add(ByteDataType(), 1, "priority", "RWIP Priority")
    acl_env.add(ByteDataType(), 1, "hop_inc", "Hop Increment")
    acl_env.add(WordDataType(), 2, "int_mask", "Interrupt Mask (0x203)")
    acl_env.add(PointerDataType(), 4, "start_cb", "ld_acl_evt_start_cbk")
    acl_env.add(PointerDataType(), 4, "stop_cb", "ld_acl_evt_stop_cbk")
    acl_env.add(PointerDataType(), 4, "canceled_cb", "ld_acl_evt_canceled_cbk")
    acl_env.insert(0x8C, UnsignedIntegerDataType(), 4, "bd_addr_low", "Remote MAC [0:3]")
    acl_env.insert(0x96, WordDataType(), 2, "bd_addr_high", "Remote MAC [4:5]")
    acl_env.insert(0xB2, ByteDataType(), 1, "link_id", "ACL Link ID")
    acl_env.insert(0xB3, ByteDataType(), 1, "role", "0=Master, 1=Slave")
    acl_env = dtm.addDataType(acl_env, None)

    # --- 4. ld_env_t (Global State) ---
    ld_env_struct = StructureDataType("ld_env_t", 512)
    ld_env_struct.add(ArrayDataType(ByteDataType(), 426, 1), 426, "padding", "")
    ld_env_struct.add(ByteDataType(), 1, "curr_rxdesc_index", "Current RX Ring Index")
    ld_env_struct = dtm.addDataType(ld_env_struct, None)

    def apply_type(addr_hex, name, datatype, count=1):
        addr = parseAddress(addr_hex)
        if currentProgram.getMemory().getBlock(addr) is None:
            print("SKIPPED: {} at {} - Memory missing!".format(name, addr_hex))
            return
        full_type = datatype if count == 1 else ArrayDataType(datatype, count, datatype.getLength())
        removeDataAt(addr)
        clearListing(addr, addr.add(full_type.getLength() - 1))
        createData(addr, full_type)
        createLabel(addr, name, True)

    # Execute Mapping
    apply_type("0x3ffb2382", "BT_RX_DESCRIPTORS", rx_desc, 4)
    apply_type("0x3ffb23ba", "BT_TX_DESCRIPTORS", tx_desc, 4)
    apply_type("0x3FFB8258", "ld_acl_env_ptrs", PointerDataType(acl_env), 8)
    apply_type("0x3ffb9510", "ld_env", ld_env_struct, 1)

map_bt_stack_final()
