# Map Bluetooth Stack including ld_env
# @category: ESP32_BT

from ghidra.program.model.data import StructureDataType, WordDataType, ArrayDataType, ByteDataType, UnsignedIntegerDataType
from ghidra.program.model.util import CodeUnitInsertionException

def map_bt_stack():
    dtm = currentProgram.getDataTypeManager()

    # --- Structure Definitions ---
    rx_desc = StructureDataType("em_bt_rx_desc_t", 0)
    rx_desc.add(WordDataType(), 2, "ctrl", "Status/Flags")
    rx_desc.add(WordDataType(), 2, "bt_header", "BT Link Header")
    rx_desc.add(WordDataType(), 2, "acl_header", "ACL/Data Header")
    rx_desc.add(WordDataType(), 2, "rxdataptr", "EM Offset to Data")
    rx_desc.add(WordDataType(), 2, "rxrfstat", "RSSI / Channel")
    rx_desc.add(WordDataType(), 2, "rate", "Modulation Info")
    rx_desc.add(WordDataType(), 2, "stat", "HW Completion Status")
    rx_desc = dtm.addDataType(rx_desc, None)

    tx_desc = StructureDataType("em_bt_tx_desc_t", 0)
    tx_desc.add(WordDataType(), 2, "txctrl", "Bit 15: Ready Flag")
    tx_desc.add(WordDataType(), 2, "bt_header", "BT Link Header")
    tx_desc.add(WordDataType(), 2, "acl_header", "ACL/Data Header")
    tx_desc.add(WordDataType(), 2, "txdataptr", "EM Offset to Data")
    tx_desc.add(WordDataType(), 2, "mic", "MIC/CRC Seed")
    tx_desc.add(WordDataType(), 2, "txrate", "Power/Modulation")
    tx_desc.add(WordDataType(), 2, "txstat", "HW Status")
    tx_desc.add(WordDataType(), 2, "txheaderptr", "EM Offset to Header")
    tx_desc.add(UnsignedIntegerDataType(), 4, "timestamp", "Target TX Clock")
    tx_desc = dtm.addDataType(tx_desc, None)

    link_ctrl = StructureDataType("bt_link_ctrl_t", 0)
    link_ctrl.add(WordDataType(), 2, "state_flags", "Connection State")
    link_ctrl.add(ArrayDataType(ByteDataType(), 6, 1), 6, "remote_bdaddr", "Peer MAC Address")
    link_ctrl.add(ArrayDataType(ByteDataType(), 16, 1), 16, "unknown_gap", "")
    link_ctrl.add(WordDataType(), 2, "enc_status", "Encryption Status")
    link_ctrl.add(ArrayDataType(ByteDataType(), 76, 1), 76, "remaining_data", "")
    link_ctrl = dtm.addDataType(link_ctrl, None)

    def apply_bt_data(addr_hex, name, datatype, count=1):
        addr = parseAddress(addr_hex)
        if currentProgram.getMemory().getBlock(addr) is None:
            print("SKIPPED: {} at {} - No memory block defined here!".format(name, addr_hex))
            return

        full_type = datatype
        if count > 1:
            full_type = ArrayDataType(datatype, count, datatype.getLength())
        
        size = full_type.getLength()
        removeDataAt(addr)
        clearListing(addr, addr.add(size - 1))
        
        try:
            createData(addr, full_type)
            createLabel(addr, name, True)
            print("MAPPED: {} at {}".format(name, addr_hex))
        except:
            print("FAILED: Conflict at {}".format(addr_hex))

    # --- Execute ---
    print("Starting Bluetooth Meta-Mapping...")
    apply_bt_data("0x3ffb1dee", "BT_LINK_CONTROL_BLOCKS", link_ctrl, 2)
    apply_bt_data("0x3ffb2382", "BT_RX_DESCRIPTORS", rx_desc, 4)
    apply_bt_data("0x3ffb23ba", "BT_TX_DESCRIPTORS", tx_desc, 4)
    apply_bt_data("0x3ffb9510", "ld_env", ByteDataType(), 512) # Mapped as raw bytes for now
    
    fhs = parseAddress("0x3ffb262c")
    if currentProgram.getMemory().getBlock(fhs):
        createLabel(fhs, "BT_FHS_PACKET_BUFFER", True)

map_bt_stack()
