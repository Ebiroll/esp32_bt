# Map BTDM Specific Memory Regions and Data Types
# @category: ESP32_BT

from ghidra.program.model.data import StructureDataType, WordDataType, ArrayDataType, ByteDataType, UnsignedIntegerDataType, PointerDataType
from ghidra.program.model.mem import MemoryBlock

def map_btdm_areas():
    dtm = currentProgram.getDataTypeManager()
    memory = currentProgram.getMemory()

    # --- 1. Helper for creating hardware/reserved blocks ---
    def add_bt_block(name, start_hex, end_hex, r, w, x):
        addr = parseAddress(start_hex)
        length = parseAddress(end_hex).getOffset() - addr.getOffset()
        if memory.getBlock(addr) is None:
            try:
                # Use uninitialized blocks for hardware/BSS regions
                memory.createUninitializedBlock(name, addr, length, False)
                block = memory.getBlock(addr)
                block.setRead(r); block.setWrite(w); block.setExecute(x)
                print("CREATED BTDM AREA: {} [{} - {}]".format(name, start_hex, end_hex))
            except Exception as e:
                print("FAILED to create {}: {}".format(name, str(e)))

    # --- 2. Map BTDM Hardware & Reserved Regions ---
    # Based on your SOC_MEM_BT macros
    add_bt_block("BTDM_DATA",     "0x3ffae6e0", "0x3ffaff10", True, True, False) #
    add_bt_block("BT_EM_BTDM0",   "0x3ffb0000", "0x3ffb09a8", True, True, False) #
    add_bt_block("BT_EM_BLE",     "0x3ffb09a8", "0x3ffb1ddc", True, True, False) #
    add_bt_block("BT_EM_BTDM1",   "0x3ffb1ddc", "0x3ffb2730", True, True, False) #
    add_bt_block("BT_EM_BREDR",   "0x3ffb2730", "0x3ffb7cd8", True, True, False) #
    add_bt_block("BTDM_BSS",      "0x3ffb8000", "0x3ffb9a20", True, True, False) #
    add_bt_block("BTDM_MISC",     "0x3ffbdb28", "0x3ffbdb5c", True, True, False) #

    # --- 3. Define BT Specific Data Types ---
    # btdm_env_t (0x30 bytes) based on your decompilation
    btdm_env = StructureDataType("btdm_env_t", 48)
    btdm_env.add(PointerDataType(), 4, "env_memory", "Link Layer Environment")
    btdm_env.add(UnsignedIntegerDataType(), 4, "env_size", "")
    btdm_env.add(PointerDataType(), 4, "msg_mem", "HCI/MSG Environment")
    btdm_env.add(UnsignedIntegerDataType(), 4, "msg_size", "")
    btdm_env.add(PointerDataType(), 4, "noret_mem", "Non-retention Memory")
    btdm_env.add(UnsignedIntegerDataType(), 4, "noret_size", "")
    btdm_env.add(PointerDataType(), 4, "hci_env_ptr", "HCI Pointer")
    btdm_env.add(PointerDataType(), 4, "vhci_ptr", "VHCI Pointer")
    btdm_env.add(PointerDataType(), 4, "vhci_block_ptr", "VHCI Base Block")
    btdm_env = dtm.addDataType(btdm_env, None)

    # --- 4. Apply Types to Key BT Pointers ---
    def apply_type(addr_hex, name, datatype):
        addr = parseAddress(addr_hex)
        if memory.getBlock(addr) is not None:
            removeDataAt(addr)
            createData(addr, datatype)
            createLabel(addr, name, True)

    # Label the global environment pointer we found in GDB
    apply_type("0x3ffbdb48", "btdm_env_p", PointerDataType(btdm_env))

    print("BTDM Area Mapping Complete.")

map_btdm_areas()
