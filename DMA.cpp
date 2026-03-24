#include "DMA.h"
#include <cstring>
#include <iostream>

/*
 * Slave b_transport: handles CPU register reads and writes.
 * wait(delay) is called AFTER the read/write to model transmission delay.
 */
void DMA::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
    tlm::tlm_command cmd    = trans.get_command();
    uint64_t         addr   = trans.get_address();
    unsigned char*   data   = trans.get_data_ptr();
    unsigned int     length = trans.get_data_length();

    // Calculate register offset relative to BASE.
    uint32_t offset = (uint32_t)(addr - BASE);

    if (cmd == tlm::TLM_WRITE_COMMAND) {
        uint32_t value = 0;
        memcpy(&value, data, (length < 4) ? length : 4);

        switch (offset) {
            case 0x0:  SOURCE      = value; break;
            case 0x4:  TARGET      = value; break;
            case 0x8:  SIZE        = value; break;
            case 0xC:  START_CLEAR = value; break;
            default:
                trans.set_response_status(
                    tlm::TLM_ADDRESS_ERROR_RESPONSE);
                return;
        }
    } else if (cmd == tlm::TLM_READ_COMMAND) {
        uint32_t value = 0;
        switch (offset) {
            case 0x0:  value = SOURCE;      break;
            case 0x4:  value = TARGET;      break;
            case 0x8:  value = SIZE;        break;
            case 0xC:  value = START_CLEAR; break;
            default:
                trans.set_response_status(
                    tlm::TLM_ADDRESS_ERROR_RESPONSE);
                return;
        }
        memcpy(data, &value, (length < 4) ? length : 4);
    }

    // Simulate transmission delay after completing the register access.
    wait(delay);
    delay = SC_ZERO_TIME;

    trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

/*
 * DMA SC_CTHREAD process.
 * Uses a state machine with exactly one wait() at the top of the main
 * loop, matching the synchronous hardware clocked always-block pattern.
 */
void DMA::dma_p() {
    // Reset section: clear all registers and outputs.
    SOURCE      = 0;
    TARGET      = 0;
    SIZE        = 0;
    START_CLEAR = 0;
    interrupt.write(false);
    dma_state = IDLE;
    xfer_src = xfer_dst = xfer_size = xfer_idx = 0;

    dbg_SOURCE.write(0);
    dbg_TARGET.write(0);
    dbg_SIZE.write(0);
    dbg_START_CLEAR.write(0);
    dbg_xfer_src.write(0);
    dbg_xfer_dst.write(0);
    dbg_xfer_size.write(0);
    dbg_xfer_idx.write(0);
    dbg_state.write(0);

    while (true) {
        wait(); // single wait - advance one clock cycle first

        switch (dma_state) {

            case IDLE:
                if (START_CLEAR != 0) {
                    // Latch transfer parameters and begin transfer.
                    xfer_src  = SOURCE;
                    xfer_dst  = TARGET;
                    xfer_size = SIZE;
                    xfer_idx  = 0;
                    dma_state = TRANSFER;
                }
                break;

            case TRANSFER: {
                if (xfer_idx < xfer_size) {
                    uint32_t chunk =
                        ((xfer_size - xfer_idx) >= 4)
                        ? 4 : (xfer_size - xfer_idx);
                    uint8_t buf[4];
                    tlm::tlm_generic_payload trans;
                    sc_time delay = sc_time(10, SC_NS);

                    // Read one chunk from source memory.
                    trans.set_command(tlm::TLM_READ_COMMAND);
                    trans.set_address(xfer_src + xfer_idx);
                    trans.set_data_ptr(buf);
                    trans.set_data_length(chunk);
                    trans.set_streaming_width(chunk);
                    trans.set_byte_enable_ptr(nullptr);
                    trans.set_byte_enable_length(0);
                    trans.set_dmi_allowed(false);
                    trans.set_response_status(
                        tlm::TLM_INCOMPLETE_RESPONSE);
                    master_p->b_transport(trans, delay);

                    if (trans.get_response_status()
                            != tlm::TLM_OK_RESPONSE) {
                        std::cerr << "[DMA] Read error at 0x"
                                  << std::hex
                                  << (xfer_src + xfer_idx)
                                  << std::endl;
                    }

                    // Write one chunk to target memory.
                    delay = sc_time(10, SC_NS);
                    trans.set_command(tlm::TLM_WRITE_COMMAND);
                    trans.set_address(xfer_dst + xfer_idx);
                    trans.set_data_ptr(buf);
                    trans.set_data_length(chunk);
                    trans.set_streaming_width(chunk);
                    trans.set_byte_enable_ptr(nullptr);
                    trans.set_byte_enable_length(0);
                    trans.set_dmi_allowed(false);
                    trans.set_response_status(
                        tlm::TLM_INCOMPLETE_RESPONSE);
                    master_p->b_transport(trans, delay);

                    if (trans.get_response_status()
                            != tlm::TLM_OK_RESPONSE) {
                        std::cerr << "[DMA] Write error at 0x"
                                  << std::hex
                                  << (xfer_dst + xfer_idx)
                                  << std::endl;
                    }

                    xfer_idx += chunk;
                } else {
                    // All data transferred - raise interrupt.
                    interrupt.write(true);
                    dma_state = WAIT_CLEAR;
                }
                break;
            }

            case WAIT_CLEAR:
                // Wait for CPU to acknowledge by writing START_CLEAR = 0.
                if (START_CLEAR == 0) {
                    interrupt.write(false);
                    dma_state = IDLE;
                }
                break;
        }
        dbg_SOURCE.write(SOURCE);
        dbg_TARGET.write(TARGET);
        dbg_SIZE.write(SIZE);
        dbg_START_CLEAR.write(START_CLEAR);

        dbg_xfer_src.write(xfer_src);
        dbg_xfer_dst.write(xfer_dst);
        dbg_xfer_size.write(xfer_size);
        dbg_xfer_idx.write(xfer_idx);
        dbg_state.write((unsigned int)dma_state);
    }
}
