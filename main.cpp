#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <cstring>
#include <iostream>
#include "DMA.h"

/*
 * Memory module: simple TLM target with a byte-array backing store.
 */
SC_MODULE(Memory) {
    tlm_utils::simple_target_socket<Memory> slave_p;

    static const uint32_t MEM_SIZE = 4096; // 4 KB
    uint8_t mem[MEM_SIZE];

    SC_CTOR(Memory) : slave_p("slave_p") {
        slave_p.register_b_transport(this, &Memory::b_transport);
        memset(mem, 0, MEM_SIZE);
    }

    void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
        uint64_t       addr   = trans.get_address();
        unsigned char* data   = trans.get_data_ptr();
        unsigned int   length = trans.get_data_length();

        if (addr + length > MEM_SIZE) {
            trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
            return;
        }

        // Simulate memory access delay.
        wait(delay); 
        delay = SC_ZERO_TIME;

        if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
            memcpy(mem + addr, data, length);
        } else {
            memcpy(data, mem + addr, length);
        }

        trans.set_response_status(tlm::TLM_OK_RESPONSE);
    }
};

/*
 * CPU module: TLM initiator that programs the DMA and waits for IRQ.
 */
SC_MODULE(CPU) {
    tlm_utils::simple_initiator_socket<CPU> master_p;
    sc_in<bool> interrupt;

    SC_CTOR(CPU) : master_p("master_p") {
        SC_THREAD(cpu_p);
    }

    // Helper: write a 32-bit register to the DMA slave port.
    void dma_write(uint32_t addr, uint32_t value) {
        tlm::tlm_generic_payload trans;
        sc_time delay = sc_time(10, SC_NS);

        trans.set_command(tlm::TLM_WRITE_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(reinterpret_cast<unsigned char*>(&value));
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_byte_enable_length(0);
        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        master_p->b_transport(trans, delay);

        if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
            std::cerr << "[CPU] DMA register write error at 0x"
                      << std::hex << addr << std::endl;
        }
    }

    void cpu_p() {
        // Wait a few cycles before starting.
        wait(5, SC_NS);

        // Test: program DMA to copy 64 bytes from 0x100 to 0x200.
        std::cout << "[CPU] Programming DMA at "
                  << sc_time_stamp() << std::endl;

        dma_write(0x0, 0x100); // SOURCE = 0x100
        dma_write(0x4, 0x200); // TARGET = 0x200
        dma_write(0x8, 64);    // SIZE   = 64 bytes
        dma_write(0xC, 1);     // START  = 1, trigger DMA

        std::cout << "[CPU] DMA started, waiting for interrupt..."
                  << std::endl;

        // Wait for DMA to raise interrupt.
        wait(interrupt.posedge_event());

        std::cout << "[CPU] Interrupt received at "
                  << sc_time_stamp() << std::endl;

        // Clear: write 0 to START/CLEAR register.
        dma_write(0xC, 0);

        std::cout << "[CPU] DMA cleared at "
                  << sc_time_stamp() << std::endl;

        // End simulation after a few more cycles.
        wait(10, SC_NS);
        sc_stop();
    }
};

/*
 * sc_main: Instantiate and connect all modules.
 */
int sc_main(int argc, char* argv[]) {
    // Signals
    sc_clock        clk("clk", 1, SC_NS);
    sc_signal<bool> reset("reset");
    sc_signal<bool> interrupt("interrupt");

    // Module instantiation
    CPU    cpu("cpu");
    DMA    dma("dma");
    Memory mem("mem");

    // Connect DMA ports.
    dma.clk(clk);
    dma.reset(reset);
    dma.interrupt(interrupt);

    // CPU master port -> DMA slave port (CPU programs DMA registers).
    cpu.master_p.bind(dma.slave_p);

    // DMA master port -> Memory slave port (DMA reads/writes data).
    dma.master_p.bind(mem.slave_p);

    // DMA interrupt output -> CPU interrupt input.
    cpu.interrupt(interrupt);

    // Pre-fill source memory with test data (0x100..0x13F = 0,1,...,63).
    for (int i = 0; i < 64; i++) {
        mem.mem[0x100 + i] = (uint8_t)i;
    }

    // VCD waveform tracing.
    sc_trace_file* tf = sc_create_vcd_trace_file("RESULT");
    tf->set_time_unit(1, SC_PS);
    sc_trace(tf, clk,       "clk");
    sc_trace(tf, reset,     "reset");
    sc_trace(tf, interrupt, "interrupt");
    sc_trace(tf, dma.dbg_SOURCE,      "SOURCE");
    sc_trace(tf, dma.dbg_TARGET,      "TARGET");
    sc_trace(tf, dma.dbg_SIZE,        "SIZE");
    sc_trace(tf, dma.dbg_START_CLEAR, "START_CLEAR");

    sc_trace(tf, dma.dbg_xfer_src,    "xfer_src");
    sc_trace(tf, dma.dbg_xfer_dst,    "xfer_dst");
    sc_trace(tf, dma.dbg_xfer_size,   "xfer_size");
    sc_trace(tf, dma.dbg_xfer_idx,    "xfer_idx");
    sc_trace(tf, dma.dbg_state,       "dma_state");

    // Drive reset: assert low for first 3 ns, then deassert.
    reset.write(false);
    sc_start(3, SC_NS);
    reset.write(true);

    std::cout << "[SIM] Reset deasserted at "
              << sc_time_stamp() << std::endl;

    // Run simulation.
    sc_start();

    // Verify result: check that 0x200..0x23F matches 0x100..0x13F.
    bool pass = true;
    for (int i = 0; i < 64; i++) {
        if (mem.mem[0x200 + i] != (uint8_t)i) {
            std::cerr << "[VERIFY] FAIL at byte " << i
                      << ": expected " << (int)i
                      << " got " << (int)mem.mem[0x200 + i]
                      << std::endl;
            pass = false;
        }
    }
    if (pass) {
        std::cout << "[VERIFY] PASS - DMA copied 64 bytes correctly."
                  << std::endl;
    }

    sc_close_vcd_trace_file(tf);
    return pass ? 0 : 1;
}
