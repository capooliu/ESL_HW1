#ifndef DMA_H
#define DMA_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

SC_MODULE(DMA) {
    // Ports
    sc_in<bool>  clk;
    sc_in<bool>  reset;    // synchronous, active-low
    sc_out<bool> interrupt;

    // TLM sockets
    tlm_utils::simple_initiator_socket<DMA> master_p;  // DMA reads/writes memory
    tlm_utils::simple_target_socket<DMA>   slave_p;   // CPU writes DMA registers

    // Debug signals for waveform tracing
    sc_signal<sc_uint<32>> dbg_SOURCE;
    sc_signal<sc_uint<32>> dbg_TARGET;
    sc_signal<sc_uint<32>> dbg_SIZE;
    sc_signal<sc_uint<32>> dbg_START_CLEAR;

    sc_signal<sc_uint<32>> dbg_xfer_src;
    sc_signal<sc_uint<32>> dbg_xfer_dst;
    sc_signal<sc_uint<32>> dbg_xfer_size;
    sc_signal<sc_uint<32>> dbg_xfer_idx;

    sc_signal<sc_uint<2>>  dbg_state;   // 0=IDLE, 1=TRANSFER, 2=WAIT_CLEAR

    // Constructor
    SC_CTOR(DMA)
        : master_p("master_p"), slave_p("slave_p"),
          BASE(0), SOURCE(0), TARGET(0), SIZE(0), START_CLEAR(0)
    {
        slave_p.register_b_transport(this, &DMA::b_transport);
        SC_CTHREAD(dma_p, clk.pos());
        reset_signal_is(reset, false);  // synchronous, active-low
    }

    // Slave b_transport: CPU accesses DMA registers
    void b_transport(tlm::tlm_generic_payload& trans, sc_time& delay);

    // DMA process thread
    void dma_p();

private:
    // DMA control registers
    uint32_t BASE;        // base address offset
    uint32_t SOURCE;      // source address (0x0)
    uint32_t TARGET;      // target address (0x4)
    uint32_t SIZE;        // transfer size in bytes (0x8)
    uint32_t START_CLEAR; // start/clear control (0xC)

    // State machine for dma_p (must be members so reset can clear them)
    enum DMAState { IDLE, TRANSFER, WAIT_CLEAR };
    DMAState dma_state;
    uint32_t xfer_src, xfer_dst, xfer_size, xfer_idx;
};

#endif // DMA_H
