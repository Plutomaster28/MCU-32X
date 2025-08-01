#include "Vtb_mcu32x.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

int main(int argc, char** argv) {
    // Initialize Verilator
    Verilated::commandArgs(argc, argv);
    
    // Create instance of our module
    Vtb_mcu32x* tb = new Vtb_mcu32x;
    
    // Initialize VCD tracing
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    tb->trace(tfp, 99);
    tfp->open("mcu32x.vcd");
    
    // Simulate
    vluint64_t main_time = 0;
    while (!Verilated::gotFinish() && main_time < 1000) {
        tb->eval();
        tfp->dump(main_time);
        main_time++;
    }
    
    // Clean up
    tfp->close();
    delete tb;
    delete tfp;
    
    return 0;
}
