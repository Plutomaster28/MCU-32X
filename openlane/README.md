# OpenLane Configuration and Scripts

This directory contains the OpenLane configuration files and scripts needed to reproduce the MCU-32X ASIC flow.

## Files

- `config.tcl` - Main OpenLane configuration file
- `run.sh` - Script to run the complete flow
- `README.md` - This file

## Usage

### Option 1: Using the run script
```bash
cd openlane/
./run.sh
```

### Option 2: Manual execution
```bash
# Start OpenLane container
make mount

# Run the flow
flow.tcl -design MCU-32X -tag production_run
```

## Configuration Details

The `config.tcl` file configures:
- Target SkyWater 130nm PDK
- 200MHz clock frequency (5ns period)
- Conservative 10% core utilization
- 500µm × 500µm die size
- Power grid settings optimized for the design
- Routing configuration for met1 through met5

## Expected Results

A successful run should produce:
- **DRC:** 1 minor violation (N-well tap)
- **LVS:** Clean - no violations
- **STA:** Clean - no timing violations
- **Final GDSII:** Ready for tapeout

## Troubleshooting

If you encounter issues:

1. **PDN pitch errors:** Check power grid settings in config.tcl
2. **Routing layer errors:** Verify metal layer configuration
3. **DRC violations:** Expected 1 minor N-well violation is acceptable

For questions, see the main project README or create an issue.
