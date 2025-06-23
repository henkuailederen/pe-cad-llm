# Power Module Design Configuration Dataset

## Overview

This dataset is specifically designed for power module design tasks, containing 35,072 high-quality instruction-input-output triplets. The dataset uses the Alpaca format and supports automatic generation of YAML configuration files from natural language descriptions for power modules.

## Dataset Statistics

| Attribute | Value |
|-----------|-------|
| Total Samples | 35,072 |
| File Size | ~40MB |
| Format | JSONL (JSON Lines) |
| Encoding | UTF-8 |
| Language | English |

## Data Format

Each data entry contains three fields:

```json
{
  "instruction": "Task instruction (fixed)",
  "input": "Power module design requirement description (natural language)",
  "output": "Corresponding YAML configuration (structured output)"
}
```

### Field Descriptions

1. **instruction**: Fixed task instruction
   - Content: "Generate the corresponding YAML configuration for the power module based on the given description."

2. **input**: Power module design description
   - Natural language description of power module topology, parameters, constraints, etc.
   - Includes chip quantities, connection methods, geometric parameters, and other information

3. **output**: Corresponding YAML configuration
   - Structured power module configuration file
   - Contains all necessary design parameters

## Supported Power Module Types

### Half-Bridge Topology (HB)
- **HB_V1**: Basic half-bridge structure
- **HB_V2**: Multi-DBC half-bridge structure
- **HB_V3**: Optimized half-bridge structure
- **HB_V4**: Advanced half-bridge structure

### Three-Phase Six-Switch Topology (3P6P)
- **3P6P_V1**: Basic three-phase six-switch structure
- **3P6P_V2**: Optimized three-phase six-switch structure

## Configuration Parameter Categories

### 1. Basic Geometric Parameters
```yaml
ceramics_width: 41.5          # Ceramic substrate width
ceramics_length: 45.5         # Ceramic substrate length
ceramics_thickness: 0.8       # Ceramic substrate thickness
upper_copper_thickness: 0.2   # Upper copper layer thickness
lower_copper_thickness: 0.3   # Lower copper layer thickness
fillet_radius: 0.5           # Fillet radius
```

### 2. Chip Dimension Parameters
```yaml
igbt_width: 10               # IGBT width
igbt_length: 10              # IGBT length
fwd_width: 5                 # Freewheeling diode width
fwd_length: 7                # Freewheeling diode length
```

### 3. Margin and Spacing Parameters
```yaml
cu2cu_margin: 1.0            # Copper-to-copper spacing
cu2ceramics_margin: 1.5      # Copper-to-ceramic margin
dbc2dbc_margin: 3.0          # DBC-to-DBC spacing
substrate_edge_margin: 3.0   # Substrate edge margin
```

### 4. Process Parameters
```yaml
substrate_solder_thickness: 0.15  # Substrate solder thickness
die_solder_thickness: 0.1         # Die solder thickness
die_thickness: 0.15               # Die thickness
```

### 5. Bond Wire Configuration
```yaml
igbt_bondwires: 4            # Number of IGBT bond wires
fwd_bondwires: 4             # Number of FWD bond wires
```

### 6. Position and Rotation
```yaml
igbt_positions:              # IGBT positions
  type_1: [[0.41, 0.48]]
fwd_positions: [[0.48, 0.18]]  # FWD positions
igbt_rotations:              # IGBT rotation angles
  type_1: [0]
fwd_rotations: [0]           # FWD rotation angles
```

### 7. Gate Design
```yaml
gate_design:
  type_1:
    start: [0.9639, 0.5604]  # Starting point proportional coordinates
    moves: [[-0.12, 0], [0, -0.11], [0.12, 0]]  # Relative movements
```

### 8. Cutting Path
```yaml
cutting_design:
  path_1: [[0.27, "MIN"], [0.27, 0.33], [0.14, 0.33]]
```

### 9. Connection Topology
```yaml
module_connections: [["zone0_0", "zone1_1"]]  # Module-level connections
dbc_connections: [                            # DBC-level connections
  ["igbt0", "zone0"], 
  ["fwd0", "zone0"]
]
```

## Data Quality Assurance

1. **Syntax Correctness**: All YAML outputs are syntax-validated
2. **Parameter Reasonableness**: Geometric parameters comply with physical constraints
3. **Topology Consistency**: Connection relationships conform to circuit rules
4. **Diversity**: Covers various design scenarios and parameter combinations

## Usage Recommendations

### Fine-tuning Large Models
```python
import json

# Load dataset
def load_dataset(file_path):
    data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            data.append(json.loads(line))
    return data

dataset = load_dataset('dataset/dataset-alpaca.jsonl')
```

### Data Preprocessing
1. Text cleaning and standardization
2. YAML format validation
3. Parameter range checking
4. Train/validation/test set splitting

### Evaluation Metrics
- YAML syntax accuracy
- Parameter reasonableness checks
- Geometric constraint satisfaction rate
- Circuit topology correctness

## Dataset Updates

- **Version**: v1.0
- **Release Date**: June 2025
- **Update Plan**: Continuous improvement and expansion based on community feedback

## License Agreement

This dataset is released under the MIT license and can be freely used for academic research and commercial applications. 