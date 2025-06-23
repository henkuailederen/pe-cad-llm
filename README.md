# Power Module Design Configuration Dataset & Inference System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Dataset Size](https://img.shields.io/badge/dataset-35k%20samples-green.svg)](./dataset/)

This project provides a large-scale dataset for LLM fine-tuning in power module design along with a corresponding inference system that supports automatic generation of power module configurations and 3D model code from natural language descriptions.

## 🚀 Key Features

- **Large-scale Dataset**: 35,072 high-quality power module design data entries
- **Multi-topology Support**: Multiple topology structures including Half-Bridge (HB), Three-Phase Six-Switch (3P6P), etc.
- **Automatic Code Generation**: Generate CadQuery 3D modeling code based on YAML configurations
- **Web Interface**: Provides an intuitive interactive design interface
- **Rapid Scalability**: Quick expansion of topology structures

## 📊 Dataset Description

### Data Format
The dataset uses Alpaca format, with each sample containing:
- `instruction`: Task instruction
- `input`: Power module design requirement description (natural language)
- `output`: Corresponding YAML configuration file

### Data Statistics
- **Total Samples**: 35,072 entries
- **File Size**: ~40MB
- **Topology Types**: 7 types
- **Language**: English descriptions

### Sample Example
```json
{
  "instruction": "Generate the corresponding YAML configuration for the power module based on the given description.",
  "input": "Build a half-bridge power module using three DBC substrates, each mounting two IGBT dies and two FWDs...",
  "output": "module_id: \"HB_V2\"\n\ncu2cu_margin: 0.9\ncu2ceramics_margin: 1.35\n..."
}
```

## 🛠️ System Architecture

### Core Components
1. **Configuration Processor** (`config_processor.py`): Processes LLM output YAML data (override) and template (default) merging
2. **Code Generator** (`templates/unified_template.py.j2`): Jinja2 template engine
3. **3D Modeling Tools** (`utils.py`): CadQuery utility function library
4. **Web Application** (`app.py`): Flask backend service
5. **Background Worker** (`worker.py`): Asynchronous task processing

## 📦 Installation & Usage

### Environment Requirements
```bash
pip install flask flask-cors requests pyyaml jinja2 numpy scipy shapely
# CadQuery installation please refer to official documentation
```

### Quick Start

1. **Start Inference Service**
```bash
cd Inference
python app.py # Need to replace the fine-tuned LLM API address in app.py
python worker.py
```

2. **Access Web Interface**
```
http://localhost:5000
```

3. **Use Dataset for Fine-tuning**
```python
import json

# Load dataset
with open('dataset/dataset-alpaca.jsonl', 'r', encoding='utf-8') as f:
    data = [json.loads(line) for line in f]

print(f"Dataset contains {len(data)} samples")
```

## 🔧 API Interface

For inference service API interface, see API.md

## 📁 Project Structure

```
Power Module Dataset/
├── dataset/
│   └── dataset-alpaca.jsonl     # Fine-tuning dataset file
├── Inference/
│   ├── app.py                   # Flask web application
│   ├── config_processor.py      # Configuration processor
│   ├── utils.py                 # CadQuery utility functions
│   ├── worker.py                # Background task processing
│   ├── defaults/                # Default configuration files
│   │   ├── HB_V1_default.yaml
│   │   ├── HB_V2_default.yaml
│   │   └── ...
│   └── templates/
│       ├── index.html           # Web interface
│       └── unified_template.py.j2  # Code generation template
└── README.md
```

## 📖 Configuration Parameter Description

### Geometric Parameters
- `ceramics_width/length/thickness`: Ceramic substrate dimensions
- `upper/lower_copper_thickness`: Upper and lower copper layer thickness
- `fillet_radius`: Fillet radius

### Chip Parameters
- `igbt_width/length`: IGBT chip dimensions
- `fwd_width/length`: Freewheeling diode dimensions

### Process Parameters
- `cu2cu_margin`: Copper-to-copper spacing
- `cu2ceramics_margin`: Copper-to-ceramic margin
- `substrate_solder_thickness`: Substrate solder thickness

### More parameters detailed in DATASET.md

## 📄 License

This project is open source under the MIT License - see the [LICENSE](LICENSE) file for details

## 📞 Contact

For questions or suggestions, please provide feedback through Issues or contact the project maintainers.

## 🙏 Acknowledgments

Thanks to all researchers and developers who contributed data and code to this project.

---

**Keywords**: power module design, CAD modeling, CadQuery, large model fine-tuning dataset 