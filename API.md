# API Documentation

This document describes the REST API interface for the frontend and backend system of power module design configuration.

## Basic Information

- **Base URL**: `http://localhost:5000`
- **Content-Type**: `application/json`
- **Encoding**: UTF-8

## API Overview

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Homepage |
| `/api/generate` | POST | Generate configuration |
| `/api/execute` | POST | Execute code |
| `/api/task_status/<task_id>` | GET | Check task status |
| `/api/model/<task_id>` | GET | Get 3D model |
| `/api/download/<task_id>` | GET | Download model file |
| `/api/launch_editor` | POST | Launch editor |
| `/api/check_connection` | POST | Check connection |

## Detailed API Description

### 1. Generate Configuration

Generate power module configuration based on natural language description.

**Endpoint**: `POST /api/generate`

**Request Body**:
```json
{
  "prompt": "Design a three-DBC half-bridge structure with each substrate containing two IGBTs and two freewheeling diodes..."
}
```

**Request Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| prompt | string | Yes | Power module design requirement description |

**Response Body**:
```json
{
  "success": true,
  "config": {
    "module_id": "HB_V2",
    "cu2cu_margin": 0.9,
    "cu2ceramics_margin": 1.35,
    "igbt_bondwires": 9,
    "fwd_bondwires": 9,
    "igbt_positions": {
      "type_1": [[0.22, 0.58]],
      "type_2": [[0.57, 0.27]]
    }
  },
  "generated_code": "# CadQuery code...",
  "message": "Configuration generated successfully"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": "API call failed: Connection timeout",
  "message": "Configuration generation failed"
}
```

**Status Codes**:
- `200`: Success
- `400`: Invalid request parameters
- `500`: Internal server error

---

### 2. Execute Code

Execute CadQuery code and generate 3D model.

**Endpoint**: `POST /api/execute`

**Request Body**:
```json
{
  "code": "# CadQuery Python code\nimport cadquery as cq\n..."
}
```

**Request Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| code | string | Yes | CadQuery code to execute |

**Response Body**:
```json
{
  "success": true,
  "task_id": "uuid-task-id",
  "message": "Task submitted, processing in background"
}
```

**Error Response**:
```json
{
  "success": false,
  "error": "Code syntax error",
  "message": "Code execution failed"
}
```

---

### 3. Check Task Status

Query the status of code execution task.

**Endpoint**: `GET /api/task_status/<task_id>`

**Path Parameters**:
| Parameter | Type | Description |
|-----------|------|-------------|
| task_id | string | Task ID |

**Response Body**:

**In Progress**:
```json
{
  "status": "running",
  "task_id": "uuid-task-id",
  "has_3d_model": false
}
```

**Completed**:
```json
{
  "status": "completed",
  "task_id": "uuid-task-id",
  "has_3d_model": true,
  "model_url": "/api/model/uuid-task-id",
  "download_url": "/api/download/uuid-task-id"
}
```

**Failed**:
```json
{
  "status": "failed",
  "task_id": "uuid-task-id",
  "error": "Execution error message",
  "has_3d_model": false
}
```

**Status Types**:
- `pending`: Awaiting processing
- `running`: Currently executing
- `completed`: Execution completed
- `failed`: Execution failed

---

### 4. Get 3D Model

Retrieve the generated 3D model file (for web display).

**Endpoint**: `GET /api/model/<task_id>`

**Path Parameters**:
| Parameter | Type | Description |
|-----------|------|-------------|
| task_id | string | Task ID |

**Response**:
- **Success**: Returns STL file (binary)
- **Failure**: 404 Not Found

**Headers**:
```
Content-Type: application/octet-stream
Content-Disposition: inline; filename="model.stl"
```

---

### 5. Download Model File

Download the generated 3D model file.

**Endpoint**: `GET /api/download/<task_id>`

**Path Parameters**:
| Parameter | Type | Description |
|-----------|------|-------------|
| task_id | string | Task ID |

**Response**:
- **Success**: Returns STL file (download)
- **Failure**: 404 Not Found

**Headers**:
```
Content-Type: application/octet-stream
Content-Disposition: attachment; filename="power_module_model.stl"
```

---

### 7. Check Connection

Check external API connection status.

**Endpoint**: `POST /api/check_connection`

**Request Body**:
```json
{
  "api_url": "https://api.example.com/v1/chat/completions" # This API endpoint should be replaced with the deployed fine-tuned model interface address
}
```

**Response Body**:
```json
{
  "success": true,
  "message": "API connection normal",
  "response_time": 245
}
```

## Version Updates

API versions will be updated based on functionality changes, with major changes recorded in CHANGELOG.md. 