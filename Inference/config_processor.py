#!/usr/bin/env python3
"""
配置处理器 - 负责默认配置与override的合并，以及模板融合
"""

import yaml
import os
import re
from copy import deepcopy
from jinja2 import Template
from typing import Dict, Any, List

# 扁平化override键到嵌套路径的映射
OVERRIDE_MAPPING = {
    # Basic module info
    "module_id": "template_id",
    
    # Geometry parameters
    "ceramics_width": "geometry.ceramics.width",
    "ceramics_length": "geometry.ceramics.length", 
    "ceramics_thickness": "geometry.ceramics.thickness",
    "upper_copper_thickness": "geometry.copper.upper_thickness",
    "lower_copper_thickness": "geometry.copper.lower_thickness",
    "fillet_radius": "geometry.fillet_radius",
    
    # Die parameters
    "igbt_width": "dies.igbt.size.width",
    "igbt_length": "dies.igbt.size.length",
    "fwd_width": "dies.fwd.size.width", 
    "fwd_length": "dies.fwd.size.length",
    
    # Margin parameters
    "cu2cu_margin": "margins.cu2cu",
    "cu2ceramics_margin": "margins.cu2ceramics",
    "dbc2dbc_margin": "margins.dbc2dbc",
    "substrate_edge_margin": "margins.substrate_edge",
    
    # Process parameters
    "substrate_solder_thickness": "process.solder.substrate",
    "die_solder_thickness": "process.solder.die",
    "die_thickness": "process.die_thickness",
    
    # Bondwire counts
    "igbt_bondwires": "counts.bondwires.igbt",
    "fwd_bondwires": "counts.bondwires.fwd",
    
    # DBC layout parameters (simplified)
    "dbc_count": "dbc_layout.count",
    
    # Gate design parameters (simplified structure will be handled separately)
    # Cutting design parameters (simplified structure will be handled separately)  
    # Position parameters (simplified structure will be handled separately)
    # Rotation parameters (simplified structure will be handled separately)
    # Connection parameters (simplified structure will be handled separately)
}

class ConfigProcessor:
    def __init__(self, defaults_dir: str = "defaults", templates_dir: str = "templates"):
        self.defaults_dir = defaults_dir
        self.templates_dir = templates_dir
        self.template_mapping = {
            "HB_V1": "HB_V1_default.yaml",
            "HB_V2": "HB_V2_default.yaml", 
            "HB_V3": "HB_V3_default.yaml",
            "HB_V4": "HB_V4_default.yaml",
            "HB_V2-2": "HB_V2-2_default.yaml",
            "3P6P_V1": "3P6P_V1_default.yaml",
            "3P6P_V2": "3P6P_V2_default.yaml"
        }

    def load_default_config(self, template_id: str) -> Dict[str, Any]:
        """加载默认配置文件"""
        if template_id not in self.template_mapping:
            raise ValueError(f"Unknown template ID: {template_id}")
        
        config_file = os.path.join(self.defaults_dir, self.template_mapping[template_id])
        with open(config_file, 'r', encoding='utf-8') as f:
            flat_config = yaml.safe_load(f)
        
        # 将扁平配置转换为嵌套结构
        return self.convert_flat_default_to_nested(flat_config)

    def convert_flat_default_to_nested(self, flat_config: Dict[str, Any]) -> Dict[str, Any]:
        """将扁平的default配置转换为嵌套结构以匹配模板期望"""
        nested = {}
        
        # 首先处理所有直接映射的扁平键
        for flat_key, value in flat_config.items():
            if flat_key in OVERRIDE_MAPPING:
                nested_path = OVERRIDE_MAPPING[flat_key]
                self._set_nested_value(nested, nested_path, value)
            elif flat_key == "module_id":
                nested["template_id"] = value
        
        # 处理复杂结构
        for flat_key, value in flat_config.items():
            # 处理门极设计参数
            if flat_key == "gate_design":
                if 'gate_design' not in nested:
                    nested['gate_design'] = {}
                if 'types' not in nested['gate_design']:
                    nested['gate_design']['types'] = {}
                
                for type_key, type_config in value.items():
                    type_id = type_key.replace('type_', '')
                    nested['gate_design']['types'][type_id] = {}
                    
                    if 'start' in type_config:
                        start_data = type_config['start']
                        # 检查是否是多个门极（嵌套列表格式）
                        if isinstance(start_data[0], list):
                            # 多个门极的情况 - 只使用第一个门极的数据，模板只支持单个门极
                            nested['gate_design']['types'][type_id]['start_point'] = {
                                'x_ratio': start_data[0][0],
                                'y_ratio': start_data[0][1]
                            }
                        else:
                            # 单个门极的情况
                            nested['gate_design']['types'][type_id]['start_point'] = {
                                'x_ratio': start_data[0],
                                'y_ratio': start_data[1]
                            }
                    
                    if 'moves' in type_config:
                        moves_data = type_config['moves']
                        # 检查是否是多个门极（三层嵌套列表格式）
                        if isinstance(moves_data[0][0], list):
                            # 多个门极的情况 - 只使用第一个门极的moves，模板只支持单个门极
                            nested['gate_design']['types'][type_id]['moves'] = []
                            for move in moves_data[0]:  # 只取第一组moves
                                nested['gate_design']['types'][type_id]['moves'].append({
                                    'x_ratio': move[0],
                                    'y_ratio': move[1]
                                })
                        else:
                            # 单个门极的情况
                            nested['gate_design']['types'][type_id]['moves'] = []
                            for move in moves_data:
                                nested['gate_design']['types'][type_id]['moves'].append({
                                    'x_ratio': move[0],
                                    'y_ratio': move[1]
                                })
            
            # 处理切割路径参数
            elif flat_key == "cutting_design":
                if 'cutting_design' not in nested:
                    nested['cutting_design'] = {}
                if 'paths' not in nested['cutting_design']:
                    nested['cutting_design']['paths'] = []
                
                path_index = 0
                for path_key, points in value.items():
                    path_data = {
                        'name': f"cut_{path_index + 1}",
                        'points': []
                    }
                    
                    for point in points:
                        x_val = point[0]
                        y_val = point[1]
                        
                        # 处理特殊值 MAX/MIN
                        if x_val == "MAX":
                            x_val = 999
                        elif x_val == "MIN":
                            x_val = -999
                        if y_val == "MAX":
                            y_val = 999
                        elif y_val == "MIN":
                            y_val = -999
                            
                        path_data['points'].append({
                            'x_ratio': x_val,
                            'y_ratio': y_val
                        })
                    
                    nested['cutting_design']['paths'].append(path_data)
                    path_index += 1
            
            # 处理IGBT位置参数
            elif flat_key == "igbt_positions":
                nested['igbt_positions'] = value
            
            # 处理FWD位置参数
            elif flat_key == "fwd_positions":
                nested['fwd_positions'] = value
            
            # 处理IGBT旋转参数
            elif flat_key == "igbt_rotations":
                nested['igbt_rotations'] = value
            
            # 处理FWD旋转参数
            elif flat_key == "fwd_rotations":
                nested['fwd_rotations'] = value
            
            # 处理DBC旋转参数
            elif flat_key == "dbc_rotations":
                nested['dbc_rotations'] = value
            
            # 处理模块连接参数
            elif flat_key == "module_connections":
                if 'topology' not in nested:
                    nested['topology'] = {}
                
                module_entities = set()
                nested['topology']['module_connections'] = []
                
                for conn in value:
                    source = conn[0].replace('zone', 'Zone_')
                    target = conn[1].replace('zone', 'Zone_')
                    module_entities.add(source)
                    module_entities.add(target)
                    nested['topology']['module_connections'].append({
                        'source': source,
                        'target': target,
                        'value': 1
                    })
                
                nested['topology']['module_entities'] = sorted(list(module_entities))
            
            # 处理DBC连接参数
            elif flat_key == "dbc_connections":
                if 'topology' not in nested:
                    nested['topology'] = {}
                
                dbc_entities = set()
                nested['topology']['dbc_connections'] = []
                
                for conn in value:
                    source_raw = str(conn[0]).lower()
                    target_raw = str(conn[1]).lower()
                    
                    # 实体命名转换
                    if source_raw.startswith('igbt'):
                        source = source_raw.replace('igbt', 'IGBT_')
                    elif source_raw.startswith('fwd'):
                        source = source_raw.replace('fwd', 'FWD_')
                    elif source_raw.startswith('zone'):
                        source = source_raw.replace('zone', 'Zone_')
                    else:
                        source = source_raw
                    
                    if target_raw.startswith('igbt'):
                        target = target_raw.replace('igbt', 'IGBT_')
                    elif target_raw.startswith('fwd'):
                        target = target_raw.replace('fwd', 'FWD_')
                    elif target_raw.startswith('zone'):
                        target = target_raw.replace('zone', 'Zone_')
                    else:
                        target = target_raw
                    
                    dbc_entities.add(source)
                    dbc_entities.add(target)
                    nested['topology']['dbc_connections'].append({
                        'source': source,
                        'target': target,
                        'value': 1
                    })
                
                nested['topology']['dbc_entities'] = sorted(list(dbc_entities))
        
        return nested

    def _set_nested_value(self, nested_dict: Dict[str, Any], path: str, value: Any):
        """在嵌套字典中设置值，支持数组索引访问"""
        keys = path.split('.')
        current = nested_dict
        
        for i, key in enumerate(keys[:-1]):
            # 检查是否是数组索引
            if key.isdigit():
                key = int(key)
                # 确保当前节点是列表
                if not isinstance(current, list):
                    # 如果父级需要初始化为列表
                    parent_key = keys[i-1] if i > 0 else None
                    if parent_key:
                        current = []
                        return  # 需要重新构建结构
                
                # 确保列表足够长
                while len(current) <= key:
                    current.append({})
                current = current[key]
            else:
                if key not in current:
                    current[key] = {}
                current = current[key]
        
        final_key = keys[-1]
        if final_key.isdigit():
            final_key = int(final_key)
            if not isinstance(current, list):
                current = []
            while len(current) <= final_key:
                current.append(None)
            current[final_key] = value
        else:
            current[final_key] = value

    def deep_merge(self, base_config: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
        """深度合并配置，overrides会覆盖base_config中的对应字段"""
        result = deepcopy(base_config)
        
        def _merge_dict(base_dict: Dict[str, Any], override_dict: Dict[str, Any]):
            for key, value in override_dict.items():
                if key in base_dict:
                    if isinstance(base_dict[key], dict) and isinstance(value, dict):
                        _merge_dict(base_dict[key], value)
                    else:
                        base_dict[key] = value
                else:
                    base_dict[key] = value
        
        def _normalize_keys(obj):
            """递归标准化键，将数字字符串键转换为整数键以保持一致性"""
            if isinstance(obj, dict):
                normalized = {}
                for key, value in obj.items():
                    # 如果键是数字字符串，尝试转换为整数
                    if isinstance(key, str) and key.isdigit():
                        new_key = int(key)
                    else:
                        new_key = key
                    normalized[new_key] = _normalize_keys(value)
                return normalized
            elif isinstance(obj, list):
                return [_normalize_keys(item) for item in obj]
            else:
                return obj
        
        _merge_dict(result, overrides)
        
        # 标准化键
        result = _normalize_keys(result)
        
        return result

    def apply_overrides(self, template_id: str, overrides_data: Dict[str, Any]) -> Dict[str, Any]:
        """应用覆盖配置到默认模板 - 保持default的扁平格式"""
        # 加载默认配置（保持扁平格式）
        base_config = self.load_default_flat_config(template_id)
        
        # 提取实际的overrides部分 - 排除元数据字段
        metadata_fields = ['module_id', 'base_template', 'description', 'reasoning']
        flat_overrides = {k: v for k, v in overrides_data.items() 
                         if k not in metadata_fields}
        
        # 直接在扁平结构上进行覆盖，不转换为嵌套
        final_config = deepcopy(base_config)
        for key, value in flat_overrides.items():
            final_config[key] = value
        
        # 保留元信息
        final_config['_metadata'] = {
            'base_template': template_id,
            'description': overrides_data.get('description', base_config.get('description', '')),
            'reasoning': overrides_data.get('reasoning', '')
        }
        
        return final_config

    def load_default_flat_config(self, template_id: str) -> Dict[str, Any]:
        """加载默认配置文件，保持扁平格式"""
        if template_id not in self.template_mapping:
            raise ValueError(f"Unknown template ID: {template_id}")
        
        config_file = os.path.join(self.defaults_dir, self.template_mapping[template_id])
        with open(config_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def render_template(self, config: Dict[str, Any], template_file: str = "unified_template.py.j2") -> str:
        """使用配置渲染模板 - 在渲染时将扁平配置转换为嵌套格式"""
        template_path = os.path.join(self.templates_dir, template_file)
        
        with open(template_path, 'r', encoding='utf-8') as f:
            template_content = f.read()
        
        # 将扁平配置转换为嵌套格式以用于模板渲染
        nested_config = self.convert_flat_default_to_nested(config)
        
        # 创建Jinja2模板
        template = Template(template_content)
        
        # 渲染模板
        rendered = template.render(**nested_config)
        
        return rendered

    def process_override_file(self, override_file_path: str) -> tuple[Dict[str, Any], str]:
        """处理override文件，返回最终配置和渲染的代码"""
        # 读取override配置
        with open(override_file_path, 'r', encoding='utf-8') as f:
            override_data = yaml.safe_load(f)
        
        # 支持新的module_id字段和旧的base_template字段
        template_id = override_data.get('module_id') or override_data.get('base_template')
        if not template_id:
            raise ValueError("Override file must specify 'module_id' or 'base_template'")
        
        # 应用覆盖
        final_config = self.apply_overrides(template_id, override_data)
        
        # 渲染模板
        rendered_code = self.render_template(final_config)
        
        return final_config, rendered_code

    def save_config(self, config: Dict[str, Any], output_path: str):
        """保存配置到文件 - 严格按照default模板的格式"""
        # 获取模板ID以确定格式
        template_id = config.get('module_id', 'HB_V3')
        
        # 构建格式化的配置内容
        formatted_content = self.format_config_like_default(config, template_id)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(formatted_content)

    def format_config_like_default(self, config: Dict[str, Any], template_id: str) -> str:
        """严格按照default模板的格式来格式化配置"""
        lines = []
        
        # 按照HB_V3_default.yaml的确切格式和顺序
        lines.append(f'module_id: "{config.get("module_id", template_id)}"')
        lines.append('')
        
        # 基础几何参数
        lines.append('# 基础几何参数')
        lines.append(f'ceramics_width: {config.get("ceramics_width", 44.0)}')
        lines.append(f'ceramics_length: {config.get("ceramics_length", 48.0)}')
        lines.append(f'ceramics_thickness: {config.get("ceramics_thickness", 0.8)}')
        lines.append(f'upper_copper_thickness: {config.get("upper_copper_thickness", 0.2)}')
        lines.append(f'lower_copper_thickness: {config.get("lower_copper_thickness", 0.3)}')
        lines.append(f'fillet_radius: {config.get("fillet_radius", 0.5)}')
        lines.append('')
        
        # 芯片尺寸参数
        lines.append('# 芯片尺寸参数')
        lines.append(f'igbt_width: {config.get("igbt_width", 10)}')
        lines.append(f'igbt_length: {config.get("igbt_length", 10)}')
        lines.append(f'fwd_width: {config.get("fwd_width", 5)}')
        lines.append(f'fwd_length: {config.get("fwd_length", 5)}')
        lines.append('')
        
        # 边距参数
        lines.append('# 边距参数')
        lines.append(f'cu2cu_margin: {config.get("cu2cu_margin", 1.0)}')
        lines.append(f'cu2ceramics_margin: {config.get("cu2ceramics_margin", 1.5)}')
        lines.append(f'dbc2dbc_margin: {config.get("dbc2dbc_margin", 3.0)}')
        lines.append(f'substrate_edge_margin: {config.get("substrate_edge_margin", 3.0)}')
        lines.append('')
        
        # 工艺参数
        lines.append('# 工艺参数')
        lines.append(f'substrate_solder_thickness: {config.get("substrate_solder_thickness", 0.15)}')
        lines.append(f'die_solder_thickness: {config.get("die_solder_thickness", 0.1)}')
        lines.append(f'die_thickness: {config.get("die_thickness", 0.15)}')
        lines.append('')
        
        # 键合线数量
        lines.append('# 键合线数量')
        lines.append(f'igbt_bondwires: {config.get("igbt_bondwires", 6)}')
        lines.append(f'fwd_bondwires: {config.get("fwd_bondwires", 6)}')
        lines.append('')
        
        # 门极设计
        lines.append('# 门极设计 - 修正为原始文件的精确比例')
        gate_design = config.get("gate_design", {})
        lines.append('gate_design:')
        for type_key, type_config in gate_design.items():
            lines.append(f'  {type_key}:')
            if 'start' in type_config:
                start = type_config['start']
                lines.append(f'    start: [{start[0]}, {start[1]}]  # distance_Cu_Ceramics + 0.1*length_ceramics 的比例')
            if 'moves' in type_config:
                moves = type_config['moves']
                moves_str = ', '.join([f'[{move[0]}, {move[1]}]' for move in moves])
                lines.append(f'    moves: [{moves_str}]  # 0.04*width_ceramics = 1.76, 比例 = 1.76/44 ≈ 0.04')
        lines.append('')
        
        # 切割路径设计
        lines.append('# 切割路径设计 - 修正为原始文件的精确比例')
        cutting_design = config.get("cutting_design", {})
        lines.append('cutting_design:')
        for path_key, points in cutting_design.items():
            points_str = ', '.join([f'[{point[0]}, {point[1]}]' for point in points])
            lines.append(f'  {path_key}: [{points_str}]')
        lines.append('')
        
        # IGBT位置
        lines.append('# IGBT位置（3个IGBT，都是type_1）')
        igbt_positions = config.get("igbt_positions", {})
        lines.append('igbt_positions:')
        for type_key, positions in igbt_positions.items():
            positions_str = ', '.join([f'[{pos[0]}, {pos[1]}]' for pos in positions])
            lines.append(f'  {type_key}: [{positions_str}]')
        lines.append('')
        
        # FWD位置
        lines.append('# FWD位置（3个FWD）')
        fwd_positions = config.get("fwd_positions", [])
        positions_str = ', '.join([f'[{pos[0]}, {pos[1]}]' for pos in fwd_positions])
        lines.append(f'fwd_positions: [{positions_str}]')
        lines.append('')
        
        # 旋转角度
        lines.append('# 旋转角度')
        igbt_rotations = config.get("igbt_rotations", {})
        lines.append('igbt_rotations:')
        for type_key, rotations in igbt_rotations.items():
            rotations_str = ', '.join([str(rot) for rot in rotations])
            lines.append(f'  {type_key}: [{rotations_str}]')
        lines.append('')
        
        fwd_rotations = config.get("fwd_rotations", [])
        rotations_str = ', '.join([str(rot) for rot in fwd_rotations])
        lines.append(f'fwd_rotations: [{rotations_str}]')
        lines.append('')
        
        # DBC布局
        lines.append('# DBC布局')
        lines.append(f'dbc_count: {config.get("dbc_count", 2)}')
        dbc_rotations = config.get("dbc_rotations", [])
        rotations_str = ', '.join([str(rot) for rot in dbc_rotations])
        lines.append(f'dbc_rotations: [{rotations_str}]')
        lines.append('')
        
        # 模块连接
        lines.append('# 模块连接（根据原始connection_module矩阵）')
        module_connections = config.get("module_connections", [])
        if module_connections:
            connections_str = ', '.join([f'["{conn[0]}", "{conn[1]}"]' for conn in module_connections])
            lines.append(f'module_connections: [{connections_str}]')
        else:
            lines.append('module_connections: []')
        lines.append('')
        
        # DBC连接
        lines.append('# DBC连接（根据原始connection_DBC矩阵）')
        dbc_connections = config.get("dbc_connections", [])
        if dbc_connections:
            lines.append('dbc_connections: [')
            # 按照default的格式：每4个连接一行，逗号在行末
            for i in range(0, len(dbc_connections), 4):
                batch = dbc_connections[i:i+4]
                conn_strs = [f'["{conn[0]}", "{conn[1]}"]' for conn in batch]
                line_content = ', '.join(conn_strs)
                
                # 如果不是最后一批，在行末添加逗号
                if i + 4 < len(dbc_connections):
                    line_content += ','
                
                lines.append(f'  {line_content}')
            lines.append(']')
        else:
            lines.append('dbc_connections: []')
        
        return '\n'.join(lines)

    def save_code(self, code: str, output_path: str):
        """保存渲染的代码到文件"""
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(code)

    def validate_override(self, override_data: Dict[str, Any]) -> List[str]:
        """验证override配置的有效性"""
        errors = []
        
        # 检查必需字段 - 支持新旧两种字段名
        template_id = override_data.get('module_id') or override_data.get('base_template')
        if not template_id:
            errors.append("Missing required field: 'module_id' or 'base_template'")
        elif template_id not in self.template_mapping:
            errors.append(f"Invalid module_id/base_template: {template_id}")
        
        # 可以添加更多验证规则...
        
        return errors

    def flatten_override_to_nested(self, flat_overrides: Dict[str, Any]) -> Dict[str, Any]:
        """将扁平化的override转换为嵌套结构 - 支持新的简洁格式"""
        nested = {}
        
        for flat_key, value in flat_overrides.items():
            # 处理标准映射
            if flat_key in OVERRIDE_MAPPING:
                nested_path = OVERRIDE_MAPPING[flat_key]
                self._set_nested_value(nested, nested_path, value)
            elif flat_key == "module_id":
                nested["template_id"] = value
            
            # 处理门极设计参数
            elif flat_key == "gate_design":
                if 'gate_design' not in nested:
                    nested['gate_design'] = {}
                if 'types' not in nested['gate_design']:
                    nested['gate_design']['types'] = {}
                
                for type_key, type_config in value.items():
                    type_id = type_key.replace('type_', '')
                    nested['gate_design']['types'][type_id] = {}
                    
                    if 'start' in type_config:
                        start_data = type_config['start']
                        # 检查是否是多个门极（嵌套列表格式）
                        if isinstance(start_data[0], list):
                            # 多个门极的情况
                            nested['gate_design']['types'][type_id]['start_points'] = []
                            for start_point in start_data:
                                nested['gate_design']['types'][type_id]['start_points'].append({
                                    'x_ratio': start_point[0],
                                    'y_ratio': start_point[1]
                                })
                        else:
                            # 单个门极的情况
                            nested['gate_design']['types'][type_id]['start_point'] = {
                                'x_ratio': start_data[0],
                                'y_ratio': start_data[1]
                            }
                    
                    if 'moves' in type_config:
                        moves_data = type_config['moves']
                        # 检查是否是多个门极（三层嵌套列表格式）
                        if isinstance(moves_data[0][0], list):
                            # 多个门极的情况：每个门极有一组moves
                            nested['gate_design']['types'][type_id]['moves_list'] = []
                            for move_group in moves_data:
                                move_list = []
                                for move in move_group:
                                    move_list.append({
                                        'x_ratio': move[0],
                                        'y_ratio': move[1]
                                    })
                                nested['gate_design']['types'][type_id]['moves_list'].append(move_list)
                        else:
                            # 单个门极的情况
                            nested['gate_design']['types'][type_id]['moves'] = []
                            for move in moves_data:
                                nested['gate_design']['types'][type_id]['moves'].append({
                                    'x_ratio': move[0],
                                    'y_ratio': move[1]
                                })
            
            # 处理切割路径参数
            elif flat_key == "cutting_design":
                if 'cutting_design' not in nested:
                    nested['cutting_design'] = {}
                if 'paths' not in nested['cutting_design']:
                    nested['cutting_design']['paths'] = []
                
                path_index = 0
                for path_key, points in value.items():
                    path_data = {
                        'name': f"cut_{path_index + 1}",
                        'points': []
                    }
                    
                    for point in points:
                        x_val = point[0]
                        y_val = point[1]
                        
                        # 处理特殊值 MAX/MIN
                        if x_val == "MAX":
                            x_val = 999
                        elif x_val == "MIN":
                            x_val = -999
                        if y_val == "MAX":
                            y_val = 999
                        elif y_val == "MIN":
                            y_val = -999
                            
                        path_data['points'].append({
                            'x_ratio': x_val,
                            'y_ratio': y_val
                        })
                    
                    nested['cutting_design']['paths'].append(path_data)
                    path_index += 1
            
            # 处理IGBT位置参数
            elif flat_key == "igbt_positions":
                nested['igbt_positions'] = value
            
            # 处理FWD位置参数
            elif flat_key == "fwd_positions":
                nested['fwd_positions'] = value
            
            # 处理IGBT旋转参数
            elif flat_key == "igbt_rotations":
                nested['igbt_rotations'] = value
            
            # 处理FWD旋转参数
            elif flat_key == "fwd_rotations":
                nested['fwd_rotations'] = value
            
            # 处理DBC旋转参数
            elif flat_key == "dbc_rotations":
                nested['dbc_rotations'] = value
            
            # 其他参数保持原样
            else:
                nested[flat_key] = value
        
        return nested

# 辅助函数，用于模板中使用
def build_connection_matrix(connections: List[Dict[str, Any]], entities: List[str]) -> Dict[str, Dict[str, int]]:
    """从连接列表构建连接矩阵"""
    matrix = {}
    
    # 初始化矩阵，所有连接值为0
    for entity in entities:
        matrix[entity] = {e: 0 for e in entities}
    
    # 填入实际连接
    for conn in connections:
        source = conn["source"]
        target = conn["target"]
        value = conn.get("value", 1)
        
        if source in matrix and target in matrix[source]:
            matrix[source][target] = value
    
    return matrix

def build_igbt_configs(setup: Dict[str, Any], positions: List[List[float]]) -> Dict[int, Dict[str, Any]]:
    """从参数构建IGBT配置"""
    configs = {}
    
    for type_id_str, type_config in setup["types"].items():
        type_id = int(type_id_str)
        igbt_positions = []
        
        for idx in type_config["positions"]:
            pos = positions[idx]
            igbt_positions.append(f"Vector({pos[0]:.4f}, {pos[1]:.4f}, 0)")
        
        configs[type_id] = {
            "positions": igbt_positions,
            "rotations": type_config["rotations"]
        }
    
    return configs

def build_fwd_positions(setup: Dict[str, Any], positions: List[List[float]]) -> List[str]:
    """构建FWD位置列表"""
    fwd_positions = []
    
    for idx in setup["positions"]:
        pos = positions[idx]
        fwd_positions.append(f"Vector({pos[0]:.4f}, {pos[1]:.4f}, 0)")
    
    return fwd_positions

def build_gate_configs(design: Dict[str, Any], width_var: str, length_var: str, margin_var: str) -> Dict[int, Dict[str, Any]]:
    """构建门极配置"""
    configs = {}
    
    for type_id_str, type_config in design["types"].items():
        type_id = int(type_id_str)
        
        start_point = type_config["start_point"]
        start_x = f"{start_point['x_ratio']} * {width_var} + {margin_var}"
        start_y = f"{start_point['y_ratio']} * {length_var} + {margin_var}"
        
        moves = []
        for move in type_config["moves"]:
            move_x = f"{move['x_ratio']} * {width_var}"
            move_y = f"{move['y_ratio']} * {length_var}"
            moves.append(f"({move_x}, {move_y})")
        
        configs[type_id] = {
            "start_points": [f"({start_x}, {start_y})"],
            "relative_moves": [moves]
        }
    
    return configs

def build_cutting_paths(paths: List[Dict[str, Any]], width_var: str, length_var: str, margin_var: str, extreme_val: str) -> List[List[str]]:
    """构建切割路径"""
    cut_paths = []
    
    for path in paths:
        points = []
        for point in path["points"]:
            x_ratio = point["x_ratio"]
            y_ratio = point["y_ratio"]
            
            # 处理特殊值（EXTREME）
            if x_ratio == 999:
                x = extreme_val
            elif x_ratio == -999:
                x = f"-{extreme_val}"
            else:
                x = f"{x_ratio} * {width_var} + {margin_var}"
            
            if y_ratio == 999:
                y = extreme_val
            elif y_ratio == -999:
                y = f"-{extreme_val}"
            else:
                y = f"{y_ratio} * {length_var} + {margin_var}"
            
            points.append(f"({x}, {y})")
        
        cut_paths.append(points)
    
    return cut_paths

if __name__ == "__main__":
    # 示例用法
    processor = ConfigProcessor()
    
    # 测试加载默认配置
    try:
        config = processor.load_default_config("HB_V1")
        print("成功加载HB_V1默认配置")
        print(f"陶瓷宽度: {config['geometry']['ceramics']['width']}")
    except Exception as e:
        print(f"加载配置失败: {e}") 