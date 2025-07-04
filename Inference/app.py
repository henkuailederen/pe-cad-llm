#!/usr/bin/env python3
"""
前后端配置生成系统 - Flask 后端
"""

from flask import Flask, request, jsonify, render_template, send_file
from flask_cors import CORS
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import json
import traceback
from config_processor import ConfigProcessor
import os
import subprocess
import uuid
from datetime import datetime

app = Flask(__name__)
CORS(app)

# 配置处理器实例
processor = ConfigProcessor()

# 外部 API 配置 (修正后)
API_BASE_URL = "https://63b0-111-186-61-142.ngrok-free.app/v1/chat/completions" # 使用ngrok地址

# CadQuery 环境配置
CADQUERY_PYTHON = r"C:\Users\User\cq-editor\python.exe"  # CQ-editor Python可执行文件路径
MODELS_DIR = "generated_models"  # 生成模型存储目录
CODES_DIR = "generated_codes"    # 生成代码存储目录
TASKS_DIR = "tasks"

# 确保目录存在
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(CODES_DIR, exist_ok=True)
os.makedirs(TASKS_DIR, exist_ok=True)

# --- 新增：创建带重试功能的 requests session 的辅助函数 ---
def requests_session_with_retries(
    retries=3,
    backoff_factor=0.5,
    status_forcelist=(500, 502, 504),
    session=None,
):
    """
    创建一个配置了重试策略的requests session。
    这能让API调用在面对临时的网络错误时更加健壮。
    """
    session = session or requests.Session()
    retry = Retry(
        total=retries,
        read=retries,
        connect=retries,
        backoff_factor=backoff_factor,
        status_forcelist=status_forcelist,
        # 默认情况下，requests不会重试POST请求，因为它们不是幂等的。
        # 在我们的场景中，重试是安全的，所以我们显式地允许它。
        allowed_methods=frozenset(['POST', 'GET'])
    )
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)
    return session

# 全局变量，用于缓存API测试状态
api_connection_tested = False
api_connection_successful = False

def call_external_api(prompt: str) -> dict:
    """调用外部 LLaMA Factory API 获取 override 配置"""
    try:
        # LLaMA Factory 使用 OpenAI 兼容的 API 格式
        session = requests_session_with_retries(retries=3, backoff_factor=1) # 获取带重试的session
        response = session.post( # 使用 session.post 代替 requests.post
            API_BASE_URL, # 直接使用完整的URL
            json={
                "model": "test",
                "messages": [
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                "temperature": 0.7,
                "max_tokens": 2000
            },
            timeout=60
        )
        response.raise_for_status()
        
        # 解析LLaMA Factory响应
        api_response = response.json()
        
        # 从响应中提取生成的内容
        if 'choices' in api_response and len(api_response['choices']) > 0:
            generated_content = api_response['choices'][0]['message']['content']
            
            # 这里需要解析生成的内容，提取override配置
            # 假设AI返回的是YAML格式的配置
            try:
                import yaml
                override_data = yaml.safe_load(generated_content)
                return override_data  # 直接返回解析后的数据
            except:
                # 如果解析失败，返回包含描述的基本配置
                return {"description": generated_content}
        else:
            raise Exception("API响应格式异常")
            
    except requests.exceptions.RequestException as e:
        raise Exception(f"API调用失败: {str(e)}")

def execute_cadquery_code(python_code: str, code_id: str) -> dict:
    """在CadQuery环境中执行Python代码并生成3D模型"""
    try:
        # 保存Python代码到文件
        code_file = os.path.join(CODES_DIR, f"{code_id}.py")
        stl_file = os.path.join(MODELS_DIR, f"{code_id}.stl")
        
        # 确保utils.py在generated_codes目录中可用
        utils_source = "utils.py"
        utils_target = os.path.join(CODES_DIR, "utils.py")
        if os.path.exists(utils_source) and not os.path.exists(utils_target):
            import shutil
            shutil.copy2(utils_source, utils_target)
            print(f"Copied utils.py to {utils_target}")
        
        # 修改代码以添加STL导出和编码修正
        modified_code = f"""# -*- coding: utf-8 -*-
import sys
import os

# 强制设置标准输出为UTF-8，解决Windows下的编码问题
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

{python_code}

# 自动导出3D模型
try:
    import cadquery as cq
    
    # 检查module变量（Assembly对象）
    if 'module' in globals():
        module_obj = globals()['module']
        print(f"Module type: {{type(module_obj)}}")
        
        if hasattr(module_obj, 'toCompound'):
            # Assembly对象，转换为Compound再导出
            compound = module_obj.toCompound()
            cq.exporters.export(compound, r'{stl_file}')
            print("STL exported successfully (Assembly -> Compound)")
        elif hasattr(module_obj, 'val'):
            # Workplane对象
            shape = module_obj.val()
            cq.exporters.export(shape, r'{stl_file}')
            print("STL exported successfully (Workplane)")
        else:
            # 尝试直接导出
            cq.exporters.export(module_obj, r'{stl_file}')
            print("STL exported successfully (direct)")
    # 备选方案：检查其他可能的变量
    elif 'result' in globals():
        result_obj = globals()['result']
        cq.exporters.export(result_obj, r'{stl_file}')
        print("STL exported successfully (result)")
    elif 'assembly' in globals():
        assembly_obj = globals()['assembly']
        if hasattr(assembly_obj, 'toCompound'):
            compound = assembly_obj.toCompound()
            cq.exporters.export(compound, r'{stl_file}')
            print("STL exported successfully (assembly -> Compound)")
        else:
            cq.exporters.export(assembly_obj, r'{stl_file}')
            print("STL exported successfully (assembly)")
    else:
        print("No exportable result found - 未找到可导出的对象")
        
except Exception as e:
    print(f"Export error: {{e}}")
"""
        
        with open(code_file, 'w', encoding='utf-8') as f:
            f.write(modified_code)
        
        # 直接使用CQ-editor的Python可执行文件
        cmd = [CADQUERY_PYTHON, code_file]
        
        result = subprocess.run(
            cmd, 
            capture_output=True, 
            text=True, 
            timeout=120, # 增加超时时间
            cwd=os.getcwd(),
            encoding='utf-8', # 明确指定编码
            errors='ignore'   # 忽略任何剩余的编码错误
        )
        
        # 检查是否生成了STL文件
        stl_exists = os.path.exists(stl_file)
        
        return {
            'success': result.returncode == 0,
            'stdout': result.stdout,
            'stderr': result.stderr,
            'code_file': code_file,
            'stl_file': stl_file if stl_exists else None,
            'has_3d_model': stl_exists
        }
        
    except subprocess.TimeoutExpired:
        return {
            'success': False,
            'error': '代码执行超时（超过60秒）',
            'has_3d_model': False
        }
    except Exception as e:
        return {
            'success': False,
            'error': f'执行失败: {str(e)}',
            'has_3d_model': False
        }

def launch_cq_editor(code_file: str) -> bool:
    """启动CQ-editor并打开指定文件"""
    try:
        # 检查是否有cq-editor可执行文件
        cq_editor_path = r"C:\Users\User\cq-editor\Scripts\cq-editor.exe"
        if os.path.exists(cq_editor_path):
            cmd = [cq_editor_path, code_file]
        else:
            # 备选方案：使用Python运行cq-editor模块
            cmd = [CADQUERY_PYTHON, "-m", "cq_editor", code_file]
        
        subprocess.Popen(cmd)
        return True
    except Exception as e:
        print(f"启动CQ-editor失败: {e}")
        return False

def _test_api_connection():
    """一个内部函数，用于测试与LLaMA Factory的连接。"""
    global api_connection_tested, api_connection_successful
    
    # 如果已经测试过并且成功，就不再测试
    if api_connection_tested and api_connection_successful:
        return {'success': True}

    try:
        # 使用一个非常短的超时和简单的请求来测试
        session = requests_session_with_retries(retries=3, backoff_factor=0.5) # 获取带重试的session
        response = session.post( # 使用 session.post 代替 requests.post
            API_BASE_URL, # 直接使用完整的URL
            json={
                "model": "test",
                "messages": [{"role": "user", "content": "hello"}],
                "max_tokens": 1
            },
            timeout=(5, 20) # (连接超时5秒, 读取超时20秒)
        )
        response.raise_for_status()
        
        # 只要能收到200 OK，就认为连接成功
        api_connection_tested = True
        api_connection_successful = True
        return {'success': True}
        
    except requests.exceptions.RequestException as e:
        api_connection_tested = True
        api_connection_successful = False
        error_message = f"无法连接到 LLaMA Factory API ({API_BASE_URL})。请检查网络或API服务状态。错误: {str(e)}"
        return {'success': False, 'error': error_message}

@app.route('/')
def index():
    """主页面"""
    return render_template('index.html')

@app.route('/api/generate', methods=['POST'])
def generate_code():
    """根据 prompt 生成最终的 Python 代码"""
    try:
        # 步骤 0: 检查API连接 (如果需要)
        connection_test = _test_api_connection()
        if not connection_test['success']:
            return jsonify(connection_test), 500

        data = request.json
        prompt = data.get('prompt', '')
        
        if not prompt:
            return jsonify({
                'success': False,
                'error': 'prompt 不能为空'
            }), 400
        
        # 步骤1: 调用外部API获取override配置
        try:
            override_data = call_external_api(prompt)  # 直接使用返回值
        except Exception as e:
            return jsonify({
                'success': False,
                'error': f'生成override配置失败: {str(e)}'
            }), 500
        
        # 步骤2: 从override数据中自动确定模板ID
        template_id = override_data.get('module_id') or override_data.get('base_template')
        if not template_id:
            # 如果没有指定模板，使用默认模板
            template_id = 'HB_V1'
            override_data['module_id'] = template_id
        
        # 验证模板ID
        if template_id not in processor.template_mapping:
            return jsonify({
                'success': False,
                'error': f'外部API返回的模板ID无效: {template_id}',
                'override_data': override_data
            }), 400
        
        # 步骤3: 应用override到默认配置
        try:
            merged_config = processor.apply_overrides(template_id, override_data)
        except Exception as e:
            return jsonify({
                'success': False,
                'error': f'配置合并失败: {str(e)}',
                'override_data': override_data
            }), 500
        
        # 步骤4: 渲染模板生成Python代码
        try:
            python_code = processor.render_template(merged_config)
        except Exception as e:
            return jsonify({
                'success': False,
                'error': f'模板渲染失败: {str(e)}',
                'override_data': override_data
            }), 500
        
        return jsonify({
            'success': True,
            'python_code': python_code,
            'override_data': override_data,
            'template_id': template_id
        })
        
    except Exception as e:
        print(f"生成代码时发生错误: {traceback.format_exc()}")
        return jsonify({
            'success': False,
            'error': f'服务器内部错误: {str(e)}'
        }), 500

@app.route('/api/execute', methods=['POST'])
def execute_code():
    """提交一个CadQuery代码执行任务到队列"""
    try:
        data = request.json
        python_code = data.get('python_code', '')
        
        if not python_code:
            return jsonify({'success': False, 'error': 'Python代码不能为空'}), 400
        
        # 1. 创建唯一的任务ID
        task_id = str(uuid.uuid4())
        
        # 2. 准备文件路径
        task_dir = os.path.join(TASKS_DIR, task_id)
        os.makedirs(task_dir, exist_ok=True)
        
        code_file = os.path.join(CODES_DIR, f"{task_id}.py")
        stl_file = os.path.join(MODELS_DIR, f"{task_id}.stl")
        status_file = os.path.join(task_dir, "status.json")

        # 3. 写入要执行的代码
        # (The worker will add the export logic)
        with open(code_file, 'w', encoding='utf-8') as f:
            f.write(python_code)

        # 4. 创建状态文件
        status_data = {
            'task_id': task_id,
            'status': 'pending', # pending, running, completed, failed
            'submitted_at': datetime.utcnow().isoformat() + 'Z',
            'code_file': code_file,
            'stl_file': stl_file,
            'has_3d_model': False
        }
        with open(status_file, 'w', encoding='utf-8') as f:
            json.dump(status_data, f, indent=4)
            
        # 5. 立即返回，告知前端任务已提交
        return jsonify({
            'success': True,
            'message': '任务已提交，正在排队等待执行...',
            'task_id': task_id
        })
        
    except Exception as e:
        print(f"提交任务时发生错误: {traceback.format_exc()}")
        return jsonify({'success': False, 'error': f'服务器内部错误: {str(e)}'}), 500

@app.route('/api/task_status/<task_id>')
def get_task_status(task_id):
    """获取指定任务的状态"""
    try:
        status_file = os.path.join(TASKS_DIR, task_id, "status.json")
        if not os.path.exists(status_file):
            return jsonify({'success': False, 'error': '任务不存在'}), 404
            
        with open(status_file, 'r', encoding='utf-8') as f:
            status_data = json.load(f)
            
        return jsonify({'success': True, 'status': status_data})

    except Exception as e:
        print(f"获取任务状态时发生错误: {traceback.format_exc()}")
        return jsonify({'success': False, 'error': f'服务器内部错误: {str(e)}'}), 500

@app.route('/api/model/<task_id>')
def serve_model(task_id):
    """提供STL模型文件"""
    try:
        stl_file = os.path.join(MODELS_DIR, f"{task_id}.stl")
        if os.path.exists(stl_file):
            return send_file(stl_file, mimetype='model/stl', as_attachment=False)
        else:
            return jsonify({'error': '模型文件不存在'}), 404
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/download/<task_id>')
def download_model(task_id):
    """下载STL模型文件"""
    try:
        stl_file = os.path.join(MODELS_DIR, f"{task_id}.stl")
        if os.path.exists(stl_file):
            return send_file(stl_file, mimetype='model/stl', as_attachment=True, 
                           download_name=f"model_{task_id}.stl")
        else:
            return jsonify({'error': '模型文件不存在'}), 404
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/launch_editor', methods=['POST'])
def launch_editor():
    """启动CQ-editor并打开代码文件"""
    try:
        data = request.json
        python_code = data.get('python_code', '')
        
        if not python_code:
            return jsonify({
                'success': False,
                'error': 'Python代码不能为空'
            }), 400
        
        # 生成唯一ID并保存代码
        code_id = str(uuid.uuid4())[:8]
        code_file = os.path.join(CODES_DIR, f"cq_editor_{code_id}.py")
        
        with open(code_file, 'w', encoding='utf-8') as f:
            f.write(python_code)
        
        # 启动CQ-editor
        success = launch_cq_editor(code_file)
        
        return jsonify({
            'success': success,
            'message': 'CQ-editor启动成功' if success else 'CQ-editor启动失败',
            'code_file': code_file
        })
        
    except Exception as e:
        print(f"启动CQ-editor时发生错误: {traceback.format_exc()}")
        return jsonify({
            'success': False,
            'error': f'启动失败: {str(e)}'
        }), 500

@app.route('/api/check_connection', methods=['POST'])
def check_connection():
    """一个专门用于前端按钮调用的API连接测试接口。"""
    result = _test_api_connection()
    return jsonify(result)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000) 