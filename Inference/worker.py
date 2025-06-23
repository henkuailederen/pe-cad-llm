#!/usr/bin/env python3
"""
后台工作进程 (Worker)
- 监控任务队列并执行CadQuery代码
"""

import os
import time
import json
import subprocess

# --- 配置 ---
CQ_PYTHON = r"C:\Users\User\cq-editor\python.exe"
TASKS_DIR = "tasks"
MODELS_DIR = "generated_models"
CODES_DIR = "generated_codes"

os.makedirs(TASKS_DIR, exist_ok=True)
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(CODES_DIR, exist_ok=True)

def find_new_task():
    """查找一个待处理的任务"""
    for task_id in os.listdir(TASKS_DIR):
        status_file = os.path.join(TASKS_DIR, task_id, "status.json")
        if os.path.exists(status_file):
            with open(status_file, 'r', encoding='utf-8') as f:
                status_data = json.load(f)
            if status_data['status'] == 'pending':
                return task_id, status_data
    return None, None

def update_task_status(task_id, new_status, data=None):
    """更新任务状态"""
    status_file = os.path.join(TASKS_DIR, task_id, "status.json")
    try:
        with open(status_file, 'r+', encoding='utf-8') as f:
            status_data = json.load(f)
            status_data['status'] = new_status
            if data:
                status_data.update(data)
            f.seek(0)
            json.dump(status_data, f, indent=4)
            f.truncate()
    except Exception as e:
        print(f"[Worker] Error updating status for {task_id}: {e}")

def execute_task(task_id, task_data):
    """执行一个任务"""
    original_code_file = task_data['code_file']
    stl_file = task_data['stl_file']
    
    print(f"[Worker] 🚀 Starting task {task_id}")
    update_task_status(task_id, 'running')
    
    try:
        # 读取原始代码
        with open(original_code_file, 'r', encoding='utf-8') as f:
            python_code = f.read()

        # --- 注入STL导出逻辑 ---
        modified_code = f"""# -*- coding: utf-8 -*-
import sys
import os

# 强制设置标准输出为UTF-8
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

{python_code}

# --- 自动导出3D模型 ---
try:
    import cadquery as cq
    
    if 'module' in globals():
        module_obj = globals()['module']
        if hasattr(module_obj, 'toCompound'):
            shape = module_obj.toCompound()
        else:
            shape = module_obj
    elif 'result' in globals():
        shape = globals()['result']
    elif 'assembly' in globals():
        shape = globals()['assembly']
    else:
        shape = None

    if shape:
        print(f"Exporting object of type: {{type(shape)}}")
        cq.exporters.export(shape, r'{stl_file}')
        print("STL exported successfully to: {stl_file}")
    else:
        print("No exportable result (module, result, assembly) found.")
        
except Exception as e:
    import traceback
    print(f"Export error: {{e}}")
    traceback.print_exc()
"""
        # 将修改后的代码写入文件
        with open(original_code_file, 'w', encoding='utf-8') as f:
            f.write(modified_code)

        cmd = [CQ_PYTHON, original_code_file]
        
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding='utf-8',
            errors='ignore',
            cwd=os.getcwd()
        )
        
        stdout, stderr = process.communicate(timeout=300) # 5分钟超时
        
        log_file = os.path.join(TASKS_DIR, task_id, "output.log")
        with open(log_file, 'w', encoding='utf-8') as f:
            f.write("--- STDOUT ---\n")
            f.write(stdout)
            f.write("\n--- STDERR ---\n")
            f.write(stderr)

        if process.returncode == 0 and "STL exported successfully" in stdout:
            print(f"[Worker] ✅ Task {task_id} completed successfully")
            update_task_status(task_id, 'completed', {
                'has_3d_model': True,
                'model_url': f'/api/model/{task_id}',
                'download_url': f'/api/download/{task_id}',
                'log_file': log_file
            })
        else:
            # 即使返回码为0，如果没能导出STL，也算失败
            raise Exception(f"Return code: {process.returncode}\n---STDOUT---\n{stdout}\n---STDERR---\n{stderr}")

    except Exception as e:
        print(f"[Worker] ❌ Task {task_id} failed: {e}")
        update_task_status(task_id, 'failed', {
            'error': str(e),
            'has_3d_model': False
        })

def main():
    """工作进程主循环"""
    print("[Worker] CadQuery Worker started. Waiting for tasks...")
    # 复制utils.py到generated_codes目录
    utils_source = "utils.py"
    utils_target = os.path.join(CODES_DIR, "utils.py")
    if os.path.exists(utils_source) and not os.path.exists(utils_target):
        import shutil
        shutil.copy2(utils_source, utils_target)
        print(f"[Worker] Copied utils.py to {utils_target}")
        
    while True:
        task_id, task_data = find_new_task()
        if task_id:
            execute_task(task_id, task_data)
        else:
            time.sleep(2) # 每2秒检查一次

if __name__ == "__main__":
    main() 